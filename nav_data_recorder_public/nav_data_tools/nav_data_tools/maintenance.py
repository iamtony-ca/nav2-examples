"""nav_data_maintenance: retention and the nightly report, started by the recorder's launch file.

  ros2 launch nav_data_recorder nav_data_recorder.launch.py ...      (starts it; maintenance:=false to not)
  ros2 run nav_data_tools nav_data_maintenance --root ~/nav_data --profile <profile> --once [--dry-run]

- Every prune_period_min: the retention rules of nav_data_prune (age, size cap, free disk).
- Once nightly_time has passed, nav_data_nightly for each of the last catch_up_days days that has
  recordings but no index yet (so a day missed while the robot was off is done after the next
  start), oldest first, one at a time, in a child process. The child waits while the robot drives
  (at most nightly_max_wait_h); if it gives up, that day is tried again at the next nightly_time.
Everything runs at the lowest CPU and disk priority. Nothing is installed in the OS (no systemd).
A wrong nightly setting only turns the nightly report off; retention keeps running.

Settings: the nav_data_maintenance section of the profile's params.yaml; command line options
override it.
"""
import argparse
import ctypes
import datetime as _dt
import fcntl
import os
import shutil
import signal
import subprocess
import sys
import time

import yaml

from nav_data_tools.common import DEFAULT_ROOT, append_event, day_range_ns, expand, list_rec_files
from nav_data_tools.prune import keep_fresh, prune

DEFAULTS = {
    'prune_period_min': 15.0,
    'keep_days': 7.0,
    'cap_gb': 60.0,
    'min_free': 0.15,
    'target_free': 0.20,
    'pinned_days': 90.0,
    'pinned_cap_gb': 10.0,
    'meta_days': 90.0,
    'nightly_time': '02:30',       # local time; '' = no nightly report
    'catch_up_days': 3,
    'nightly_max_wait_h': 6.0,     # nightly gives up after waiting this long for the robot to stop
    'nightly_retry_min': 60.0,     # next attempt after a nightly run that failed (not "gave up")
    'start_delay_s': 60.0,         # first run after start (let the robot stack come up first)
}
EXIT_GAVE_UP = 3                   # nav_data_nightly: robot kept driving, nothing written


def log(msg):
    print(f'[nav_data_maintenance] {msg}', flush=True)


def load_params(path, warn=log):
    """DEFAULTS updated with nav_data_maintenance.ros__parameters of a ROS params yaml.
    Unknown keys are reported and ignored (a typo must not stop retention)."""
    p = dict(DEFAULTS)
    if not path:
        return p
    with open(os.path.expanduser(path), encoding='utf-8') as f:
        doc = yaml.safe_load(f) or {}
    for key, val in doc.items():
        if str(key).lstrip('/').endswith('nav_data_maintenance') and isinstance(val, dict):
            for k, v in (val.get('ros__parameters') or {}).items():
                if k not in DEFAULTS:
                    warn(f'{path}: unknown nav_data_maintenance parameter {k!r} ignored')
                    continue
                p[k] = '' if v is None else v
    return p


def norm_time(v):
    """'HH:MM' from '2:30', '02:30', '02:30:00' or a YAML 1.1 sexagesimal int (unquoted 2:30 -> 150,
    02:30:00 -> 9000). '' / None / False -> ''. Raises ValueError for anything else."""
    if v is None or v is False or v == '':
        return ''
    if isinstance(v, bool):
        raise ValueError(f'nightly_time {v!r}')
    if isinstance(v, int):
        mins = v // 60 if v >= 24 * 60 else v
        h, m = divmod(mins, 60)
    else:
        parts = str(v).strip().split(':')
        if len(parts) not in (2, 3):
            raise ValueError(f'nightly_time {v!r}: expected "HH:MM"')
        h, m = int(parts[0]), int(parts[1])
    if not (0 <= h < 24 and 0 <= m < 60):
        raise ValueError(f'nightly_time {v!r}: expected "HH:MM"')
    return f'{h:02d}:{m:02d}'


def parse_hhmm(s):
    h, m = norm_time(s).split(':')
    return _dt.time(int(h), int(m))


def next_window(now, nightly_time):
    """Wall time of the next nightly_time after now."""
    at = parse_hhmm(nightly_time)
    t = _dt.datetime.combine(_dt.datetime.fromtimestamp(now).date(), at)
    if t.timestamp() <= now:
        t += _dt.timedelta(days=1)
    return t.timestamp()


def due_dates(files, index_dir, now, nightly_time, catch_up_days):
    """Days (YYYYMMDD, oldest first) to index now: day D is due once local time has passed
    D+1 nightly_time, it has recordings and index/D.jsonl does not exist yet."""
    if not nightly_time:
        return []
    at = parse_hhmm(nightly_time)
    today = _dt.datetime.fromtimestamp(now).date()
    out = []
    for back in range(int(catch_up_days), 0, -1):
        day = today - _dt.timedelta(days=back)
        ds = day.strftime('%Y%m%d')
        ready = _dt.datetime.combine(day + _dt.timedelta(days=1), at).timestamp()
        if now < ready or os.path.exists(os.path.join(index_dir, f'{ds}.jsonl')):
            continue
        s, e = day_range_ns(ds)
        if any(f.start_ns < e and f.end_ns > s for f in files):
            out.append(ds)
    return out


def nightly_cmd(root, date, profile, max_wait_h):
    cmd = [sys.executable, '-u', '-m', 'nav_data_tools.nightly', '--root', root, '--date', date,
           '--max-wait-h', str(max_wait_h)]
    if profile:
        cmd += ['--profile', profile]
    return cmd


def _child_setup():
    """In the nightly child: die with the parent (PR_SET_PDEATHSIG), even if it is SIGKILLed."""
    try:
        ctypes.CDLL('libc.so.6', use_errno=True).prctl(1, signal.SIGTERM)
    except (OSError, AttributeError):
        pass


def lowest_priority():
    try:
        os.nice(19 - os.nice(0))
    except OSError:
        pass
    if shutil.which('ionice'):
        subprocess.run(['ionice', '-c3', '-p', str(os.getpid())], check=False,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


class Maintenance:
    """Intervals use the monotonic clock; only nightly_time and calendar days use wall time."""

    def __init__(self, root, params, profile='', dry_run=False):
        self.root, self.p, self.profile, self.dry_run = root, params, profile, dry_run
        self.child = None            # (Popen, date)
        self.retry_mono = {}         # date -> monotonic time of the next attempt (after a failure)
        self.skip_until = {}         # date -> wall time (after "gave up": the next nightly window)
        self.next_prune = None       # monotonic

    def run_prune(self):
        p = self.p
        try:
            deleted = prune(self.root, days=float(p['keep_days']), cap_gb=float(p['cap_gb']),
                            min_free=float(p['min_free']), target_free=float(p['target_free']),
                            pinned_days=float(p['pinned_days']),
                            pinned_cap_gb=float(p['pinned_cap_gb']),
                            meta_days=float(p['meta_days']), dry_run=self.dry_run, log=log)
            if deleted:
                log(f'prune: {len(deleted)} item(s) {"would be " if self.dry_run else ""}deleted')
        except Exception as e:  # noqa: BLE001  (keep running; next period tries again)
            log(f'prune failed: {e}')
        if not self.dry_run:
            keep_fresh(self.root)

    def check_child(self, now, mono):
        if not self.child:
            return
        proc, date = self.child
        if proc.poll() is None:
            return
        self.child = None
        if os.path.exists(os.path.join(self.root, 'index', f'{date}.jsonl')):
            log(f'nightly {date}: done')
        elif proc.returncode == EXIT_GAVE_UP:
            self.skip_until[date] = next_window(now, self.p['nightly_time'])
            log(f'nightly {date}: robot kept driving; next try at {self.p["nightly_time"]}')
        else:
            self.retry_mono[date] = mono + float(self.p['nightly_retry_min']) * 60
            log(f'nightly {date}: failed (exit {proc.returncode}); retry in '
                f'{float(self.p["nightly_retry_min"]):g} min')

    def due(self, now, mono):
        files = list_rec_files(self.root, read_summary=False)
        return [d for d in due_dates(files, os.path.join(self.root, 'index'), now,
                                     self.p['nightly_time'], self.p['catch_up_days'])
                if mono >= self.retry_mono.get(d, 0) and now >= self.skip_until.get(d, 0)]

    def start_nightly(self, now, mono):
        if self.child or not self.p['nightly_time']:
            return
        days = self.due(now, mono)
        if not days:
            return
        if self.dry_run:
            log(f'[dry] nightly would run for {", ".join(days)}')
            return
        date = days[0]
        log(f'nightly {date}: start')
        self.child = (subprocess.Popen(nightly_cmd(self.root, date, self.profile,
                                                   self.p['nightly_max_wait_h']),
                                       start_new_session=True, preexec_fn=_child_setup), date)

    def tick(self, now=None, mono=None):
        now = time.time() if now is None else now
        mono = time.monotonic() if mono is None else mono
        try:
            if self.next_prune is None or mono >= self.next_prune:
                self.run_prune()
                self.next_prune = mono + float(self.p['prune_period_min']) * 60
            self.check_child(now, mono)
            self.start_nightly(now, mono)
        except Exception as e:  # noqa: BLE001  (never let one bad tick end the loop)
            log(f'error: {e}')

    def stop(self, grace_s=3.0):
        """Stop a running nightly child and its process group (within launch's SIGTERM budget)."""
        if not self.child or self.child[0].poll() is not None:
            return
        proc = self.child[0]
        for sig, wait in ((signal.SIGTERM, grace_s), (signal.SIGKILL, 2.0)):
            try:
                os.killpg(proc.pid, sig)
            except ProcessLookupError:
                return
            try:
                proc.wait(timeout=wait)
                return
            except subprocess.TimeoutExpired:
                continue


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('--root', default=os.environ.get('NAV_DATA_ROOT', DEFAULT_ROOT))
    p.add_argument('--params-file', default='',
                   help="ROS params yaml with a nav_data_maintenance section (default: the profile's)")
    p.add_argument('--profile', default='',
                   help="profile name or path for params and nightly rules (default: $NAV_DATA_PROFILE, else 'default')")
    p.add_argument('--once', action='store_true',
                   help='prune once and run the nightly report for due days, then exit')
    p.add_argument('--dry-run', action='store_true',
                   help='delete nothing and run no nightly report: only list what would be done')
    for k, v in DEFAULTS.items():
        p.add_argument('--' + k.replace('_', '-'), type=type(v) if not isinstance(v, str) else str,
                       default=None, help=f'override params (default {v!r})')
    argv = list(sys.argv[1:] if argv is None else argv)
    if '--ros-args' in argv:                 # added by launch_ros; not ours
        argv = argv[:argv.index('--ros-args')]
    a = p.parse_args(argv)
    root = expand(a.root)
    params_file = a.params_file
    try:
        if not params_file:
            from nav_data_tools.profile import profile_file
            params_file = profile_file('params.yaml', a.profile)
        params = load_params(params_file)
    except Exception as e:  # noqa: BLE001  (bad profile/params: retention with defaults still runs)
        log(f'cannot read params ({e}); using defaults')
        params = dict(DEFAULTS)
    for k in DEFAULTS:
        v = getattr(a, k)
        if v is not None:
            params[k] = v
    try:
        params['nightly_time'] = norm_time(params['nightly_time'])
    except ValueError as e:
        log(f'{e}; nightly report OFF (retention keeps running). Write it quoted, e.g. "02:30"')
        params['nightly_time'] = ''
    os.makedirs(root, exist_ok=True)
    lowest_priority()

    if not a.dry_run:                        # a dry run changes nothing: no lock needed
        lock = open(os.path.join(root, '.maintenance.lock'), 'w')
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            if a.once:
                log('nav_data_maintenance is already running for this root (started by the '
                    'recorder launch); use --dry-run to only look, or stop the launch first')
                return 1
            log('another nav_data_maintenance is running on this root; waiting for it to end')
            fcntl.flock(lock, fcntl.LOCK_EX)

    m = Maintenance(root, params, a.profile, a.dry_run)
    log(f'root {root}, params {params_file}: keep {params["keep_days"]} d / {params["cap_gb"]} GB / '
        f'free >= {params["min_free"]}, prune every {params["prune_period_min"]} min, '
        f'nightly {params["nightly_time"] or "off"}{" [dry run]" if a.dry_run else ""}')
    if a.once:
        m.tick()
        while m.child:
            m.child[0].wait()
            m.tick()
        return 0

    stopping = []
    for sig in (signal.SIGTERM, signal.SIGINT):        # launch stops us with SIGINT, then SIGTERM
        signal.signal(sig, lambda *_: stopping.append(1))
    t_start = time.monotonic()
    while not stopping and time.monotonic() - t_start < float(params['start_delay_s']):
        time.sleep(1)
    if not stopping:
        try:
            append_event(root, 'recorder', 'maintenance_start', keep_days=params['keep_days'],
                         nightly=params['nightly_time'])
        except OSError as e:                           # e.g. disk full: prune must still run
            log(f'cannot write event: {e}')
    while not stopping:
        m.tick()
        for _ in range(30):
            if stopping:
                break
            time.sleep(1)
    m.stop()
    return 0


if __name__ == '__main__':
    sys.exit(main())
