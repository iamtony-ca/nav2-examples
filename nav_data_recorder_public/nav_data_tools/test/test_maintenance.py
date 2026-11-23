"""nav_data_maintenance: which days the nightly report is due for, params, one real prune pass."""
import datetime as _dt
import os
import time

import pytest

from nav_data_tools import maintenance as M
from nav_data_tools.common import RecFile

DAY = 86400.0


def _t(y, mo, d, h, mi=0):
    return _dt.datetime(y, mo, d, h, mi).timestamp()


def _file(t0, t1):
    return RecFile('x', 'b', 0, int(t0 * 1e9), int(t1 * 1e9), True, 1)


def test_due_after_nightly_time_only(tmp_path):
    files = [_file(_t(2026, 11, 17, 9), _t(2026, 11, 17, 18))]
    idx = str(tmp_path)
    assert M.due_dates(files, idx, _t(2026, 11, 18, 2, 29), '02:30', 3) == []
    assert M.due_dates(files, idx, _t(2026, 11, 18, 2, 30), '02:30', 3) == ['20261117']
    (tmp_path / '20261117.jsonl').write_text('')
    assert M.due_dates(files, idx, _t(2026, 11, 18, 3), '02:30', 3) == []


def test_catch_up_oldest_first_and_limit(tmp_path):
    # recorded across several days (one long session), robot then off; started on the 20th at 09:00
    files = [_file(_t(2026, 11, 14, 8), _t(2026, 11, 18, 20))]
    due = M.due_dates(files, str(tmp_path), _t(2026, 11, 20, 9), '02:30', 3)
    assert due == ['20261117', '20261118']        # 19th has no data, 16th is beyond catch_up_days
    assert M.due_dates(files, str(tmp_path), _t(2026, 11, 20, 9), '', 3) == []


def test_load_params(tmp_path):
    p = tmp_path / 'params.yaml'
    p.write_text('nav_data_recorder:\n  ros__parameters: {root_dir: x}\n'
                 'nav_data_maintenance:\n  ros__parameters: {keep_days: 14.0, nightly_time: ""}\n')
    params = M.load_params(str(p))
    assert params['keep_days'] == 14.0 and params['nightly_time'] == '' and params['cap_gb'] == 60.0
    # a typo is reported and ignored: it must not stop retention
    p.write_text('nav_data_maintenance:\n  ros__parameters: {keep_dayz: 1}\n')
    warned = []
    assert M.load_params(str(p), warn=warned.append)['keep_days'] == 7.0 and 'keep_dayz' in warned[0]


def test_nightly_time_forms(tmp_path):
    assert M.norm_time('02:30') == '02:30' and M.norm_time('2:30') == '02:30'
    assert M.norm_time('02:30:00') == '02:30' and M.norm_time('') == '' and M.norm_time(None) == ''
    # unquoted in YAML 1.1 these become base-60 ints
    p = tmp_path / 'p.yaml'
    for text, want in (('2:30', '02:30'), ('22:00', '22:00'), ('02:30:00', '02:30')):
        p.write_text(f'nav_data_maintenance:\n  ros__parameters: {{nightly_time: {text}}}\n')
        assert M.norm_time(M.load_params(str(p))['nightly_time']) == want, text
    for bad in ('25:00', 'soon', '2', True):
        with pytest.raises(ValueError):
            M.norm_time(bad)


def test_bad_nightly_time_keeps_pruning(tmp_path):
    params = tmp_path / 'p.yaml'
    params.write_text('nav_data_maintenance:\n  ros__parameters: {nightly_time: "soon"}\n')
    assert M.main(['--root', str(tmp_path / 'nd'), '--params-file', str(params), '--once']) == 0


class _Proc:
    def __init__(self, rc):
        self.returncode = rc

    def poll(self):
        return self.returncode


def test_gave_up_waits_for_next_window_failure_retries(tmp_path):
    root = str(tmp_path)
    os.makedirs(os.path.join(root, 'index'))
    params = dict(M.DEFAULTS)
    m = M.Maintenance(root, params)
    files = [_file(_t(2026, 11, 17, 9), _t(2026, 11, 17, 18))]
    m.due = lambda now, mono: [d for d in M.due_dates(files, root + '/index', now, '02:30', 3)
                               if mono >= m.retry_mono.get(d, 0) and now >= m.skip_until.get(d, 0)]
    now = _t(2026, 11, 18, 9)
    m.child = (_Proc(M.EXIT_GAVE_UP), '20261117')
    m.check_child(now, 1000.0)
    assert m.due(now, 1000.0) == [] and m.due(_t(2026, 11, 18, 23), 99999.0) == []
    assert m.due(_t(2026, 11, 19, 2, 31), 99999.0) == ['20261117']      # next night
    m.skip_until.clear()
    m.child = (_Proc(1), '20261117')
    m.check_child(now, 1000.0)
    assert m.due(now, 1000.0 + 59 * 60) == [] and m.due(now, 1000.0 + 61 * 60) == ['20261117']


def test_prune_interval_is_monotonic(tmp_path):
    m = M.Maintenance(str(tmp_path), dict(M.DEFAULTS, nightly_time=''))
    runs = []
    m.run_prune = lambda: runs.append(1)
    m.tick(now=_t(2026, 11, 20, 9), mono=100.0)
    m.tick(now=_t(2026, 11, 18, 9), mono=100.0 + 16 * 60)    # wall clock stepped back 2 days
    assert len(runs) == 2


def test_dry_run_starts_no_nightly(tmp_path, capsys):
    m = M.Maintenance(str(tmp_path), dict(M.DEFAULTS), dry_run=True)
    m.due = lambda now, mono: ['20261117']
    m.start_nightly(_t(2026, 11, 18, 9), 0.0)
    assert m.child is None and '[dry] nightly would run for 20261117' in capsys.readouterr().out


def test_once_fails_fast_when_running(tmp_path):
    import fcntl
    root = str(tmp_path)
    held = open(os.path.join(root, '.maintenance.lock'), 'w')
    fcntl.flock(held, fcntl.LOCK_EX)
    params = tmp_path / 'p.yaml'
    params.write_text('nav_data_maintenance:\n  ros__parameters: {nightly_time: ""}\n')
    assert M.main(['--root', root, '--params-file', str(params), '--once']) == 1
    assert M.main(['--root', root, '--params-file', str(params), '--once', '--dry-run']) == 0


def test_stop_kills_child_group(tmp_path):
    import subprocess
    import sys
    proc = subprocess.Popen([sys.executable, '-c', 'import signal, time; '
                             'signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(60)'],
                            start_new_session=True)
    m = M.Maintenance(str(tmp_path), dict(M.DEFAULTS))
    m.child = (proc, '20261117')
    t0 = time.monotonic()
    m.stop(grace_s=0.5)
    assert proc.poll() is not None and time.monotonic() - t0 < 4


def test_shipped_params_load():
    here = os.path.join(os.path.dirname(__file__), '..', '..', 'profiles')
    for name in sorted(os.listdir(here)):
        f = os.path.join(here, name, 'params.yaml')
        if os.path.isfile(f):
            params = M.load_params(f)
            if params['nightly_time']:
                M.parse_hhmm(params['nightly_time'])


def test_once_deletes_old_recordings(tmp_path):
    root = str(tmp_path)
    now = time.time()
    paths = []
    for age_d in (9, 8, 1, 0):
        day = time.strftime('%Y%m%d', time.localtime(now - age_d * DAY))
        bag = os.path.join(root, 'rec', day, '080000_ab01')
        os.makedirs(bag, exist_ok=True)
        p = os.path.join(bag, f'080000_ab01_{len(paths)}.mcap')
        with open(p, 'wb') as f:
            f.write(b'\0' * 1000)
        os.utime(p, (now - age_d * DAY - 600, now - age_d * DAY - 600))
        paths.append(p)
    params = tmp_path / 'p.yaml'
    params.write_text('nav_data_maintenance:\n  ros__parameters: {keep_days: 7.0, nightly_time: ""}\n')
    assert M.main(['--root', root, '--params-file', str(params), '--once']) == 0
    assert [os.path.exists(p) for p in paths] == [False, False, True, True]
    ev = open(os.path.join(root, 'events', os.listdir(os.path.join(root, 'events'))[0])).read()
    assert ev.count('"pruned"') == 2


def test_nightly_cmd():
    cmd = M.nightly_cmd('/r', '20261117', '/p/amr', 6.0)
    assert cmd[1:4] == ['-u', '-m', 'nav_data_tools.nightly'] and '--profile' in cmd and '20261117' in cmd
