"""Shared helpers: paths, time parsing, recording file discovery, event log append."""
import datetime as _dt
import json
import os
import re
import time
from dataclasses import dataclass
from typing import List, Optional

DEFAULT_ROOT = '~/nav_data'
_FILE_RE = re.compile(r'^(?P<base>.+)_(?P<idx>\d+)\.mcap$')
_BAG_RE = re.compile(r'^(?P<hms>\d{6})_')


def expand(path: str) -> str:
    return os.path.abspath(os.path.expanduser(path))


def now_ns() -> int:
    return time.time_ns()


def parse_time(text: str) -> int:
    """Local time string, ISO 8601 with offset, or epoch seconds -> ns since epoch."""
    text = text.strip()
    if re.fullmatch(r'\d+(\.\d+)?', text):
        return int(float(text) * 1e9)
    t = text.replace('T', ' ')
    for fmt in ('%Y-%m-%d %H:%M:%S.%f', '%Y-%m-%d %H:%M:%S', '%Y-%m-%d %H:%M',
                '%Y%m%d %H%M%S', '%Y%m%d_%H%M%S'):
        try:
            d = _dt.datetime.strptime(t, fmt)
            return int(d.astimezone().timestamp() * 1e9)  # naive -> local time
        except ValueError:
            pass
    try:
        d = _dt.datetime.fromisoformat(text)
        if d.tzinfo is None:
            d = d.astimezone()
        return int(d.timestamp() * 1e9)
    except ValueError:
        raise ValueError(f'cannot parse time: {text!r}') from None


def iso_local(ns: int) -> str:
    d = _dt.datetime.fromtimestamp(ns / 1e9).astimezone()
    return d.isoformat(timespec='milliseconds')


def local_date(ns: int) -> str:
    return _dt.datetime.fromtimestamp(ns / 1e9).strftime('%Y%m%d')


def day_range_ns(date: str):
    d = _dt.datetime.strptime(date, '%Y%m%d').astimezone()
    start = int(d.timestamp() * 1e9)
    end = int((d + _dt.timedelta(days=1)).timestamp() * 1e9)
    return start, end


@dataclass
class RecFile:
    path: str
    bag_dir: str
    index: int
    start_ns: int
    end_ns: int
    indexed: bool       # False -> file was not closed (power loss / kill -9)
    size: int

    def rel(self, root: str) -> str:
        return os.path.relpath(self.path, root)


def _bag_start_ns(bag_dir: str) -> Optional[int]:
    day = os.path.basename(os.path.dirname(bag_dir))
    m = _BAG_RE.match(os.path.basename(bag_dir))
    if not m or not re.fullmatch(r'\d{8}', day):
        return None
    try:
        return parse_time(f'{day}_{m.group("hms")}')
    except ValueError:
        return None


def _read_range(path: str):
    """(start_ns, end_ns, indexed) from the mcap summary; (None, None, False) if not readable."""
    try:
        import rosbag2_py  # noqa: deferred so prune works without ROS sourced
        md = rosbag2_py.Info().read_metadata(path, 'mcap')
        if md.message_count == 0:
            return None, None, False
        s = md.starting_time.nanoseconds
        return s, s + md.duration.nanoseconds, True
    except Exception:  # noqa: BLE001
        return None, None, False


def list_bag_dirs(root: str) -> List[str]:
    rec = os.path.join(root, 'rec')
    out = []
    if not os.path.isdir(rec):
        return out
    for day in sorted(os.listdir(rec)):
        dd = os.path.join(rec, day)
        if not os.path.isdir(dd):
            continue
        for b in sorted(os.listdir(dd)):
            bd = os.path.join(dd, b)
            if os.path.isdir(bd):
                out.append(bd)
    return out


def list_mcaps(bag_dir: str) -> List[str]:
    files = []
    for f in os.listdir(bag_dir):
        m = _FILE_RE.match(f)
        if m:
            files.append((int(m.group('idx')), os.path.join(bag_dir, f)))
    return [p for _, p in sorted(files)]


def list_rec_files(root: str, read_summary: bool = True) -> List[RecFile]:
    """All recorded mcap files in time order. Unclosed files get start = previous end."""
    out: List[RecFile] = []
    for bd in list_bag_dirs(root):
        prev_end = _bag_start_ns(bd)
        for p in list_mcaps(bd):
            try:
                st = os.stat(p)
            except FileNotFoundError:          # removed meanwhile (another cleaner): skip it
                continue
            idx = int(_FILE_RE.match(os.path.basename(p)).group('idx'))
            s = e = None
            indexed = False
            if read_summary:
                s, e, indexed = _read_range(p)
            if s is None:
                s = prev_end if prev_end is not None else int(st.st_mtime * 1e9)
                e = int(st.st_mtime * 1e9)
            out.append(RecFile(p, bd, idx, s, e, indexed, st.st_size))
            prev_end = e
    return out


def active_file(root: str) -> Optional[str]:
    """The file the recorder is most likely writing: the last file of the bag written most recently
    (by mtime, so a bag under an older date directory after a clock step is still found)."""
    best, best_mt = None, None
    for bd in list_bag_dirs(root):
        files = list_mcaps(bd)
        if not files:
            continue
        try:
            mt = os.path.getmtime(files[-1])
        except OSError:
            continue
        if best_mt is None or mt >= best_mt:
            best, best_mt = files[-1], mt
    return best


def append_event(root: str, kind: str, value, **extra):
    """Same line format as the recorder's EventLog."""
    t = now_ns()
    d = os.path.join(root, 'events')
    os.makedirs(d, exist_ok=True)
    rec = {'t': iso_local(t), 'kind': kind}
    if value is not None:
        rec['value'] = value
    rec.update(extra)
    with open(os.path.join(d, local_date(t) + '.jsonl'), 'a', encoding='utf-8') as f:
        f.write(json.dumps(rec, ensure_ascii=False) + '\n')
