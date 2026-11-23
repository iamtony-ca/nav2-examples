"""nav_data_prune: retention for ~/nav_data (run every 15 min by nav_data_maintenance; also by hand).

rec/      delete oldest mcap first when: older than --days, total over --cap-gb, or disk free below
          --min-free (then until --target-free). The file being written is never deleted.
pinned/   older than --pinned-days or over --pinned-cap-gb (oldest first)
events/ index/ sessions/   older than --meta-days
Empty bag directories are removed; metadata.yaml is not rewritten (export reads mcap directly).
Every deletion is logged to events/ as a recorder event.
"""
import argparse
import os
import shutil
import sys
import time

from nav_data_tools.common import (DEFAULT_ROOT, active_file, append_event, expand, list_bag_dirs,
                                   list_mcaps, list_rec_files)

GB = 1024 ** 3


def _disk(path):
    """(free bytes for unprivileged users, total bytes)"""
    st = os.statvfs(path)
    return st.f_bavail * st.f_frsize, st.f_blocks * st.f_frsize


def _mtime(path):
    """mtime, or None if the path vanished meanwhile (e.g. removed by another log cleaner)."""
    try:
        return os.path.getmtime(path)
    except OSError:
        return None


def _file_size(path):
    try:
        return os.path.getsize(path)
    except OSError:
        return 0


def _rm(path):
    try:
        shutil.rmtree(path, ignore_errors=True) if os.path.isdir(path) else os.remove(path)
    except FileNotFoundError:
        pass


TOP_DIRS = ('rec', 'events', 'index', 'sessions', 'pinned')


def keep_fresh(root, now=None):
    """Set the mtime of root, its top-level folders and the rec/<day> folders to now.

    Their mtime otherwise changes only when an entry is added, so a generic age-based cleaner
    (e.g. 'find <logs> -mtime +60 -exec rm -rf') run on a parent folder would see them as old and
    delete them as a whole, data being recorded included. Files keep their own times."""
    now = time.time() if now is None else now
    dirs = [root] + [os.path.join(root, d) for d in TOP_DIRS]
    rec = os.path.join(root, 'rec')
    if os.path.isdir(rec):
        dirs += [os.path.join(rec, d) for d in os.listdir(rec)]
    for d in dirs:
        if os.path.isdir(d):
            try:
                os.utime(d, (now, now))
            except OSError:
                pass


def _dir_size(path):
    total = 0
    for dp, _, fs in os.walk(path):
        for f in fs:
            try:
                total += os.path.getsize(os.path.join(dp, f))
            except OSError:
                pass
    return total


def prune(root, days=7.0, cap_gb=60.0, min_free=0.15, target_free=0.20, pinned_days=90.0,
          pinned_cap_gb=10.0, meta_days=90.0, recent_s=120.0, dry_run=False, now=None,
          disk_fn=None, log=print):
    now = time.time() if now is None else now
    disk_fn = disk_fn or (lambda: _disk(root))
    deleted = []

    def rm_file(path, reason, size):
        log(f'{"[dry] " if dry_run else ""}delete {os.path.relpath(path, root)} ({reason})')
        if not dry_run:
            try:
                os.remove(path)
            except FileNotFoundError:          # already removed by someone else: fine
                return True
            except OSError as e:
                log(f'  ! {e}')
                return False
            append_event(root, 'recorder', 'pruned', file=os.path.relpath(path, root),
                         reason=reason, bytes=size)
        deleted.append((path, reason))
        return True

    # ---- rec/
    active = active_file(root)
    files = [f for f in list_rec_files(root, read_summary=False) for mt in [_mtime(f.path)]
             if f.path != active and mt is not None and not (0 <= now - mt <= recent_s)]
    # (a file whose mtime is in the future - the clock was stepped back - is not "recent")
    files.sort(key=lambda f: (f.end_ns, f.path))
    total = sum(f.size for f in list_rec_files(root, read_summary=False))
    keep = []
    for f in files:
        if (now - f.end_ns / 1e9) > days * 86400:
            if rm_file(f.path, f'older than {days:g} d', f.size):
                total -= f.size
        else:
            keep.append(f)
    while keep and total > cap_gb * GB:
        f = keep.pop(0)
        if rm_file(f.path, f'rec over {cap_gb:g} GB', f.size):
            total -= f.size
    free, disk_total = disk_fn()
    if keep and disk_total and free / disk_total < min_free:
        need = target_free * disk_total - free
        freed = 0
        while keep and freed < need:
            f = keep.pop(0)
            if not rm_file(f.path, f'disk free below {min_free:.0%}', f.size):
                break
            freed += f.size

    # empty bag dirs and day dirs
    for bd in list_bag_dirs(root):
        if active and os.path.dirname(active) == bd:
            continue
        if not list_mcaps(bd):
            log(f'{"[dry] " if dry_run else ""}remove empty bag {os.path.relpath(bd, root)}')
            if not dry_run:
                shutil.rmtree(bd, ignore_errors=True)
    rec = os.path.join(root, 'rec')
    if os.path.isdir(rec) and not dry_run:
        for day in os.listdir(rec):
            dd = os.path.join(rec, day)
            if os.path.isdir(dd) and not os.listdir(dd):
                os.rmdir(dd)

    # ---- pinned/
    pinned = os.path.join(root, 'pinned')
    if os.path.isdir(pinned):
        items = sorted((mt, os.path.join(pinned, d)) for d in os.listdir(pinned)
                       for mt in [_mtime(os.path.join(pinned, d))] if mt is not None)
        sizes = {p: _dir_size(p) if os.path.isdir(p) else _file_size(p) for _, p in items}
        ptotal = sum(sizes.values())
        for mt, pth in list(items):
            over_age = now - mt > pinned_days * 86400
            if over_age or ptotal > pinned_cap_gb * GB:
                reason = f'pinned older than {pinned_days:g} d' if over_age else \
                    f'pinned over {pinned_cap_gb:g} GB'
                log(f'{"[dry] " if dry_run else ""}delete {os.path.relpath(pth, root)} ({reason})')
                if not dry_run:
                    _rm(pth)
                    append_event(root, 'recorder', 'pruned', file=os.path.relpath(pth, root),
                                 reason=reason, bytes=sizes[pth])
                deleted.append((pth, reason))
                ptotal -= sizes[pth]

    # ---- events/ index/ sessions/
    for sub in ('events', 'index', 'sessions'):
        d = os.path.join(root, sub)
        if not os.path.isdir(d):
            continue
        for name in os.listdir(d):
            pth = os.path.join(d, name)
            mt = _mtime(pth)
            if mt is not None and now - mt > meta_days * 86400:
                log(f'{"[dry] " if dry_run else ""}delete {sub}/{name} (older than {meta_days:g} d)')
                if not dry_run:
                    _rm(pth)
                deleted.append((pth, 'meta'))
    return deleted


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('--root', default=DEFAULT_ROOT)
    p.add_argument('--days', type=float, default=7.0)
    p.add_argument('--cap-gb', type=float, default=60.0)
    p.add_argument('--min-free', type=float, default=0.15)
    p.add_argument('--target-free', type=float, default=0.20)
    p.add_argument('--pinned-days', type=float, default=90.0)
    p.add_argument('--pinned-cap-gb', type=float, default=10.0)
    p.add_argument('--meta-days', type=float, default=90.0)
    p.add_argument('--dry-run', action='store_true')
    a = p.parse_args(argv)
    root = expand(a.root)
    if not os.path.isdir(root):
        print(f'{root} does not exist; nothing to do')
        return 0
    d = prune(root, a.days, a.cap_gb, a.min_free, a.target_free, a.pinned_days, a.pinned_cap_gb,
              a.meta_days, dry_run=a.dry_run)
    print(f'{len(d)} item(s) {"would be " if a.dry_run else ""}deleted')
    return 0


if __name__ == '__main__':
    sys.exit(main())
