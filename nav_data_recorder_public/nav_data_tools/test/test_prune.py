"""Retention rules on a temporary tree (no ROS needed)."""
import os
import time

from nav_data_tools.prune import prune

DAY = 86400.0


def _mk(path, size, mtime):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'wb') as f:
        f.write(b'\0' * size)
    os.utime(path, (mtime, mtime))


def _tree(tmp, now):
    root = str(tmp)
    files = []
    # 10 days, one bag per day, 3 files each, 1 MB each
    for d in range(10):
        day = time.strftime('%Y%m%d', time.localtime(now - (9 - d) * DAY))
        bag = os.path.join(root, 'rec', day, f'080000_ab{d:02d}')
        for i in range(3):
            p = os.path.join(bag, f'080000_ab{d:02d}_{i}.mcap')
            _mk(p, 1 << 20, now - (9 - d) * DAY - (2 - i) * 600 - 300)
            files.append(p)
    return root, files


def test_age_rule_keeps_active_and_recent(tmp_path):
    now = time.time()
    root, files = _tree(tmp_path, now)
    deleted = prune(root, days=7, cap_gb=100, now=now, disk_fn=lambda: (90, 100), log=lambda *_: None)
    paths = {p for p, _ in deleted}
    # files of days 0,1 (older than 7 d) deleted; day 2 is 7 days old minus a few minutes
    assert all(f in paths for f in files[:6])
    assert files[-1] not in paths                      # active file
    assert all(os.path.exists(f) for f in files[9:])
    # empty bag dirs removed
    assert not os.path.exists(os.path.dirname(files[0]))


def test_cap_rule_deletes_oldest_first(tmp_path):
    now = time.time()
    root, files = _tree(tmp_path, now)
    cap = 20 / 1024  # 20 MB in GB
    prune(root, days=30, cap_gb=cap, now=now, disk_fn=lambda: (90, 100), log=lambda *_: None)
    left = [f for f in files if os.path.exists(f)]
    assert len(left) == 20
    assert left == files[-20:]


def test_disk_free_rule(tmp_path):
    now = time.time()
    root, files = _tree(tmp_path, now)
    total = 1000 << 20
    # 10% free; target 20% -> need 100 MB, more than we have: everything but the active file goes
    prune(root, days=30, cap_gb=100, now=now, disk_fn=lambda: (100 << 20, total),
          log=lambda *_: None)
    left = [f for f in files if os.path.exists(f)]
    assert left == [files[-1]]


def test_disk_free_rule_partial(tmp_path):
    now = time.time()
    root, files = _tree(tmp_path, now)
    total = 100 << 20
    # 14 MB free of 100 MB: need 6 MB -> 6 oldest files
    prune(root, days=30, cap_gb=100, now=now, disk_fn=lambda: (14 << 20, total),
          log=lambda *_: None)
    left = [f for f in files if os.path.exists(f)]
    assert left == files[6:]


def test_pinned_and_meta(tmp_path):
    now = time.time()
    root, _ = _tree(tmp_path, now)
    old_pin = os.path.join(root, 'pinned', '20250101_000000_old')
    new_pin = os.path.join(root, 'pinned', '20261101_000000_new')
    _mk(os.path.join(old_pin, 'export.mcap'), 10, now - 100 * DAY)
    _mk(os.path.join(new_pin, 'export.mcap'), 10, now - 1 * DAY)
    os.utime(old_pin, (now - 100 * DAY, now - 100 * DAY))
    _mk(os.path.join(root, 'events', '20250101.jsonl'), 10, now - 100 * DAY)
    _mk(os.path.join(root, 'events', '20261101.jsonl'), 10, now - 1 * DAY)
    prune(root, days=30, cap_gb=100, now=now, disk_fn=lambda: (90, 100), log=lambda *_: None)
    assert not os.path.exists(old_pin) and os.path.exists(new_pin)
    assert not os.path.exists(os.path.join(root, 'events', '20250101.jsonl'))
    assert os.path.exists(os.path.join(root, 'events', '20261101.jsonl'))


def test_dry_run_changes_nothing(tmp_path):
    now = time.time()
    root, files = _tree(tmp_path, now)
    d = prune(root, days=7, cap_gb=0.001, now=now, disk_fn=lambda: (1, 100), dry_run=True,
              log=lambda *_: None)
    assert d and all(os.path.exists(f) for f in files)


def test_keep_fresh_touches_folders_not_files(tmp_path):
    from nav_data_tools.prune import keep_fresh
    now = time.time()
    root, files = _tree(tmp_path, now)
    old = now - 100 * DAY
    for d in [root, os.path.join(root, 'rec')] + [os.path.join(root, 'rec', x) for x in os.listdir(os.path.join(root, 'rec'))]:
        os.utime(d, (old, old))
    mt_file = os.path.getmtime(files[0])
    keep_fresh(root, now)
    assert abs(os.path.getmtime(root) - now) < 2
    assert all(abs(os.path.getmtime(os.path.join(root, 'rec', x)) - now) < 2 for x in os.listdir(os.path.join(root, 'rec')))
    assert os.path.getmtime(files[0]) == mt_file          # files keep their age (retention uses it)


def test_prune_tolerates_files_vanishing(tmp_path, monkeypatch):
    import nav_data_tools.prune as P
    now = time.time()
    root, files = _tree(tmp_path, now)
    gone = files[0]
    real = os.path.getmtime
    old = [f for f in files if f != gone and now - real(f) > 7 * DAY]

    def flaky(p):                     # another cleaner removes this file right after listing
        if p == gone:
            raise FileNotFoundError(p)
        return real(p)
    monkeypatch.setattr(P.os.path, 'getmtime', flaky)
    deleted = P.prune(root, days=7, now=now, disk_fn=lambda: (900 << 30, 1000 << 30), log=lambda *_: None)
    assert gone not in [p for p, _ in deleted] and old
    assert sorted(p for p, r in deleted if 'older' in r) == sorted(old)   # the rest still pruned
