"""Nightly output order: pins are made before the index (the 'day done' marker) is written, and a
pin left unfinished by an interrupted run is redone."""
import json
import os

from nav_data_tools import export, nightly


class _Hit:
    rule, name, count, file = 'R10', 'mark', 1, 'f.mcap'

    def __init__(self, t):
        self.start = self.end = t
        self.detail = {'mark': 'here'}


def test_unfinished_pin_is_redone_and_failure_is_not_fatal(tmp_path, monkeypatch):
    root = str(tmp_path)
    calls = []

    def fake_export(root_, a, b, out, log=print):
        calls.append(out)
        if 'XBROKENX' in out:
            raise RuntimeError('broken file')
        open(out, 'w').close()

    monkeypatch.setattr(export, 'run_export', fake_export)
    cfg = {'rules': {'R10': {}}}
    h = _Hit(1794981819 * 10**9)
    made = nightly.pin_marks(root, [h], cfg, log=lambda *_: None)
    assert len(made) == 1 and os.path.exists(os.path.join(made[0], 'note.json'))
    os.remove(os.path.join(made[0], 'note.json'))           # as if interrupted before note.json
    assert nightly.pin_marks(root, [h], cfg, log=lambda *_: None) == made and len(calls) == 2
    assert nightly.pin_marks(root, [h], cfg, log=lambda *_: None) == [] and len(calls) == 2
    bad = _Hit(1794981900 * 10**9)
    bad.detail = {'mark': 'XBROKENX'}
    assert nightly.pin_marks(root, [bad], cfg, log=lambda *_: None) == []
    ev = [json.loads(l) for f in os.listdir(os.path.join(root, 'events'))
          for l in open(os.path.join(root, 'events', f))]
    assert any(e.get('value') == 'pin_failed' for e in ev)


def test_index_written_atomically_last(tmp_path):
    root = str(tmp_path)
    cfg = {'rules': {'R10': {'name': 'mark'}}, 'report_top': 5}
    nightly.write_outputs(root, '20261117', [], cfg, [])
    names = sorted(os.listdir(os.path.join(root, 'index')))
    assert names == ['20261117.jsonl', '20261117_report.md']
