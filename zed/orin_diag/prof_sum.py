#!/usr/bin/env python3
"""Summarize a trtexec --exportProfile JSON: time per block (backbone/encoder/decoder/post) + top layers.

  python3 prof_sum.py deim_prof.json [N_TOP]
"""
import json, sys

rows = [r for r in json.load(open(sys.argv[1])) if 'name' in r]
ntop = int(sys.argv[2]) if len(sys.argv) > 2 else 15
key = 'averageMs' if 'averageMs' in rows[0] else 'timeMs'
KW = [('backbone', '/backbone'), ('encoder', '/encoder'), ('decoder', '/decoder'), ('post', '/postprocessor')]


def group(name):
    n = name.lower()
    hits = {g: n.count(k) for g, k in KW}
    g, c = max(hits.items(), key=lambda x: x[1])
    return g if c else 'other'


tot = sum(r[key] for r in rows)
print('layers %d  total %.3f ms/iter (%s)' % (len(rows), tot, key))
agg = {}
for r in rows:
    g = group(r['name'])
    agg[g] = agg.get(g, 0) + r[key]
for g, t in sorted(agg.items(), key=lambda x: -x[1]):
    print('  %-9s %7.3f ms  %5.1f%%' % (g, t, 100 * t / tot))
print('top %d layers:' % ntop)
for r in sorted(rows, key=lambda r: -r[key])[:ntop]:
    nm = r['name'].replace('\n', ' ')
    nm = nm if len(nm) <= 90 else nm[:60] + ' ... ' + nm[-25:]
    print('  %6.3f ms %5.1f%% [%s] %s' % (r[key], 100 * r[key] / tot, group(r['name']), nm))
