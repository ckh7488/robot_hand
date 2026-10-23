"""E10: simultaneous MCP + spread errors (not one-at-a-time) around Backbend/Splay/Close: all combos of {-3,0,+3} deg."""
import sys, json, itertools, copy, re, numpy as np
sys.path.insert(0, sys.argv[2]); from v9geo import Geo
g = Geo(sys.argv[1])
def servo(n):
    m = re.match(r'^(index|middle|ring|little|thumb)_(root|mcp|pip|dip)_', n); return m.group(0) if m else None
allp = g.moving_pairs(); g.set(g.Tw['Open']); internal = set()
for i, j in g.near(allp, 1.0):
    a, b = g.names[i], g.names[j]
    if servo(a) and servo(a) == servo(b) and (g.meta[i]['role'].startswith('vendor') or '_tap' in a) and (g.meta[j]['role'].startswith('vendor') or '_tap' in b): internal.add((i, j))
ext = [p for p in allp if tuple(p) not in internal]
F = ['index', 'middle', 'ring', 'little']; out = {}
for base in ['Backbend', 'Splay', 'Close']:
    for jidx, label in [(1, 'MCP'), (0, 'spread')]:
        worst = (99, None, None); n = 0; below = 0; contact = 0
        for e in itertools.product([-3, 0, 3], repeat=4):
            p = copy.deepcopy(g.C['poses'][base])
            for f, d in zip(F, e): p[f][jidx] += d
            g.set(g.place(p)); best = (99, None)
            for i, j in g.near(ext, 1.5):
                d, _ = g.dist(int(i), int(j))
                if d < best[0]: best = (d, (g.names[i], g.names[j]))
            n += 1; below += best[0] < 1.0; contact += best[0] <= 0
            if best[0] < worst[0]: worst = (round(best[0], 3), best[1], e)
        out[f'{base} {label} ±3'] = {'combos': n, 'below_1mm': int(below), 'contact': int(contact), 'worst': worst}
        print(base, label, out[f'{base} {label} ±3'], flush=True)
json.dump(out, open(sys.argv[3], 'w'), indent=1, default=str)
