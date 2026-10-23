"""E8: same-rigid-body (static) pairs, which the motion checks never test: near/contacting pairs in Open."""
import sys, json, collections, numpy as np, fcl
sys.path.insert(0, sys.argv[2]); from v9geo import Geo
g = Geo(sys.argv[1]); g.set(g.Tw['Open'])
pairs = [(i, j) for i in range(g.N) for j in range(i + 1, g.N) if g.body[i] == g.body[j] and not (g.armlink[i] != 'hand' or g.armlink[j] != 'hand')]
rows = []
for i, j in g.near(pairs, 0.5):
    d, npts = g.dist(int(i), int(j))
    if d < 0.5: rows.append((round(d, 3), g.names[i], g.names[j], g.meta[i]['asset'], g.meta[j]['asset']))
cat = collections.Counter((r[0] == 0, r[3], r[4]) for r in rows)
for k, v in sorted(cat.items(), key=lambda x: (not x[0][0], x[0][1])): print('touch/intersect' if k[0] else 'gap<0.5', k[1], '|', k[2], 'x', v)
json.dump(rows, open(sys.argv[3], 'w'), indent=0)
