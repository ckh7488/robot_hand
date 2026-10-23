"""E1: arm-vs-hand and arm-internal clearance over arm grid x 6 hand poses (their arm scan: Open hand only, roll +-60, elbow 0)."""
import sys, json, itertools, time, numpy as np
sys.path.insert(0, sys.argv[2]); from v9geo import Geo
g = Geo(sys.argv[1]); OUT = sys.argv[3]
armI = [i for i in range(g.N) if g.armlink[i] != 'hand']
pairs = g.moving_pairs(lambda i, j: (i in armI) or (j in armI))
print('arm-involving moving pairs', len(pairs))
best = {}; t0 = time.time(); n = 0
grid = list(itertools.product([0, 55, 110], [-80, -60, 0, 60, 80], range(-35, 56, 5), range(-20, 21, 5)))
for ps in g.poses:
    for (e, r, p, y) in grid:
        g.set(g.place(g.C['poses'][ps], (e, r, p, y))); n += 1
        for i, j in g.near(pairs, 5.0):
            d, _ = g.dist(i, j); k = (g.names[i], g.names[j])
            if k not in best or d < best[k][0]: best[k] = (d, ps, (e, r, p, y))
rows = sorted([{'a': k[0], 'b': k[1], 'min_mm': round(v[0], 3), 'hand_pose': v[1], 'arm_q': v[2]} for k, v in best.items()], key=lambda r: r['min_mm'])
json.dump({'evaluations': n, 'grid': 'elbow{0,55,110} x roll{-80,-60,0,60,80} x pitch -35..55/5 x yaw -20..20/5 x 6 hand poses', 'rows': rows}, open(OUT, 'w'), indent=1)
print('evals', n, 'time', round(time.time() - t0, 1))
for r in rows[:25]: print(r)
