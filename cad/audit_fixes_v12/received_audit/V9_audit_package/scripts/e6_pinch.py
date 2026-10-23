"""E6: can the V9 thumb tip reach each fingertip (pinch-type gesture) without other collisions?
Coarse search: thumb 4 joints x finger MCP/PIP/DIP grid -> tip centroid distance (KD-tree), then exact FCL check of the
best candidates (tip-tip distance + all other moving pairs >= 0). Arm fixed. Estimate (grid 5-15 deg)."""
import sys, json, itertools, copy, re, numpy as np
from scipy.spatial import cKDTree
sys.path.insert(0, sys.argv[2]); from v9geo import Geo
g = Geo(sys.argv[1]); OUT = sys.argv[3]
O = g.C['poses']['Open']
tipname = {f: f + '_dip_tip' for f in ['index', 'middle', 'ring', 'little', 'thumb']}
ti = {f: g.idx[n] for f, n in tipname.items()}
def tip_c(f, pose):
    fr, _ = g.fk(pose); M = fr[f + '_3'] @ g.local[ti[f]]
    v = g.verts[g.meta[ti[f]]['asset']]; c = v.mean(0); return M[:3, :3] @ c + M[:3, 3]
thumb_grid = list(itertools.product(range(-10, 15, 4), range(0, 115, 8), range(0, 61, 6), range(0, 91, 10)))
TP = np.array([tip_c('thumb', {**O, 'thumb': list(q)}) for q in thumb_grid])
tree = cKDTree(TP)
allp = g.moving_pairs()
def servo(n):
    m = re.match(r'^(index|middle|ring|little|thumb)_(root|mcp|pip|dip)_', n); return m.group(0) if m else None
g.set(g.Tw['Open']); internal = set()
for i, j in g.near(allp, 1.0):
    a, b = g.names[i], g.names[j]
    if servo(a) and servo(a) == servo(b): internal.add((i, j))
ext = [p for p in allp if tuple(p) not in internal]
res = {}
for f in ['index', 'middle', 'ring', 'little']:
    fg = list(itertools.product(range(0, 91, 10), range(0, 101, 10), range(0, 81, 20)))
    FP = np.array([tip_c(f, {**O, f: [0, *q]}) for q in fg])
    cands = []
    for a_, nb in enumerate(tree.query_ball_point(FP, 40.0)):
        for b_ in nb: cands.append((float(np.linalg.norm(FP[a_] - TP[b_])), a_, b_))
    cands.sort()
    pick = []
    for lo_, hi_ in [(0, 10), (10, 20), (20, 30), (30, 40)]:
        band = [c for c in cands if lo_ <= c[0] < hi_]
        if band: pick += [band[int(x)] for x in np.linspace(0, len(band) - 1, min(len(band), 120))]
    best = None; tested = 0; free = 0
    for cd, ai, bi in pick:
        tested += 1
        pose = copy.deepcopy(O); pose[f] = [0, *fg[ai]]; pose['thumb'] = list(thumb_grid[bi])
        g.set(g.place(pose))
        dt = g.dist(ti['thumb'], ti[f])[0]
        others = [(g.dist(i, j)[0], g.names[i], g.names[j]) for i, j in g.near(ext, 0.5) if {i, j} != {ti['thumb'], ti[f]}]
        coll = [o for o in others if o[0] <= 0]
        row = {'tip_tip_mm': round(dt, 2), 'centroid_mm': round(cd, 1), 'finger_q': fg[ai], 'thumb_q': thumb_grid[bi]}
        if not coll: free += 1
        if not coll and dt > 0 and (best is None or dt < best['tip_tip_mm']): best = row
    res[f] = {'best_collision_free': best, 'candidates_within_40mm': len(cands), 'tested': tested, 'collision_free': free}
    print(f, res[f], flush=True)
json.dump(res, open(OUT, 'w'), indent=1, default=str)
