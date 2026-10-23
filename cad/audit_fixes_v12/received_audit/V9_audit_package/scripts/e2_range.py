"""E2: mechanical range of each joint in the actual CAD (single-axis sweeps from Open, 1 deg), plus elbow sweep and
coupled fist sweep. Servo-internal pairs (same servo, both vendor/TAP, <1mm at Open) are tracked separately."""
import sys, json, re, copy, numpy as np
sys.path.insert(0, sys.argv[2]); from v9geo import Geo
g = Geo(sys.argv[1]); OUT = sys.argv[3]
def servo(n):
    m = re.match(r'^(index|middle|ring|little|thumb)_(root|mcp|pip|dip)_', n); return m.group(0) if m else None
allp = g.moving_pairs()
g.set(g.Tw['Open'])
internal = set()
for i, j in g.near(allp, 1.0):
    a, b = g.names[i], g.names[j]
    if servo(a) and servo(a) == servo(b) and (g.meta[i]['role'].startswith('vendor') or '_tap' in a) and (g.meta[j]['role'].startswith('vendor') or '_tap' in b):
        internal.add((i, j))
ext = [p for p in allp if tuple(p) not in internal]
print('moving pairs', len(allp), 'servo-internal near pairs', len(internal))
def scan(pose, arm=(0, 0, 0, 0), cap=1.0, pairs=ext):
    g.set(g.place(pose, arm)); best = (cap, None)
    for i, j in g.near(pairs, cap):
        d, _ = g.dist(i, j)
        if d < best[0]: best = (d, (g.names[i], g.names[j]))
    return best
O = g.C['poses']['Open']
R = {'index': [(-30, 30), (-45, 110), (-15, 120), (-15, 100)], 'thumb': [(-90, 90), (-45, 120), (-40, 110), (-40, 100)]}
for f in ['middle', 'ring', 'little']: R[f] = R['index']
res = {'single_axis': [], 'note': 'first angle from Open where an external (non servo-internal) moving pair is <1.0mm and <=0 (contact). 1 deg steps; contact angle +-1 deg.'}
for f, rr in R.items():
    for j, (lo, hi) in enumerate(rr):
        row = {'finger': f, 'joint': j}
        for sign, lim in [(+1, hi), (-1, lo)]:
            first1 = firstc = None; q0 = O[f][j]; q = q0
            while (q - lim) * sign < 0:
                q += sign; p = copy.deepcopy(O); p[f][j] = q; d, pr = scan(p)
                if first1 is None and d < 1.0: first1 = (q, round(d, 3), pr)
                if d <= 0: firstc = (q, pr); break
            row['+' if sign > 0 else '-'] = {'first_below_1mm': first1, 'first_contact': firstc, 'searched_to': lim}
        res['single_axis'].append(row); print(row, flush=True)
# elbow (arm internal only)
armI = {i for i in range(g.N) if g.armlink[i] != 'hand'}
arm_pairs = [p for p in allp if p[0] in armI or p[1] in armI]
el = []
for e in range(-20, 121, 1):
    d, pr = scan(O, (e, 0, 0, 0), cap=3.0, pairs=arm_pairs); el.append((e, round(d, 3), pr))
res['elbow_sweep'] = el
print('elbow', [x for x in el if x[1] < 1.0][:3], [x for x in el if x[1] < 1.0][-3:])
# coupled fist: fingers MCP/PIP/DIP = t*(90,100,80), thumb Close
fist = []
for t in np.linspace(0, 1, 101):
    p = copy.deepcopy(O)
    for f in ['index', 'middle', 'ring', 'little']: p[f] = [0, 90 * t, 100 * t, 80 * t]
    d, pr = scan(p); fist.append((round(float(t), 2), round(d, 3), pr))
res['fist_fingers_only'] = fist
print('fist first <1mm', next((x for x in fist if x[1] < 1.0), None), 'first contact', next((x for x in fist if x[1] <= 0), None))
json.dump(res, open(OUT, 'w'), indent=1, default=str)
