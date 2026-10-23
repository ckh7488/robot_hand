"""E4: V9 15 named-pose linear paths re-run directly (no V8 transfer), my code.
1) Samples with max joint step 0.5 deg (same as theirs) -> sampled minimum per path.
2) Continuous certification of the nominal path: for each interval and near pair, lower bound
   LB = min(d0,d1) - (dispA+dispB)/2, disp = sum_j |dq_j| * R_j(body) (R from part AABB corners to joint origin,
   max of both ends). Intervals with LB < target are bisected (depth<=8) and re-evaluated.
Servo-internal invariant pairs (same servo, vendor/TAP, <1 mm at Open) excluded and reported separately."""
import sys, json, itertools, math, re, time, numpy as np
sys.path.insert(0, sys.argv[2]); from v9geo import Geo
g = Geo(sys.argv[1]); OUT = sys.argv[3]; TARGET = 1.0; STEP = 0.5
def servo(n):
    m = re.match(r'^(index|middle|ring|little|thumb)_(root|mcp|pip|dip)_', n); return m.group(0) if m else None
allp = g.moving_pairs(); g.set(g.Tw['Open']); internal = set()
for i, j in g.near(allp, 1.0):
    a, b = g.names[i], g.names[j]
    if servo(a) and servo(a) == servo(b) and (g.meta[i]['role'].startswith('vendor') or '_tap' in a) and (g.meta[j]['role'].startswith('vendor') or '_tap' in b):
        internal.add((i, j))
pairs = [p for p in allp if tuple(p) not in internal]
fingers = ['index', 'middle', 'ring', 'little', 'thumb']
body_f = [None if b in ('base',) or b.startswith('arm') else (b.split('_')[0], int(b.split('_')[1])) for b in g.body]
def disp_bound(p0, p1):
    """per-part upper bound of point displacement between two configurations (linear joint path)."""
    out = np.zeros(g.N)
    for P in (p0, p1):
        fr, _ = g.fk(P); M = g.place(P)
        for i in range(g.N):
            if body_f[i] is None: continue
            f, k = body_f[i]
            c = g.corners[g.meta[i]['asset']] @ M[i, :3, :3].T + M[i, :3, 3]
            s = 0.
            for j in range(k + 1):  # joints 0..k of finger f move body k
                dq = math.radians(abs(p1[f][j] - p0[f][j])); org = fr[f + '_' + str(j)][:3, 3]
                s += dq * (np.linalg.norm(c - org, axis=1).max() + 1.0)
            out[i] = max(out[i], s)
    return out
def interp(pa, pb, t): return {n: [x + (y - x) * t for x, y in zip(pa[n], pb[n])] for n in pa}
def eval_pairs(P, cap, subset=None):
    g.set(g.place(P)); res = {}
    for i, j in g.near(subset if subset is not None else pairs, cap):
        res[(int(i), int(j))] = g.dist(int(i), int(j))[0]
    return res
rows = []; t0 = time.time()
for a, b in itertools.combinations(g.C['poses'], 2):
    pa, pb = g.C['poses'][a], g.C['poses'][b]
    delta = max(abs(x - y) for n in pa for x, y in zip(pa[n], pb[n])); n = max(2, math.ceil(delta / STEP))
    ts = [i / n for i in range(n + 1)]
    bnd = disp_bound(pa, interp(pa, pb, 1 / n))  # per-sample-step displacement bound (linear in t, same each step)
    cap = TARGET + bnd.max() + 0.5
    D = [eval_pairs(interp(pa, pb, t), cap) for t in ts]
    smin = min((min(d.values()) if d else cap, k) for k, d in enumerate(D))
    smin_pair = min(D[smin[1]], key=D[smin[1]].get) if D[smin[1]] else None
    # continuous certification
    worst_lb = (9e9, None); refined = 0; fail = []
    stack = []
    for k in range(n):
        for pr in set(D[k]) | set(D[k + 1]):
            d0 = D[k].get(pr, cap); d1 = D[k + 1].get(pr, cap)
            stack.append((ts[k], ts[k + 1], pr, d0, d1, 0))
    while stack:
        u0, u1, pr, d0, d1, depth = stack.pop()
        frac = (u1 - u0) * n; lb = min(d0, d1) - frac * (bnd[pr[0]] + bnd[pr[1]]) / 2
        if lb >= TARGET or depth >= 8:
            if lb < worst_lb[0]: worst_lb = (lb, (g.names[pr[0]], g.names[pr[1]]), u0, u1, depth)
            if lb < TARGET: fail.append((round(lb, 3), g.names[pr[0]], g.names[pr[1]], round(u0, 5)))
            continue
        um = (u0 + u1) / 2; dm = eval_pairs(interp(pa, pb, um), 99, [pr]).get(pr, 99); refined += 1
        stack.append((u0, um, pr, d0, dm, depth + 1)); stack.append((um, u1, pr, dm, d1, depth + 1))
    # dense true minimum near the sampled worst pair (0.02 deg-equivalent scan)
    row = {'from': a, 'to': b, 'max_joint_delta_deg': delta, 'samples': n + 1, 'step_deg': delta / n,
           'max_disp_bound_per_step_mm': round(float(bnd.max()), 3), 'sampled_min_mm': round(smin[0], 4), 'sampled_min_t': round(ts[smin[1]], 4),
           'sampled_min_pair': None if smin_pair is None else [g.names[smin_pair[0]], g.names[smin_pair[1]]],
           'continuous_lower_bound_mm': round(worst_lb[0], 4), 'lb_pair': worst_lb[1], 'refine_calls': refined, 'uncertified_intervals': len(fail), 'examples': fail[:5]}
    rows.append(row); print(row, round(time.time() - t0), flush=True)
    json.dump({'target_mm': TARGET, 'internal_pairs_excluded': len(internal), 'rows': rows}, open(OUT, 'w'), indent=1)
