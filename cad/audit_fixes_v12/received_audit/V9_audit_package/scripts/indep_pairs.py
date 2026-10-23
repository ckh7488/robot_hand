"""Independent V9 pair audit (Claude). Uses only data files: scene.json (6 poses x 251 placements)
and collision_meshes.npz (FreeCAD tessellation, mm). Their eligible()/rigid_body() are imported from a
COPY only to label which pairs THEY exclude. Usage: python3 -I indep_pairs.py <modular_roots_v9 dir> <copy dir> <out.json>"""
import sys, json, itertools, collections, numpy as np, fcl
D, COPY, OUT = sys.argv[1:4]
sys.path.insert(0, COPY)
import kinematics_numpy as K  # copy; reads copied design.json/scene.json
S = json.load(open(D + '/scene.json'))
poses = list(S)
meta = S['Open']; N = len(meta); names = [p['name'] for p in meta]
Tw = {}
for ps in poses:
    arr = np.zeros((N, 4, 4))
    for i, p in enumerate(S[ps]):
        assert p['name'] == names[i]
        arr[i, :3, :3] = p['R']; arr[i, :3, 3] = p['pos_mm']; arr[i, 3, 3] = 1
    Tw[ps] = arr
# ---- rigid clusters from motion: relative transform constant over all 6 poses
def rel(a, b, ps):
    return np.linalg.inv(Tw[ps][a]) @ Tw[ps][b]
parent = list(range(N))
def find(x):
    while parent[x] != x:
        parent[x] = parent[parent[x]]; x = parent[x]
    return x
for a in range(N):
    for b in range(a + 1, N):
        r0 = rel(a, b, 'Open')
        if all(np.abs(rel(a, b, ps) - r0).max() < 1e-6 for ps in poses[1:]):
            parent[find(a)] = find(b)
cl = [find(i) for i in range(N)]
groups = collections.defaultdict(list)
for i, c in enumerate(cl): groups[c].append(names[i])
# name-based grouping (theirs) vs motion clusters
rb = [K.rigid_body(n) for n in names]
mismatch = []
for i in range(N):
    for j in range(i + 1, N):
        if rb[i] == rb[j] and cl[i] != cl[j]: mismatch.append((names[i], names[j]))
# ---- meshes
z = np.load(D + '/collision_meshes.npz')
models, corners = {}, {}
for a in sorted({p['asset'] for p in meta}):
    v = z[a + '_vertices']; f = z[a + '_faces']
    b = fcl.BVHModel(); b.beginModel(len(v), len(f)); b.addSubModel(v, f); b.endModel(); models[a] = b
    lo, hi = v.min(0), v.max(0)
    corners[a] = np.array([[x, y, zz] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for zz in (lo[2], hi[2])])
objs = [fcl.CollisionObject(models[p['asset']]) for p in meta]
req = fcl.DistanceRequest(enable_nearest_points=True)
def place(T):
    lo = []; hi = []
    for i, p in enumerate(meta):
        objs[i].setTransform(fcl.Transform(T[i, :3, :3], T[i, :3, 3]))
        c = corners[p['asset']] @ T[i, :3, :3].T + T[i, :3, 3]; lo.append(c.min(0)); hi.append(c.max(0))
    return np.array(lo), np.array(hi)
def dist(i, j):
    r = fcl.DistanceResult(); d = float(fcl.distance(objs[i], objs[j], req, r))
    if d <= 1e-7:
        d = 0.0 if fcl.collide(objs[i], objs[j], fcl.CollisionRequest(), fcl.CollisionResult()) else d
    return d, [np.asarray(x).tolist() for x in r.nearest_points] if r.nearest_points is not None else None
moving = [(i, j) for i in range(N) for j in range(i + 1, N) if cl[i] != cl[j]]
cap = 3.0
rows = collections.defaultdict(dict)
for ps in poses:
    lo, hi = place(Tw[ps])
    for i, j in moving:
        g = np.maximum(0, np.maximum(lo[i] - hi[j], lo[j] - hi[i]))
        if g @ g >= cap * cap: continue
        d, npts = dist(i, j)
        rows[(i, j)][ps] = (round(d, 4), npts)
out = {'n_parts': N, 'n_motion_clusters': len(groups), 'n_moving_pairs': len(moving),
       'name_group_vs_motion_mismatch': mismatch[:50], 'n_mismatch': len(mismatch),
       'clusters': sorted([sorted(v) for v in groups.values()], key=len, reverse=True), 'near': []}
for (i, j), per in rows.items():
    dmin = min(v[0] for v in per.values()); ps = min(per, key=lambda k: per[k][0])
    out['near'].append({'a': names[i], 'b': names[j], 'role_a': meta[i]['role'], 'role_b': meta[j]['role'],
                        'their_eligible': bool(K.eligible(meta[i], meta[j])),
                        'arm_scope_excluded': meta[i]['role'].startswith('arm_') or meta[j]['role'].startswith('arm_'),
                        'min_mm': dmin, 'pose': ps, 'nearest': per[ps][1], 'per_pose': {k: v[0] for k, v in per.items()}})
out['near'].sort(key=lambda r: r['min_mm'])
json.dump(out, open(OUT, 'w'), indent=1)
print('clusters', len(groups), 'moving pairs', len(moving), 'name/motion mismatches', len(mismatch))
for r in out['near'][:60]:
    print(f"{r['min_mm']:8.4f} {r['pose']:10s} elig={int(r['their_eligible'])} arm={int(r['arm_scope_excluded'])} {r['a']} | {r['b']}")
