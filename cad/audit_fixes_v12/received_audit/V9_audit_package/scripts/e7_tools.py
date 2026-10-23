"""E7: straight tool / screw-head access along each M3 screw axis, at the assembly stage when it is driven.
Tool model (estimate): head passage cylinder d=5.7 mm (M3 socket head 5.5 + 0.2) over 0..12 mm beyond the head top,
driver shaft d=3.0 mm over 12..90 mm, handle d=22 mm over 90..180 mm. Distances by FCL to every part present
at that stage (except the screw set itself). Open hand pose, arm at 0."""
import sys, json, numpy as np, fcl, trimesh
sys.path.insert(0, sys.argv[2]); from v9geo import Geo
g = Geo(sys.argv[1]); OUT = sys.argv[3]
z = np.load(sys.argv[1] + '/collision_meshes.npz')
M = g.Tw['Open']; g.set(M)
def screws(part):
    i = g.idx[part]; a = g.meta[i]['asset']
    comps = trimesh.Trimesh(z[a + '_vertices'], z[a + '_faces'], process=True).split(only_watertight=False)
    out = []
    for c in comps:
        v = c.vertices @ M[i, :3, :3].T + M[i, :3, 3]; ctr = v.mean(0)
        _, _, vt = np.linalg.svd(v - ctr, full_matrices=False); ax = vt[0]; t = (v - ctr) @ ax
        rad = np.linalg.norm((v - ctr) - np.outer(t, ax), axis=1)
        if rad[t > t.max() - 0.3].max() < rad[t < t.min() + 0.3].max(): ax = -ax; t = -t
        out.append((ctr + ax * t.max(), ax))  # head top point, outward axis
    return out
def cyl_obj(p, ax, r, s0, s1):
    L = s1 - s0; c = p + ax * (s0 + L / 2)
    zax = np.array([0, 0, 1.]); v = np.cross(zax, ax); s = np.linalg.norm(v); cth = zax @ ax
    if s < 1e-9: R = np.eye(3) if cth > 0 else np.diag([1, -1, -1])
    else:
        K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]]); R = np.eye(3) + K + K @ K * ((1 - cth) / s**2)
    return fcl.CollisionObject(fcl.Cylinder(r, L), fcl.Transform(R, c))
segs = [('head_pass', 2.85, 0.05, 12), ('shaft', 1.5, 12, 90), ('handle', 11, 90, 180)]
def check(part, stage):
    res = []
    for k, (p, ax) in enumerate(screws(part)):
        row = {'screw': part, 'k': k, 'head_top': np.round(p, 1).tolist(), 'axis': np.round(ax, 2).tolist()}
        for name, r, s0, s1 in segs:
            ob = cyl_obj(p, ax, r, s0, s1); best = (99., None)
            for j in stage:
                if g.names[j] == part: continue
                rr = fcl.DistanceResult(); d = fcl.distance(ob, g.objs[j], fcl.DistanceRequest(), rr)
                if d <= 1e-7: d = 0.0 if fcl.collide(ob, g.objs[j], fcl.CollisionRequest(), fcl.CollisionResult()) else d
                if d < best[0]: best = (round(float(d), 2), g.names[j])
            row[name] = best
        res.append(row)
    return res
names = g.names
arm = [i for i in range(g.N) if g.meta[i]['role'].startswith('arm_')]
palm = [g.idx['Palm_back_chassis']]
saddle = [g.idx['Palm_wrist_saddle'], g.idx['Palm_saddle_mount_screws'], g.idx['Palm_saddle_mount_nuts']]
def module(f):
    F = f.capitalize()
    return [i for i, n in enumerate(names) if n.startswith(f + '_') or n.startswith(F + '_root_')]
thumb_mod = [i for i, n in enumerate(names) if n.startswith('thumb_') or n in ('Thumb_internal_cradle', 'Palm_thumb_mount_screws', 'Palm_thumb_mount_nuts')]
out = {'tool_model': segs, 'rows': []}
# saddle screws: palm + saddle only, then with arm attached
out['rows'] += [dict(r, stage='palm+saddle') for r in check('Palm_saddle_mount_screws', palm + saddle)]
out['rows'] += [dict(r, stage='palm+saddle+arm') for r in check('Palm_saddle_mount_screws', palm + saddle + arm)]
order = ['little', 'ring', 'middle', 'index']; done = []
for f in order:
    done += module(f)
    for extra, tag in [([], 'no saddle/arm'), (saddle, '+saddle'), (saddle + arm, '+saddle+arm')]:
        out['rows'] += [dict(r, stage=f'modules {order[:order.index(f)+1]} {tag}') for r in check(f.capitalize() + '_root_mount_screws', palm + done + extra)]
done += thumb_mod
for extra, tag in [([], 'no saddle/arm'), (saddle, '+saddle'), (saddle + arm, '+saddle+arm')]:
    out['rows'] += [dict(r, stage='all modules ' + tag) for r in check('Palm_thumb_mount_screws', palm + done + extra)]
allp = list(range(g.N))
out['rows'] += [dict(r, stage='complete hand, Open pose') for r in check('Palm_cover_screws', allp)]
json.dump(out, open(OUT, 'w'), indent=1, default=str)
for r in out['rows']:
    flag = '' if min(r['head_pass'][0], r['shaft'][0]) > 0 else '  <-- BLOCKED'
    print(r['stage'][:40].ljust(40), r['screw'].ljust(26), r['k'], 'head', r['head_pass'], 'shaft', r['shaft'], 'handle', r['handle'], flag)
