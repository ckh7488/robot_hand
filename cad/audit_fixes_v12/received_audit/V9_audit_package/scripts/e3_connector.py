"""E3: connector/lead exit keep-out per XL330, using the submitter's own index_joint_v1 connector registration
(joint_hardware_spec.json) shifted by the V9 vendor placement (-14.5 mm in x: build_candidate.py line 120/125 vs
index_joint_v1 build_joint.py (8,0,0)). Two ports: dorsal (-z, registered in spec) and its +z mirror (other port,
the one used by check_connector_direction.py). Keep-out = plug box + straight lead box to the spec's bend start.
Distances to every other part except the motor's own case/rotor, in the 6 exported poses."""
import sys, json, numpy as np, fcl
sys.path.insert(0, sys.argv[2]); from v9geo import Geo
g = Geo(sys.argv[1]); spec = json.load(open(sys.argv[3])); OUT = sys.argv[4]
dx = -14.5
c = spec['connector']; w = spec['wire']
po = np.array(c['registered_outer_origin_mm']) + [dx, 0, 0]; ps = np.array(c['registered_outer_size_mm'])
ys = w['start_y_mm']; rr = w['insulation_od_max_mm'] / 2 + w['extra_radial_allowance_mm']
x0 = w['start_x_mm'] + dx; x1 = w['bend_start_x_mm'] + dx
lead_lo = np.array([x1, min(ys) - rr, w['z_mm'] - rr]); lead_hi = np.array([x0, max(ys) + rr, w['z_mm'] + rr])
boxes = {'plug_dorsal': (po, po + ps), 'lead_dorsal': (lead_lo, lead_hi)}
mir = np.diag([1, 1, -1])
for k in list(boxes):
    lo, hi = boxes[k]; a, b = mir @ lo, mir @ hi; boxes[k.replace('dorsal', 'palmar')] = (np.minimum(a, b), np.maximum(a, b))
print({k: (np.round(v[0], 2).tolist(), np.round(v[1], 2).tolist()) for k, v in boxes.items()})
motors = [i for i in range(g.N) if g.meta[i]['role'] == 'vendor_motor']
rows = {}
for ps_name in g.poses:
    M = g.Tw[ps_name]; g.set(M)
    for mi in motors:
        own = g.names[mi].replace('_motor', '')
        for k, (lo, hi) in boxes.items():
            ctr = (lo + hi) / 2; L = hi - lo; T = M[mi] @ np.block([[np.eye(3), ctr[:, None]], [np.zeros((1, 3)), np.ones((1, 1))]])
            ob = fcl.CollisionObject(fcl.Box(*L), fcl.Transform(T[:3, :3], T[:3, 3]))
            wc = T[:3, 3]; rad = np.linalg.norm(L) / 2
            best = (99, None)
            for j in range(g.N):
                if j == mi or g.names[j] in (own + '_rotor',): continue
                # quick prune by AABB of part vs sphere
                gap = np.maximum(0, np.maximum(g.lo[j] - wc, wc - g.hi[j]))
                if np.linalg.norm(gap) > rad + 3: continue
                r = fcl.DistanceResult(); d = fcl.distance(ob, g.objs[j], fcl.DistanceRequest(), r)
                if d <= 1e-7: d = 0.0 if fcl.collide(ob, g.objs[j], fcl.CollisionRequest(), fcl.CollisionResult()) else d
                if d < best[0]: best = (round(float(d), 3), g.names[j])
            key = (g.names[mi], k)
            if key not in rows or best[0] < rows[key][0]: rows[key] = (best[0], best[1], ps_name)
out = [{'motor': k[0], 'keepout': k[1], 'min_mm': v[0], 'nearest_part': v[1], 'pose': v[2]} for k, v in rows.items()]
out.sort(key=lambda r: r['min_mm'])
json.dump({'boxes_motor_local_mm': {k: [v[0].tolist(), v[1].tolist()] for k, v in boxes.items()}, 'rows': out}, open(OUT, 'w'), indent=1)
blocked = [r for r in out if r['min_mm'] <= 0]
print('keep-outs', len(out), 'blocked(<=0)', len(blocked), 'below 1mm', sum(r['min_mm'] < 1 for r in out))
for r in out[:45]: print(r)
