"""Fit a supported 25mm sphere between actual distal cap surfaces.

Same CAD and joint domain. Capsules/root interpolation guide the fit only;
all materials remain in the subsequent native hand/object audit.
"""
import importlib.util
import json
from pathlib import Path
import sys
import time

import numpy as np
from scipy.interpolate import RegularGridInterpolator
from scipy.optimize import least_squares
import trimesh

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.dont_write_bytecode = True
spec = importlib.util.spec_from_file_location('_pinch_guide', ROOT/'cad/object_contact_v1/search.py')
g = importlib.util.module_from_spec(spec)
spec.loader.exec_module(g)


def sample_path(waypoints):
    samples = []
    for segment, (qa, qb) in enumerate(zip(waypoints[:-1], waypoints[1:])):
        steps = max(1, int(np.ceil(np.max(np.abs(qb-qa))/5.)))
        for k in range(steps+1):
            if segment and not k:
                continue
            q = qa.copy() if not k else qb.copy() if k == steps else qa+(qb-qa)*k/steps
            samples.append(dict(segment=segment, fraction=k/steps, angles_deg=q.tolist()))
    return samples


def main():
    output = HERE/'generated/search.json'
    if output.exists():
        raise FileExistsError('Preserve completed and failed candidates')
    start = time.monotonic()
    previous = g.read(ROOT/'cad/posture_correction_v1/generated/search.json')
    refinement = g.read(ROOT/'cad/posture_correction_v1/generated/tip_refined.json')
    root = g.read(ROOT/'cad/posture_correction_v1/generated/root_posture_map.json')
    scene = g.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')
    d = previous['design']
    anchors = previous['tip']['anchors']
    source = dict(previous['source_sha256'])
    for rel in ['cad/object_pinch_v2/search.py', 'cad/posture_correction_v1/generated/tip_refined.json',
                'cad/posture_correction_v1/generated/search.json',
                'cad/posture_correction_v1/generated/root_posture_map.json']:
        source[rel] = g.sha(ROOT/rel)
    if not all(g.sha(ROOT/p) == h for p, h in source.items()):
        raise ValueError('Changed inputs')
    active = [0, 1, 2, 3, 16, 17, 18, 19]
    limits = g.fk.joint_ranges(d)[active]
    interpolation = RegularGridInterpolator((root['spread_deg'], root['mcp_deg']),
        np.array(root['overlap_volume_mm3'])**(1/3), bounds_error=True)
    box = np.array(next(r for r in scene['hand'] if r['name'] == 'Palm_shaped_front_cover')['bbox_mm'])
    box[0, 2] = -35.
    pairs = np.array([[a, b] for a in range(15) for b in range(a+1, 15)
        if not (a//3 == b//3 and b-a <= 1) and (a//3 in [0, 4] or b//3 in [0, 4])
        and (a, b) != (2, 14)])
    seed = np.array(refinement['selected'][0]['angles_deg'])[active]
    seeds = [seed, seed + [0, -8, -8, 0, -5, 0, -5, 0],
        [0, 45, 60, 30, 45, 65, 45, 35], [5, 40, 65, 30, 25, 60, 40, 30],
        [-5, 55, 60, 35, 55, 70, 55, 35], [5, 55, 55, 30, 45, 65, 50, 40]]
    solutions = []
    for si, initial in enumerate(seeds):
        def residual(x):
            q = np.zeros(20)
            q[active] = x
            e = g.evaluate(d, q, anchors, 'index', 12.5, pairs, box)
            n = np.array(e['normal'])
            return np.r_[10*e['centre_residual'], 100*(n[0]+n[1]),
                4*e['capsule_residual'], 4*e['palm_residual'],
                10*float(interpolation([[x[0], x[1]]])[0]), .02*(x-seed),
                .6*np.maximum(-x[[1, 2, 3, 5, 6, 7]], 0),
                .2*max(sum(x[1:4])-210, 0)]
        fit = least_squares(residual, initial, bounds=(limits[:, 0], limits[:, 1]),
            max_nfev=260, ftol=1e-8, xtol=1e-8, gtol=1e-8)
        q = np.zeros(20)
        q[active] = fit.x
        metrics = g.evaluate(d, q, anchors, 'index', 12.5, pairs, box)
        for key in ['centre_residual', 'capsule_residual', 'palm_residual']:
            metrics.pop(key)
        solutions.append(dict(id='sphere25_seed'+str(si), angles_deg=q.tolist(),
            metrics=metrics, objective=float(np.linalg.norm(fit.fun)), nfev=fit.nfev))
    solutions.sort(key=lambda row: row['objective'])
    selected = []
    for row in solutions:
        if all(np.linalg.norm(np.array(row['angles_deg'])-r['angles_deg']) >= 4 for r in selected):
            selected.append(row)
        if len(selected) == 3:
            break

    # This mesh screen is an inexpensive route guide, not a contact certificate.
    neutral = g.fk.forward_kinematics(d, np.zeros(20))['bodies']
    hand = trimesh.collision.CollisionManager()
    for row in scene['hand']:
        hand.add_object(row['name'], trimesh.load_mesh(ROOT/row['mesh'], process=False))
    routes = []
    for candidate in selected:
        q = np.array(candidate['angles_deg'])
        centre = candidate['metrics']['centre_mm']
        sphere = trimesh.creation.icosphere(subdivisions=3, radius=12.5)
        sphere.apply_translation(centre)
        configurations = [('coordinated_all', [np.zeros(20), q])]
        for scale in [.55, .75]:
            pre = q*scale
            configurations.append(('coordinated_via_'+str(scale), [np.zeros(20), pre, q]))
        for ident, waypoints in configurations:
            samples = sample_path(waypoints)
            observed = []
            for sample in samples:
                frames = g.fk.forward_kinematics(d, sample['angles_deg'])['bodies']
                for row in scene['hand']:
                    hand.set_transform(row['name'], frames[row['body']] @ np.linalg.inv(neutral[row['body']]))
                hit, names = hand.in_collision_single(sphere, return_names=True)
                observed.append(dict(**sample, object_mesh_intersections=sorted(names), intersects=bool(hit)))
            routes.append(dict(id=candidate['id']+'__'+ident, target=candidate['id'],
                waypoints_deg=[q.tolist() for q in waypoints], mesh_observations=observed,
                mesh_intersecting_samples=sum(o['intersects'] for o in observed),
                interpretation='Faceted sphere/mesh guide may miss exact contact or shallow interference.'))
    report = dict(completed=True, source_sha256=source, design=d, anchors=anchors,
        object=dict(shape='sphere', diameter_mm=25., role='Externally supported fixed object, no support CAD or gravity/force model.'),
        solutions=solutions, selected=selected, routes=routes, geometry_changed=False,
        input_hash_guard=all(g.sha(ROOT/p) == h for p, h in source.items()), elapsed_s=time.monotonic()-start,
        task='Open hand approaches a stationary supported sphere with opposed distal tips, then opens along the exact reverse. No lifting or unsupported hold claim.',
        proxy_note='Terminal capsule pair omitted from optimizer only. Native audit includes all hand pairs and all364 hand/object pairs.')
    g.write(output, report)
    print(json.dumps(dict(selected=selected, routes=[dict(id=r['id'], mesh_hits=r['mesh_intersecting_samples']) for r in routes])), flush=True)
    if not report['input_hash_guard']:
        raise ValueError('Changed inputs')


if __name__ == '__main__':
    main()
