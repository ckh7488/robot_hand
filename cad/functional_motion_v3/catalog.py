"""Whole-hand task probes, independent of any collision verdict or controller.

The optimizer finds finite kinematic candidates, never a proof of infeasibility.
Pad landmarks are actual cap-mesh surface points, not link-centre fingertips.
"""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import least_squares
import trimesh

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.dont_write_bytecode = True
spec = importlib.util.spec_from_file_location('_function_fk', ROOT/'cad/gesture_hand_v3/poses.py')
fk = importlib.util.module_from_spec(spec); spec.loader.exec_module(fk)
DIGITS = ['index', 'middle', 'ring', 'little', 'thumb']


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def design():
    d = read(ROOT/'cad/motion_clearance_v2/generated/design.json')
    d['joint_ranges_deg'] = dict(spread=[-27, 27], mcp=[-35, 100], pip=[-25, 135], dip=[-15, 95],
        thumb_base_sweep=[-70, 60], thumb_opposition=[-5, 90], thumb_mcp=[-15, 90], thumb_ip=[-25, 90])
    return d


def pad_anchors(d, rows):
    neutral = fk.forward_kinematics(d, np.zeros(20))['bodies']; result = {}
    for i, digit in enumerate(DIGITS):
        row = next(r for r in rows if r['name'] == digit+'_segment3_rounded_tip_cap')
        mesh = trimesh.load_mesh(ROOT/row['mesh'], process=False)
        inv = np.linalg.inv(neutral[row['body']])
        mesh.vertices = trimesh.transform_points(mesh.vertices, inv)
        length = (d['fingers']+[d['thumb']])[i]['links'][2]
        point, distance, face = trimesh.proximity.closest_point_naive(mesh, [[0., length-4., 11.]])
        result[digit] = dict(part=row['name'], body=row['body'], local_point_mm=point[0].tolist(),
            local_outward_normal=mesh.face_normals[face[0]].tolist(), mesh_sha256=row['sha256_stl'],
            target_to_actual_surface_mm=float(distance[0]), role='engineering palmar cap landmark')
    return result


def world_anchor(d, q, anchor):
    t = fk.forward_kinematics(d, q)['bodies'][anchor['body']]
    return t[:3, :3]@anchor['local_point_mm']+t[:3, 3], t[:3, :3]@anchor['local_outward_normal']


def contact_metrics(d, q, anchors, digit):
    p, n = world_anchor(d, q, anchors[digit]); t, m = world_anchor(d, q, anchors['thumb'])
    return dict(landmark_separation_mm=float(np.linalg.norm(p-t)),
        outward_normal_dot=float(n@m), digit_point_mm=p.tolist(), thumb_point_mm=t.tolist(),
        criterion='Candidate only: landmarks <=2mm and outward-normal dot <=-0.8; chosen engineering screen, not a biological standard',
        landmark_target_met=bool(np.linalg.norm(p-t) <= 2 and n@m <= -.8))


def fit_pinch(d, anchors, digit, initial):
    i = DIGITS.index(digit); active = list(range(i*4, i*4+4))+list(range(16, 20))
    limits = fk.joint_ranges(d)[active]
    # Active flexion stays nonnegative; reverse extension is not used to fake a pinch.
    limits[:4, 0] = [-27, 0, 0, 0]
    bounds = (limits[:, 0], limits[:, 1]); solutions = []
    seeds = [[0, 45, 65, 40, -15, 50, 40, 40], [15, 70, 70, 35, 10, 75, 30, 25],
             [-15, 80, 40, 60, -30, 50, 65, 55], [0, 20, 80, 65, 35, 70, 20, 70],
             [0, 80, 90, 70, -45, 30, 75, 75], [0, 60, 30, 20, 55, 85, 5, 5]]
    def residual(x):
        q = initial.copy(); q[active] = x
        p, n = world_anchor(d, q, anchors[digit]); t, m = world_anchor(d, q, anchors['thumb'])
        return np.r_[p-t, 15*(n+m)]
    for seed in seeds:
        fit = least_squares(residual, np.asarray(seed), bounds=bounds, max_nfev=350,
                            ftol=1e-9, xtol=1e-9, gtol=1e-9)
        q = initial.copy(); q[active] = fit.x
        metrics = contact_metrics(d, q, anchors, digit)
        solutions.append(dict(angles_deg=q.tolist(), metrics=metrics, residual_norm=float(np.linalg.norm(fit.fun)),
                              optimizer_success=bool(fit.success), evaluations=int(fit.nfev)))
    return min(solutions, key=lambda x:x['residual_norm']), solutions


def crossings(d, q):
    chains = fk.forward_kinematics(d, q)['chains']; out = []
    for i in range(3):
        for j in range(3):
            a, b = chains['index'][i:i+2]; c, e = chains['middle'][j:j+2]
            matrix = np.stack([(b-a)[:2], -(e-c)[:2]], axis=1)
            if abs(np.linalg.det(matrix)) < 1e-8:
                continue
            s, t = np.linalg.solve(matrix, (c-a)[:2])
            if 0 < s < 1 and 0 < t < 1:
                p, r = a+s*(b-a), c+t*(e-c)
                out.append(dict(index_segment=i, middle_segment=j, xy_mm=p[:2].tolist(),
                                signed_depth_mm=float(p[2]-r[2]), index_point_mm=p.tolist(), middle_point_mm=r.tolist()))
    return out


def make():
    d = design(); scene = read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')
    rows = scene['hand']; anchors = pad_anchors(d, rows)
    old = read(ROOT/'cad/gesture_hand_v3/poses.json'); cases = []; paths = []; fits = {}
    zero = np.zeros(20); fist = np.array(old['reference']['authored_fist_deg'])
    def add(ident, label, q, category, target, **extra):
        q = fk.validate_angles(q, d)
        cases.append(dict(id=ident, label_ko=label, category=category, target=target,
                          angles_deg=q.tolist(), **extra))
        return q.copy()
    def path(ident, points, target):
        for q in points: fk.validate_angles(q, d)
        paths.append(dict(id=ident, target_case=target, waypoints_deg=[np.array(q).tolist() for q in points],
            sample_step_deg=5., reverse='Exact geometric reverse uses identical configurations; separately indexed, no dynamic reversal claim'))
    for g in old['gestures']:
        q = np.array(g['angles_deg'])
        if g['id'] == 'thumb_up': q=fist.copy(); q[16:]=[0, 0, 0, -15]
        add('gesture_'+g['id'], g['label_ko'], q, 'existing_expression', 'Preserved named gesture, subjective anatomy not automatically accepted')
    for count in range(1, 6):
        q = fist.copy(); q[16:]=[0, 0, 0, 0] if count==5 else fist[16:]
        for i in range(min(count, 4)): q[i*4:i*4+4]=0
        add('count_'+str(count), str(count)+' 손가락 수 표현', q, 'count', 'Named extended/folded digits, no object contact claim')
    for ident, extended in [('L', [0, 4]), ('shaka', [3, 4])]:
        q=fist.copy()
        for i in extended: q[i*4:i*4+4]=0
        add(ident, 'L 손모양' if ident=='L' else '전화·샤카 손모양', q, 'expression', 'Index/thumb or little/thumb extended, remaining digits folded')
    for name, values in [('hook', [0, 100, 75]), ('tabletop', [85, 0, 0]),
                         ('flat_fist', [80, 100, 0]), ('half_curl', [40, 45, 30])]:
        q=zero.copy()
        for i in range(4): q[i*4+1:i*4+4]=values
        add('four_'+name, name, q, 'coordinated_flexion', 'Four coordinated digits with thumb extended')
    for i, digit in enumerate(DIGITS[:4]):
        for label, angles in [('curl', [85, 110, 80]), ('hook', [0, 100, 75]),
                              ('tabletop', [85, 0, 0]), ('tip_curl', [0, 0, 75])]:
            q=zero.copy(); q[i*4+1:i*4+4]=angles
            add(digit+'_'+label, digit+' '+label, q, 'independent_digit', 'Other digits remain extended')
        q=zero.copy(); q[i*4+1:i*4+4]=[45, 60, 40]
        add(digit+'_tap', digit+' 순차 굽힘', q, 'sequential_tap', 'Flex/release a free digit; contact with a table is not modeled')
        path(digit+'_flex_release', [zero, q, zero], digit+'_tap')
    for suffix, curl in [('open', [0, 0, 0]), ('half', [40, 45, 30])]:
        q=zero.copy(); q[[0, 4, 8, 12]]=[22, 6, -8, -22]
        for i in range(4):q[i*4+1:i*4+4]=curl
        add('fan_'+suffix, '벌림 '+suffix, q, 'coordinated_spread', 'Spread at two flexion levels; no isolated maximal-angle requirement')
    # Fit actual surface landmarks, retaining all seeds and residual failures.
    for digit in DIGITS[:4]:
        best, allfits = fit_pinch(d, anchors, digit, zero)
        fits[digit]=allfits
        add('pinch_'+digit, '엄지–'+digit+' 손끝 집기', best['angles_deg'], 'surface_contact',
            'Real cap landmarks coincide with opposing outward normals, then native whole-hand audit',
            contact_digit=digit, functional_metrics=best['metrics'])
        q=np.array(best['angles_deg']); pre=q*.8
        path('approach_'+digit, [zero, pre, q, pre, zero], 'pinch_'+digit)
    # OK and heart retain the fit as a contact probe, not a shape-success label.
    for ident, label, base in [('OK', 'OK 고리', zero), ('finger_heart', '손가락 하트', fist)]:
        best, allfits=fit_pinch(d, anchors, 'index', base.copy())
        fits[ident]=allfits
        add(ident, label, best['angles_deg'], 'expression_contact',
            'Index/thumb cap contact plus named other-digit posture; silhouette still requires inspection',
            contact_digit='index', functional_metrics=best['metrics'])
    crossbase = np.array(next(g for g in old['gestures'] if g['id']=='scissors')['angles_deg'])
    oldcross=read(ROOT/'cad/whole_hand_motion_v1/reviews/crossed_fingers.json')['cases'][0]['angles_deg']
    add('cross_old_reference', '이전 교차, 앞뒤가 크게 떨어짐', oldcross, 'cross_reference',
        'Reference only, projected X is insufficient', crossings=crossings(d, oldcross))
    for digit, direction in [('middle', 1), ('index', -1)]:
        for depth in [10, 14, 18, 22, 25]:
            q=crossbase.copy(); q[:8]=0
            i=DIGITS.index(digit);q[i*4:i*4+4]=[20*direction, depth, -min(depth, 10), -max(0, depth-10)]
            ident='cross_'+digit+'_'+str(depth)
            add(ident, digit+' 앞쪽 교차 '+str(depth), q, 'close_cross_candidate',
                'Projected index/middle crossing with close local surfaces; actual material intersection is failure',
                crossings=crossings(d, q))
            if depth in [14, 22]:
                pre=q.copy(); pre[0]=-1;pre[4]=1
                path(ident+'_approach_release', [crossbase, pre, q, pre, crossbase], ident)
    # These are explicit coverage obligations, not angle-only fabricated passes.
    backlog=[dict(grasp_taxonomy_id=i, status='not_yet_object_defined_or_tested', source='Feix2016_GRASP') for i in range(1, 34)]
    objects=[dict(id=i, object_geometry=g, target_contact=t, status='not_yet_solved_or_audited') for i,g,t in [
        ('cylinder_small','diameter40 length120 mm','thumb, palm and >=2 fingers surround cylinder'),
        ('cylinder_large','diameter80 length120 mm','power wrap without finger/object penetration'),
        ('sphere_small','diameter40 mm','precision sphere, opposed distributed contacts'),
        ('sphere_large','diameter90 mm','power sphere, palm and distributed digit contact'),
        ('plate','80x50x2 mm','thumb opposed to two or more finger pads'),
        ('key','25x20x2 mm head plus 35mm shaft','thumb pad against index lateral surface'),
        ('writing','diameter8 length140 mm','thumb/index/middle tripod plus supported shaft'),
        ('in_hand_roll','diameter40 mm sphere','maintained contact while rolling'),
        ('regrasp','diameter20 length100 mm cylinder','transfer contacts without dropping')]]
    inputs={str(Path(__file__).relative_to(ROOT)).replace('\\','/'), 'cad/functional_motion_v3/SOURCES.json',
        'cad/functional_motion_v3/REQUIREMENTS.md', 'cad/gesture_hand_v3/poses.py', 'cad/gesture_hand_v3/poses.json',
        'cad/motion_clearance_v2/generated/design.json','cad/motion_clearance_v2/generated/assembly_catalog.json',
        'cad/whole_hand_motion_v1/reviews/crossed_fingers.json'}
    for r in rows:
        for key,hkey in [('mesh','sha256_stl'),('step','sha256_step')]:
            if sha(ROOT/r[key]) != r[hkey]: raise ValueError('Changed CAD '+r[key])
            inputs.add(r[key])
    return dict(schema='functional_motion_v3.catalog.1', cad_parts=364, design=d,
        input_sha256={p:sha(ROOT/p) for p in sorted(inputs)}, pad_anchors=anchors,
        cases=cases, paths=paths, fit_attempts=fits, static_grasp33_backlog=backlog, object_tasks=objects,
        assumptions='Angles, 2mm/-0.8 contact screen and 5deg samples are engineering choices. No clinical/anatomical coverage or physical grasp approval.',
        hardware_validated=False, whole_hand_motion_approved=False)


if __name__ == '__main__':
    out=HERE/'generated/catalog.json'
    if out.exists(): raise FileExistsError('Preserve previous catalog')
    result=make();write(out,result)
    print(json.dumps(dict(cases=len(result['cases']), paths=len(result['paths']),
        pinches={k:v[0]['metrics'] for k,v in result['fit_attempts'].items()})), flush=True)
