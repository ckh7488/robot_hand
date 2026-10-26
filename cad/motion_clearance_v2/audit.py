"""Read saved STEP, recheck changed-palm pairs, and retain every other finding.

Unchanged pair outcomes are reused only with the same STEP bytes, body and pose.
Additional extension endpoints are fully recomputed, including all 364 parts.
The local continuous bound uses a rigid-point speed bound, with an explicitly
declared native distance accuracy assumption. It never approves the whole hand.
"""
import argparse, copy, hashlib, importlib.util, json, math, platform, sys, time
from pathlib import Path
import cadquery as cq
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
PALM = 'Palm_tapered_chassis'


def read(p):
    return json.loads(Path(p).read_text(encoding='utf-8-sig'))


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def module(name, p):
    spec = importlib.util.spec_from_file_location(name, p)
    obj = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(obj)
    return obj


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--mode', choices=['full', 'core'], default='full')
    a = ap.parse_args()
    if a.output.exists():
        raise FileExistsError(a.output)
    start = time.monotonic()
    hashes = {}

    def bind(p, expected=None):
        p = Path(p).resolve()
        if not p.is_relative_to(ROOT):
            raise ValueError('Input escaped checkout')
        h = sha(p)
        if expected is not None and expected != h:
            raise ValueError('Changed input ' + str(p))
        hashes[p.relative_to(ROOT).as_posix()] = h
        return p

    bind(__file__)
    baseline = read(bind(ROOT/'cad/whole_hand_motion_v1/reviews/latest_hand.json'))
    if not baseline['completed'] or not baseline['input_hash_guard']['passed']:
        raise ValueError('Incomplete baseline')
    for k, h in baseline['source_sha256'].items():
        bind(ROOT/k, h)
    build = read(bind(HERE/'generated/build_report.json'))
    for k, h in build['source_sha256'].items():
        bind(ROOT/k, h)
    if not build['completed'] or not build['input_hash_guard']['passed']:
        raise ValueError('Incomplete build')
    scene = read(bind(HERE/'generated/scene.json'))
    design = read(bind(HERE/'generated/design.json'))
    if design != read(ROOT/'cad/manufacturing_v2/generated/design.json'):
        raise ValueError('Design axes/lengths changed')
    native = module('_motion002_native', bind(ROOT/'cad/gesture_hand_v3/audit_gestures.py'))
    v = native.load_v2(ROOT)
    old_scene = read(ROOT/'cad/thumb_ip_drive_v1/generated/scene.json')
    old_rows = {r['name']:r for r in old_scene['hand']}
    rows = {r['name']:r for r in scene['hand']}
    if len(rows) != 364 or rows.keys() != old_rows.keys():
        raise ValueError('Changed part identity/count')
    parts = {}
    for i, r in enumerate(scene['hand']):
        for key, hk in [('step', 'sha256_step'), ('mesh', 'sha256_stl')]:
            bind(ROOT/r[key], r[hk])
        if r['name'] != PALM:
            for key in ['sha256_step', 'sha256_stl', 'body', 'volume_mm3', 'bbox_mm']:
                if r[key] != old_rows[r['name']][key]:
                    raise ValueError('Changed supposedly invariant part '+r['name'])
        s = cq.importers.importStep(str(ROOT/r['step'])).val()
        if not s.isValid() or len(s.Solids()) != 1 or abs(s.Volume()-r['volume_mm3']) > .001:
            raise ValueError('Saved STEP mismatch '+r['name'])
        parts[r['name']] = s
        if i % 60 == 0:
            print(json.dumps(dict(loaded=i+1)), flush=True)
    source_palm = cq.importers.importStep(str(ROOT/old_rows[PALM]['step'])).val()
    palm = parts[PALM]
    added = palm.cut(source_palm).Volume()
    if added > 1e-6:
        raise ValueError('Palm is not a material-removal subset')
    # Read independent masks from the preserved design, not generator output.
    masks = {'floor':cq.Solid.makeBox(400, 500, 20, cq.Vector(-200, -100, -60.5))}
    for f in design['fingers']:
        x, y, z = f['root']
        masks[f['name']+'_rear_mount'] = cq.Solid.makeBox(32, 14, 40, cq.Vector(x-16, y-15, -50))
    protected = {k:source_palm.intersect(m).cut(palm).Volume() for k,m in masks.items()}
    if max(protected.values()) > 1e-6:
        raise ValueError('Protected support geometry removed')
    neutral = v.frames(design, np.zeros(20))
    chains = native.body_chains(v.joint_names())
    bodies = {n:r['body'] for n,r in rows.items()}
    others = [n for n in parts if n != PALM]
    selected = {n:native.relative_indices(bodies[PALM], bodies[n], chains) for n in others}
    cache = {}
    report = dict(completed=False, mode=a.mode, source_sha256=hashes, cases=[], extra_extension_endpoints=[],
        part_count=364, unchanged_parts=363, changed_part=PALM,
        changed_pairs_per_observation=363, full_hand_pairs=364*363//2,
        geometry=dict(valid_single_solid=True, added_material_mm3=added,
            removed_material_mm3=source_palm.cut(palm).Volume(), protected_removed_mm3=protected),
        scope='Finite nominal changed-palm pairs plus exactly invariant pair findings; local continuous bounds only for four first shells at spread=0. No complete motion, tolerance, assembly, strength or motor approval.',
        manufacturing_release=False, whole_hand_motion_approved=False,
        environment=dict(python=platform.python_version(), cadquery=cq.__version__, numpy=np.__version__))

    def save():
        report['elapsed_s'] = time.monotonic()-start
        a.output.parent.mkdir(parents=True, exist_ok=True)
        temp = a.output.with_suffix('.partial')
        temp.write_text(json.dumps(report, separators=(',',':'), allow_nan=False), encoding='utf-8')
        temp.replace(a.output)

    core_ids = {'gesture_open','gesture_fist','gesture_thumb_up','four_fingers_extension','four_fingers_curl','fan_1',
                *[f+'_ordinary_extension_mcp' for f in ['index','middle','ring','little']],
                'gesture_open__gesture_fist@0.75','gesture_open__gesture_middle_finger@0.75'}
    observations = [o for o in baseline['cases'] if a.mode == 'full' or o['id'] in core_ids]
    palm_box = v.bounds(palm)
    for number, obs in enumerate(observations):
        q = obs['angles_deg']
        frames = v.frames(design, q)
        hits, errors = [], []
        for n in others:
            s = v.relocate(parts[n], frames[bodies[n]], neutral[bodies[n]])
            box = v.bounds(s)
            if np.any(np.minimum(palm_box[1], box[1])-np.maximum(palm_box[0], box[0]) <= 0):
                continue
            key = (n, tuple(float(q[i]).hex() for i in selected[n]))
            if key not in cache:
                try:
                    common = palm.intersect(s, tol=1e-6)
                    volume = float(common.Volume())
                    if not math.isfinite(volume) or volume < -1e-9:
                        raise ValueError('Invalid common volume')
                    cache[key] = dict(a=PALM, b=n, material_intersection_mm3=volume)
                except Exception as e:
                    cache[key] = dict(a=PALM, b=n, error=repr(e))
            result = cache[key]
            if 'error' in result:
                errors.append(result)
            elif result['material_intersection_mm3'] > .02:
                hits.append(result)
        invariant = [r for r in obs['collision_details'] if PALM not in [r['a'],r['b']]]
        invariant_errors = [r for r in obs['operation_failures'] if PALM not in [r['a'],r['b']]]
        old_palm = [r for r in obs['collision_details'] if PALM in [r['a'],r['b']]]
        old_pairs = {frozenset([r['a'],r['b']]) for r in old_palm}
        new_pairs = [r for r in hits if frozenset([r['a'],r['b']]) not in old_pairs]
        report['cases'].append(dict(id=obs['id'],kind=obs['kind'],angles_deg=q,
            baseline_collision_count=obs['collision_count'],baseline_palm_collisions=old_palm,
            changed_palm_collisions=hits, new_reportable_pairs=new_pairs,
            inherited_invariant_collisions=invariant,
            collision_count=len(hits)+len(invariant), operation_failures=errors+invariant_errors))
        save()
        print(json.dumps(dict(case=number+1,total=len(observations),id=obs['id'],palm_hits=len(hits),remaining=len(hits)+len(invariant))),flush=True)
    # At newly requested endpoints no old whole-hand pose result is inherited.
    full_engine = native.PairScreen(v, parts, scene['hand'], design, 0., True)
    for fi,finger in enumerate(design['fingers']):
        q = np.zeros(20);q[fi*4+1]=-25
        result = full_engine.evaluate(q)
        result.pop('exact_candidate_results');result.pop('near_pairs')
        report['extra_extension_endpoints'].append(dict(finger=finger['name'],result=result))
        save()
    # Adaptive distance samples bound every point in these four one-axis paths.
    # A point's rotation speed is <= radius * pi/180 per degree. Fixed palm
    # distance is therefore Lipschitz with the same bound. Debit numerical error.
    path_bounds = []
    for fi,finger in enumerate(design['fingers']):
        name = finger['name']+'_segment1_segment_shell'
        box = v.bounds(parts[name]); origin = neutral[bodies[name]][0]
        radius = max(math.hypot(y-origin[1], z-origin[2]) for y in box[:,1] for z in box[:,2])
        speed = radius*math.pi/180
        samples = {}
        def distance(angle):
            if angle not in samples:
                q = np.zeros(20);q[fi*4+1]=angle
                frames = v.frames(design,q)
                moved = v.relocate(parts[name],frames[bodies[name]],neutral[bodies[name]])
                gap = float(palm.distance(moved))
                if not math.isfinite(gap) or gap < 0:
                    raise ValueError('Invalid exact STEP distance')
                samples[angle] = gap
            return samples[angle]
        pending = [(-25., 0.)]; intervals = []; failures = []
        while pending:
            lo,hi = pending.pop()
            lower = min(distance(lo),distance(hi))-speed*(hi-lo)/2-1e-5
            if lower >= .5:
                intervals.append(dict(angle_interval_deg=[lo,hi],gap_lower_bound_mm=lower))
            elif hi-lo <= .03125 or min(distance(lo),distance(hi)) < .5:
                failures.append(dict(angle_interval_deg=[lo,hi],gap_lower_bound_mm=lower))
            else:
                mid = (lo+hi)/2;pending.extend([(lo,mid),(mid,hi)])
        row = dict(finger=finger['name'],pair=[PALM,name],mcp_range_deg=[-25,0],spread_deg=0,
            goal_mm=.5,radius_bound_mm=radius,distance_speed_bound_mm_per_degree=speed,
            assumed_native_distance_absolute_error_mm=1e-5,samples=[dict(angle_deg=k,gap_mm=samples[k]) for k in sorted(samples)],
            bounded_intervals=intervals,failed_intervals=failures,
            minimum_sampled_gap_mm=min(samples.values()),
            continuous_nominal_gap_lower_bound_mm=min((r['gap_lower_bound_mm'] for r in intervals+failures)),
            local_goal_met=not failures,
            scope='This shell/palm pair, zero spread and other joints zero. Native STEP distances under stated numeric-error assumption; no manufacturing tolerance or other-pair approval.')
        path_bounds.append(row)
        report['local_path_bounds']=path_bounds;save()
        print(json.dumps(dict(path=finger['name'],samples=len(samples),bound=row['continuous_nominal_gap_lower_bound_mm'],passed=not failures)),flush=True)
    report['completed'] = True
    report['input_hash_guard'] = {'passed':all(sha(ROOT/k)==h for k,h in hashes.items())}
    report['summary'] = dict(observations=len(report['cases']),
        static_cases=sum(c['kind']=='static' for c in report['cases']),
        colliding_static_cases=sum(c['kind']=='static' and c['collision_count']>0 for c in report['cases']),
        transition_observations=sum(c['kind']=='transition' for c in report['cases']),
        colliding_transition_paths=len({c['id'].split('@')[0] for c in report['cases'] if c['kind']=='transition' and c['collision_count']}),
        new_palm_collision_observations=sum(len(c['new_reportable_pairs']) for c in report['cases']),
        operation_failure_observations=sum(len(c['operation_failures']) for c in report['cases']),
        local_extension_paths_meeting_goal=sum(r['local_goal_met'] for r in path_bounds),
        whole_hand_motion_approved=False)
    save()
    return 0 if report['input_hash_guard']['passed'] and not report['summary']['new_palm_collision_observations'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
