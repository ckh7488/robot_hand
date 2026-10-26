"""One additional human-like combined pose on unchanged hand CAD."""
import importlib.util
import json
from pathlib import Path
import sys
import time

import cadquery as cq
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.dont_write_bytecode = True
spec = importlib.util.spec_from_file_location('_combined_native', ROOT/'cad/object_contact_v1/audit.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
p = m.prior


def main():
    out = HERE/'reviews/audit.json'
    if out.exists():
        raise FileExistsError('Preserve earlier example results')
    started = time.monotonic()
    pinch = p.read(ROOT/'cad/object_pinch_v2/generated/selected_posture.json')
    authored = p.read(ROOT/'cad/gesture_hand_v3/poses.json')['reference']['authored_fist_deg']
    old = p.read(ROOT/'cad/object_pinch_v2/reviews/audit.json')
    source = dict(old['source_sha256'])
    for rel in ['cad/combined_pose_v1/audit.py', 'cad/object_pinch_v2/generated/selected_posture.json',
                'cad/object_pinch_v2/reviews/audit.json', 'cad/gesture_hand_v3/poses.json', pinch['object_step']]:
        source[rel] = p.sha(ROOT/rel)
    if not old['completed'] or not old['input_hash_guard'] or not all(p.sha(ROOT/r) == h for r,h in source.items()):
        raise ValueError('Changed baseline or incomplete prior evidence')
    target = np.array(pinch['angles_deg'])
    target[4:8] = [-8., 0., 0., 0.]
    target[8:16] = authored[8:16]
    obj = cq.importers.importStep(str(ROOT/pinch['object_step'])).val()
    audit = m.ObjectAudit()
    limits = p.read(ROOT/'cad/functional_motion_v3/generated/catalog.json')['design']
    spec_fk = importlib.util.spec_from_file_location('_combined_fk', ROOT/'cad/gesture_hand_v3/poses.py')
    fk = importlib.util.module_from_spec(spec_fk)
    spec_fk.loader.exec_module(fk)
    ranges = fk.joint_ranges(limits)
    if np.any(target < ranges[:,0]) or np.any(target > ranges[:,1]):
        raise ValueError('Target outside existing engineering joint domain')
    cache = {}
    def evaluate(q):
        key = tuple(float(x).hex() for x in q)
        if key not in cache:
            got, moved, _ = audit.evaluate(q, pinch['object_centre_mm'], 12.5, obj)
            got['contacts'] = [dict(part=name, gap_mm=float(moved[name].distance(obj))) for name in
                ['index_segment3_rounded_tip_cap', 'thumb_segment3_rounded_tip_cap']]
            cache[key] = got
        return cache[key]
    def failed(got):
        return bool(got['hand']['collision_count'] or got['object_collisions'] or
            got['hand']['operation_failures'] or got['object_operation_failures'])
    report = dict(completed=False, source_sha256=source, geometry_changed=False, axes_changed=False,
        lengths_changed=False, hand_parts=364, self_pairs_per_pose=66066, object_pairs_per_pose=364,
        reporting_floor_mm3=.02, boolean_tolerance_mm=1e-6, physical_hold_validated=False,
        task='Thumb/index distal tips oppose a supported25mm sphere, middle extends and spreads outward8deg, ring/little fold. One combined example, not several new grasp successes.',
        object=dict(step=pinch['object_step'], diameter_mm=25, centre_mm=pinch['object_centre_mm'], externally_supported=True),
        cases=[], paths=[], selected=[])
    native = evaluate(target)
    clear = not failed(native) and max(c['gap_mm'] for c in native['contacts']) <= .05
    report['cases'].append(dict(id='pinch_middle_open_ring_little_fold', angles_deg=target.tolist(), native=native,
        object_step=pinch['object_step'], object_sha256=p.sha(ROOT/pinch['object_step']),
        status='clear_combined_pose_candidate' if clear else 'candidate_failed_or_uncertain'))
    p.write(out, report)
    print(json.dumps(dict(endpoint_clear=clear, hand=native['hand']['collision_count'], object=len(native['object_collisions']))), flush=True)
    if clear:
        zero = np.zeros(20)
        preshape = zero.copy()
        preshape[4:16] = target[4:16]
        routes = [('all_fingers_coordinated', [zero, target]),
            ('middle_ring_little_first_then_pinch', [zero, preshape, target])]
        for ident, waypoints in routes:
            samples = []
            for segment, (qa,qb) in enumerate(zip(waypoints[:-1],waypoints[1:])):
                count = max(1,int(np.ceil(np.max(np.abs(qb-qa))/5)))
                bad = False
                for i in range(count+1):
                    if segment and not i:
                        continue
                    q = qa.copy() if not i else qb.copy() if i == count else qa+(qb-qa)*i/count
                    got = evaluate(q)
                    samples.append(dict(segment=segment, fraction=i/count, native=got))
                    if failed(got):
                        bad = True
                        break
                if bad:
                    break
            row = dict(id=ident, target=report['cases'][0]['id'], waypoints_deg=[q.tolist() for q in waypoints],
                status='aborted_at_first_failure' if bad else 'sampled_approach_release_clear',
                forward_observations=samples, forward_sample_count=len(samples), max_joint_step_deg=5,
                total_approach_release_samples=None if bad else len(samples)*2-1,
                release='Exact reverse of saved forward configurations; no force/dynamics claim.')
            report['paths'].append(row)
            p.write(out,report)
            print(json.dumps(dict(path=ident, status=row['status'], forward_samples=len(samples))),flush=True)
            if not bad:
                report['selected'].append(dict(case=report['cases'][0]['id'],path=ident))
                break
    report.update(completed=True, input_hash_guard=all(p.sha(ROOT/r) == h for r,h in source.items()),
        unique_native_configurations=len(cache), elapsed_s=time.monotonic()-started)
    p.write(out,report)
    if not report['input_hash_guard']:
        raise ValueError('Inputs changed')


if __name__ == '__main__':
    main()
