"""Reimported full sphere and all364 saved hand parts, with finite routes."""
import importlib.util
import json
import math
from pathlib import Path
import sys
import time

import cadquery as cq
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.dont_write_bytecode = True
spec = importlib.util.spec_from_file_location('_pinch_native', ROOT/'cad/object_contact_v1/audit.py')
previous = importlib.util.module_from_spec(spec)
spec.loader.exec_module(previous)
p = previous.prior


def main():
    output = HERE/'reviews/audit.json'
    if output.exists():
        raise FileExistsError('Preserve previous native results')
    start = time.monotonic()
    search = p.read(HERE/'generated/search.json')
    source = dict(search['source_sha256'])
    for rel in ['cad/object_pinch_v2/audit.py', 'cad/object_pinch_v2/generated/search.json',
                'cad/object_contact_v1/audit.py', 'cad/functional_motion_v3/audit.py',
                'cad/gesture_hand_v3/audit_gestures.py', 'cad/articulation_drive_v2/verify.py']:
        source[rel] = p.sha(ROOT/rel)
    if not search['completed'] or not search['input_hash_guard'] or not all(p.sha(ROOT/r) == h for r, h in source.items()):
        raise ValueError('Incomplete or changed search')
    audit = previous.ObjectAudit()
    report = dict(completed=False, source_sha256=source, cases=[], paths=[], selected=[],
        hand_parts=364, self_pairs_per_pose=66066, object_pairs_per_pose=364,
        reporting_floor_mm3=.02, boolean_tolerance_mm=1e-6,
        contact_tolerance_mm=.05, contact_tolerance_note='Geometric engineering screen, not measured contact force or a standard.',
        task=search['task'], geometry_changed=False, axes_changed=False, lengths_changed=False,
        physical_hold_validated=False, whole_hand_or_manufacturing_approved=False)
    objects = {}
    cache = {}
    def evaluate(q, candidate):
        key = (candidate['id'], tuple(float(x).hex() for x in q))
        if key not in cache:
            got, moved, _ = audit.evaluate(q, candidate['metrics']['centre_mm'], 12.5, objects[candidate['id']])
            got['contacts'] = [dict(part=name, gap_mm=float(moved[name].distance(objects[candidate['id']])))
                for name in ['index_segment3_rounded_tip_cap', 'thumb_segment3_rounded_tip_cap']]
            cache[key] = got
        return cache[key]
    def failed(got):
        return bool(got['hand']['collision_count'] or got['object_collisions'] or
            got['hand']['operation_failures'] or got['object_operation_failures'])
    for c in search['selected']:
        path = HERE/'generated/objects'/(c['id']+'.step')
        path.parent.mkdir(parents=True, exist_ok=True)
        obj = cq.Solid.makeSphere(12.5, cq.Vector(*c['metrics']['centre_mm']),
            angleDegrees1=-90, angleDegrees2=90, angleDegrees3=360)
        cq.exporters.export(obj, str(path))
        obj = cq.importers.importStep(str(path)).val()
        if not obj.isValid() or len(obj.Solids()) != 1 or abs(obj.Volume()-4/3*math.pi*12.5**3) > 1e-5:
            raise ValueError('Saved sphere is not the declared full25mm object')
        if np.linalg.norm(np.array(obj.Center().toTuple())-c['metrics']['centre_mm']) > 1e-6:
            raise ValueError('Saved sphere centre changed')
        objects[c['id']] = obj
        got = evaluate(c['angles_deg'], c)
        contact = max(r['gap_mm'] for r in got['contacts']) <= .05
        natural = not any(c['angles_deg'][i] < -1 for i in [1, 2, 3, 17, 18, 19])
        status = 'native_clear_opposed_surface_candidate'
        if not contact or c['metrics']['outward_normal_dot'] > -.99:
            status = 'contact_target_not_reached'
        if not natural:
            status = 'reverse_flexion_candidate_not_adopted'
        if failed(got):
            status = 'material_interference_or_uncertainty'
        report['cases'].append(dict(id=c['id'], angles_deg=c['angles_deg'], metrics=c['metrics'],
            native=got, status=status, object_step=path.relative_to(ROOT).as_posix(), object_sha256=p.sha(path)))
        p.write(output, report)
        print(json.dumps(dict(case=c['id'], status=status, hand=got['hand']['collision_count'],
            object=len(got['object_collisions']), contacts=got['contacts'])), flush=True)
    for c in search['selected']:
        endpoint = next(r for r in report['cases'] if r['id'] == c['id'])
        if endpoint['status'] != 'native_clear_opposed_surface_candidate':
            continue
        routes = sorted([r for r in search['routes'] if r['target'] == c['id']], key=lambda r:r['mesh_intersecting_samples'])
        for route in routes:
            observed = []
            for sample in route['mesh_observations']:
                got = evaluate(sample['angles_deg'], c)
                observed.append(dict(segment=sample['segment'], fraction=sample['fraction'], native=got))
                if failed(got):
                    break
            bad = any(failed(s['native']) for s in observed)
            row = dict(id=route['id'], target=c['id'], waypoints_deg=route['waypoints_deg'],
                status='aborted_at_first_failure' if bad else 'sampled_approach_release_clear',
                forward_observations=observed, max_joint_step_deg=5., forward_sample_count=len(observed),
                total_approach_release_samples=None if bad else len(observed)*2-1,
                release='Exact same saved forward configurations reversed; no dynamics or force model.')
            report['paths'].append(row)
            p.write(output, report)
            print(json.dumps(dict(path=row['id'], status=row['status'], samples=len(observed))), flush=True)
            if not bad:
                report['selected'].append(dict(case=c['id'], path=row['id']))
                break
        if report['selected']:
            break
    report.update(completed=True, input_hash_guard=all(p.sha(ROOT/r) == h for r,h in source.items()),
        elapsed_s=time.monotonic()-start, unique_whole_hand_object_evaluations=len(cache))
    p.write(output, report)
    if not report['input_hash_guard']:
        raise ValueError('Inputs changed')


if __name__ == '__main__':
    main()
