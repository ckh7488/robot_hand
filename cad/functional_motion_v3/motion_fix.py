"""Repair one required gesture without hiding its original failing test.

Only thumb posture changes. Accepted dimensions, axes and all364solids remain.
The clear endpoint must survive its approach/release before selection.
"""
import argparse
import importlib.util
import json
from pathlib import Path

import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
spec=importlib.util.spec_from_file_location('_motion_fix_audit',HERE/'audit.py')
audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    if args.output.exists():raise FileExistsError('Keep earlier evidence')
    native,v,cat,scene,parts=audit.load();engine=native.PairScreen(v,parts,scene['hand'],cat['design'],0.,True)
    original=np.array(next(c for c in cat['cases'] if c['id']=='count_1')['angles_deg'])
    fixed=original.copy();fixed[16:]+=np.array([-6.,3.,-8.,0.])
    # Open thumb while closing middle/ring/little, then fold it across them.
    pre=fixed.copy();pre[16:]=0.;zero=np.zeros(20)
    report=dict(completed=False,selected=False,cases=[],paths=[],geometry_changed=False,
        changed_joints=['thumb_base_sweep','thumb_opposition','thumb_mcp'],
        target='Index extended; middle/ring/little retain original folded angles; thumb folded without material penetration',
        original_case='count_1',input_sha256=dict(cat['input_sha256']),whole_hand_motion_approved=False)
    for p in ['cad/functional_motion_v3/motion_fix.py','cad/functional_motion_v3/audit.py','cad/gesture_hand_v3/audit_gestures.py','cad/articulation_drive_v2/verify.py','cad/functional_motion_v3/generated/catalog.json']:
        report['input_sha256'][p]=audit.sha(ROOT/p)
    for ident,q in [('count_1_original',original),('count_1_thumb_corrected',fixed)]:
        row=audit.compact(engine,q);report['cases'].append(dict(id=ident,native=row));audit.write(args.output,report)
    observations=[];points=[zero,pre,fixed,pre,zero];maxstep=0.
    for si,(a,b) in enumerate(zip(points[:-1],points[1:])):
        steps=max(1,int(np.ceil(np.max(np.abs(b-a))/5.)))
        for k in range(steps+1):
            if si and not k:continue
            q=a.copy() if not k else b.copy() if k==steps else a+(b-a)*k/steps
            if observations:maxstep=max(maxstep,float(np.max(np.abs(q-np.array(observations[-1]['native']['angles_deg'])))))
            observations.append(dict(segment=si,fraction=k/steps,native=audit.compact(engine,q)))
        report['paths']=[dict(id='count_1_corrected_approach_release',waypoints_deg=[q.tolist() for q in points],observations=observations,maximum_actual_joint_step_deg=maxstep)]
        audit.write(args.output,report)
    failures=sum(len(r['native']['operation_failures']) for r in observations)
    collisions=sum(bool(r['native']['collision_count']) for r in observations)
    guard=all(audit.sha(ROOT/p)==h for p,h in report['input_sha256'].items())
    original_collisions=report['cases'][0]['native']['collision_count']
    target_clear=report['cases'][1]['native']['no_reportable_material_collision_and_no_operation_failure']
    same_goal=bool(np.array_equal(original[:16],fixed[:16]))
    report.update(completed=True,input_hash_guard=guard,selected=bool(original_collisions and target_clear and not collisions and not failures and guard and same_goal),
        same_four_finger_goal=same_goal,summary=dict(original_colliding_pairs=original_collisions,corrected_colliding_pairs=report['cases'][1]['native']['collision_count'],
        sampled_path_configurations=len(observations),colliding_path_configurations=collisions,calculation_failures=failures),
        limitation='Finite5deg configurations including exact geometric reverse. No continuous, dynamics, tactile-object or physical approval. Other required functions still fail.')
    audit.write(args.output,report)
    if not guard:raise ValueError('Inputs changed')
    print(json.dumps(report['summary']),flush=True)


if __name__=='__main__':main()
