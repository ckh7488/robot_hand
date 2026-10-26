"""Full-hand routing of refined tip candidates; same CAD, no joint clipping."""
import copy
import importlib.util
import json
from pathlib import Path
import sys
import time

import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_tip_posture_audit',HERE/'audit.py')
audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit);p=audit.prior


def main():
    output=HERE/'reviews/tip_refined.json'
    if output.exists():raise FileExistsError('Keep earlier refinement audit')
    start=time.monotonic();ref=p.read(HERE/'generated/tip_refined.json');source=dict(ref['source_sha256'])
    for rel in ['cad/posture_correction_v1/audit_tip_refined.py','cad/posture_correction_v1/generated/tip_refined.json',
        'cad/posture_correction_v1/audit.py','cad/functional_motion_v3/audit.py','cad/gesture_hand_v3/audit_gestures.py','cad/articulation_drive_v2/verify.py']:source[rel]=p.sha(ROOT/rel)
    if not ref['completed'] or not ref['input_hash_guard'] or not all(p.sha(ROOT/r)==h for r,h in source.items()):raise ValueError('Stale refinement')
    native,v,old,scene,parts=p.load();engine=native.PairScreen(v,parts,scene['hand'],old['design'],0.,True);cache={}
    def evaluate(q):
        key=tuple(float(x).hex() for x in q)
        if key in cache:
            got=copy.deepcopy(cache[key]);got['reused_identical_whole_pose']=True;return got
        got=p.compact(engine,q);cache[key]=copy.deepcopy(got);return got
    result=dict(completed=False,source_sha256=source,cases=[],paths=[],selected=[],geometry_changed=False,axes_changed=False,lengths_changed=False,
        scope='Full364parts, all66066pairs per unique posture. Staged<=5deg forward paths, exact saved reverse. Unloaded tip-near posture, no object hold or physical approval.')
    body={r['name']:r['body'] for r in scene['hand']}
    for i,c in enumerate(ref['selected']):
        got=evaluate(c['angles_deg']);frames=v.frames(old['design'],c['angles_deg']);names=['index_segment3_rounded_tip_cap','thumb_segment3_rounded_tip_cap']
        shapes=[v.relocate(parts[n],frames[body[n]],engine.neutral[body[n]]) for n in names];gap=float(shapes[0].distance(shapes[1]))
        status='native_clear_endpoint_candidate' if not got['collision_count'] and not got['operation_failures'] and c['metrics']['tip_target_met'] and gap<=1. else 'unresolved_refined_tip_candidate'
        row=dict(id='near_tip_refined_'+str(i),group='index_thumb_distal_contact',angles_deg=c['angles_deg'],metrics=c['metrics'],native_tip_surface_gap_mm=gap,native=got,status=status)
        result['cases'].append(row);p.write(output,result);print(json.dumps(dict(case=row['id'],collisions=got['collision_count'],gap=gap,status=status)),flush=True)
    chosen=False
    for target in result['cases']:
        if target['status']!='native_clear_endpoint_candidate':continue
        for routeid,waypoints in audit.make_routes(target['group'],target['angles_deg'],old):
            observations=[];failed=False
            for si,(qa,qb) in enumerate(zip(waypoints[:-1],waypoints[1:])):
                n=max(1,int(np.ceil(np.max(np.abs(qb-qa))/5)))
                for k in range(n+1):
                    if si and not k:continue
                    q=qa.copy() if not k else qb.copy() if k==n else qa+(qb-qa)*k/n
                    got=evaluate(q);observations.append(dict(segment=si,fraction=k/n,native=got))
                    if got['collision_count'] or got['operation_failures']:failed=True;break
                if failed:break
            path=dict(id=target['id']+'__'+routeid,target=target['id'],group=target['group'],waypoints_deg=[q.tolist() for q in waypoints],
                status='aborted_at_first_reported_failure' if failed else 'sampled_approach_release_clear',forward_observations=observations,
                forward_sample_count=len(observations),max_joint_step_deg=5.,reverse='Not run after a failed forward path' if failed else 'Exact saved forward sequence in reverse',
                total_approach_release_samples=None if failed else len(observations)*2-1)
            result['paths'].append(path);p.write(output,result);print(json.dumps(dict(path=path['id'],status=path['status'],forward_samples=len(observations))),flush=True)
            if not failed:
                result['selected'].append(dict(group=target['group'],case=target['id'],path=path['id'],angles_deg=target['angles_deg']));chosen=True;break
        if chosen:break
    result.update(completed=True,input_hash_guard=all(p.sha(ROOT/r)==h for r,h in source.items()),elapsed_s=time.monotonic()-start,unique_whole_pose_native_evaluations=len(cache))
    p.write(output,result)
    if not result['input_hash_guard']:raise ValueError('Inputs changed')


if __name__=='__main__':main()
