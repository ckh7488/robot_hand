"""Saved364part audit of posture-only corrections and staged exact reverses."""
import argparse
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
spec=importlib.util.spec_from_file_location('_posture_native',ROOT/'cad/functional_motion_v3/audit.py')
prior=importlib.util.module_from_spec(spec);spec.loader.exec_module(prior)


def make_routes(group,target,old):
    q=np.array(target);zero=np.zeros(20)
    if group.startswith('cross_front_'):
        front=1 if group.endswith('middle') else 0;back=1-front
        start=np.array(next(c for c in old['cases'] if c['id']=='gesture_scissors')['angles_deg'])
        frontfirst=start.copy();frontfirst[front*4+1:front*4+4]=q[front*4+1:front*4+4]
        both=frontfirst.copy();both[back*4+1:back*4+4]=q[back*4+1:back*4+4]
        backfirst=start.copy();backfirst[back*4+1:back*4+4]=q[back*4+1:back*4+4]
        return [('lift_then_curl_then_cross',[start,frontfirst,both,q]),
            ('curl_then_lift_then_cross',[start,backfirst,both,q]),('coordinated_two_finger',[start,q])]
    lift=zero.copy();lift[17]=q[17]
    curled=lift.copy();curled[1:4]=q[1:4]
    flexed=curled.copy();flexed[18:20]=q[18:20]
    return [('thumb_lift_finger_curl_thumb_fold_lateral',[zero,lift,curled,flexed,q]),
            ('coordinated_flex_then_lateral',[zero,flexed,q]),('coordinated_all',[zero,q])]


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    if a.output.exists():raise FileExistsError('Keep previous audit')
    start=time.monotonic();search=prior.read(HERE/'generated/search.json')
    if not search['completed'] or not search['input_hash_guard']:raise ValueError('Unfinished search')
    sources=dict(search['source_sha256'])
    for p in ['cad/posture_correction_v1/audit.py','cad/posture_correction_v1/generated/search.json','cad/functional_motion_v3/audit.py',
        'cad/gesture_hand_v3/audit_gestures.py','cad/articulation_drive_v2/verify.py']:sources[p]=prior.sha(ROOT/p)
    if not all(prior.sha(ROOT/p)==h for p,h in sources.items()):raise ValueError('Changed input')
    native,v,old,scene,parts=prior.load();engine=native.PairScreen(v,parts,scene['hand'],old['design'],0.,True);cache={}
    def evaluate(q):
        key=tuple(float(x).hex() for x in q)
        if key in cache:
            result=copy.deepcopy(cache[key]);result['reused_identical_whole_pose']=True;return result
        result=prior.compact(engine,q);cache[key]=copy.deepcopy(result);return result
    report=dict(completed=False,schema='posture_correction_v1.native.1',source_sha256=sources,cases=[],paths=[],
        hand_parts=364,pairs_per_pose=66066,geometry_changed=False,axes_changed=False,lengths_changed=False,
        reporting_floor_mm3=.02,boolean_tolerance_mm=1e-6,clearance_micromargin_target_applied=False,
        scope='Whole saved hand endpoints and<=5deg staged paths. Reverse is the exact saved forward sequence reversed; no dynamics, continuous or physical approval.',
        human_form_note='Both crossing fingers may flex. Tip posture uses actual distal fronts, not forced palmar-pad coincidence. Visual naturalness is reviewed separately.',
        near_tip_note='Unloaded near-touch target0.25mm, no object retention or physical pinch claim.',selected=[])
    for ident in ['cross_middle_18','pinch_index']:
        source=next(c for c in old['cases'] if c['id']==ident)
        report['cases'].append(dict(id='before_'+ident,group='before',angles_deg=source['angles_deg'],native=evaluate(source['angles_deg']),status='original_failed_pose'))
        prior.write(a.output,report)
    candidates=[]
    for group in search['cross']:
        for i,row in enumerate(group['selected']):candidates.append(dict(id=group['id']+'_'+str(i),group=group['id'],**row))
    for i,row in enumerate(search['tip']['selected']):candidates.append(dict(id='near_tip_'+str(i),group='index_thumb_distal_contact',**row))
    for c in candidates:
        got=evaluate(c['angles_deg']);row=dict(id=c['id'],group=c['group'],angles_deg=c['angles_deg'],native=got,status='native_clear_endpoint_candidate')
        if c['group'].startswith('cross_'):
            row['crossings']=prior.cross_surfaces(v,old['design'],parts,scene,c['angles_deg'],c['crossings'])
            row['close_cross_surface_observed']=any(x['distance_is_local_to_cross'] and x['segment_surface_distance_mm']<=4. for x in row['crossings'])
            if not row['close_cross_surface_observed']:row['status']='projection_cross_but_close_surface_not_confirmed'
        else:
            row['metrics']=c['metrics'];frames=v.frames(old['design'],c['angles_deg']);neutral=engine.neutral
            names=['index_segment3_rounded_tip_cap','thumb_segment3_rounded_tip_cap'];bodymap={r['name']:r['body'] for r in scene['hand']}
            moved=[v.relocate(parts[n],frames[bodymap[n]],neutral[bodymap[n]]) for n in names]
            row['native_tip_surface_gap_mm']=float(moved[0].distance(moved[1]))
            if not c['metrics']['tip_target_met'] or row['native_tip_surface_gap_mm']>1.:row['status']='near_tip_target_not_reached'
        if got['collision_count']:row['status']='actual_material_interference'
        if got['operation_failures']:row['status']='native_calculation_uncertainty'
        report['cases'].append(row);prior.write(a.output,report)
        print(json.dumps(dict(case=row['id'],collisions=got['collision_count'],status=row['status'])),flush=True)
    for group in [g['id'] for g in search['cross']]+['index_thumb_distal_contact']:
        eligible=[c for c in report['cases'] if c['group']==group and c['status']=='native_clear_endpoint_candidate']
        selected=False
        for target in eligible:
            for routeid,waypoints in make_routes(group,target['angles_deg'],old):
                samples=[];failure=False
                for si,(qa,qb) in enumerate(zip(waypoints[:-1],waypoints[1:])):
                    steps=max(1,int(np.ceil(np.max(np.abs(qb-qa))/5.)))
                    for k in range(steps+1):
                        if si and not k:continue
                        q=qa.copy() if not k else qb.copy() if k==steps else qa+(qb-qa)*k/steps
                        got=evaluate(q);samples.append(dict(segment=si,fraction=k/steps,native=got))
                        if got['collision_count'] or got['operation_failures']:
                            failure=True;break
                    if failure:break
                path=dict(id=target['id']+'__'+routeid,target=target['id'],group=group,waypoints_deg=[q.tolist() for q in waypoints],
                    status='aborted_at_first_reported_failure' if failure else 'sampled_approach_release_clear',forward_observations=samples,
                    forward_sample_count=len(samples),max_joint_step_deg=5.,reverse='Not run after a failed forward path' if failure else 'Exact same saved configurations in reverse order',
                    total_approach_release_samples=None if failure else len(samples)*2-1)
                report['paths'].append(path);prior.write(a.output,report)
                print(json.dumps(dict(path=path['id'],status=path['status'],forward_samples=len(samples))),flush=True)
                if not failure:
                    report['selected'].append(dict(group=group,case=target['id'],path=path['id'],angles_deg=target['angles_deg']))
                    selected=True;break
            if selected:break
        if not eligible:report['paths'].append(dict(group=group,status='no_reached_clear_endpoint_to_route'))
    report.update(completed=True,input_hash_guard=all(prior.sha(ROOT/p)==h for p,h in sources.items()),elapsed_s=time.monotonic()-start,
        unique_whole_pose_native_evaluations=len(cache))
    prior.write(a.output,report)
    if not report['input_hash_guard']:raise ValueError('Inputs changed')


if __name__=='__main__':main()
