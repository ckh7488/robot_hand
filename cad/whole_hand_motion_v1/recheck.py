"""Reproduce core positive/negative findings from a committed clean snapshot.

passed means agreement with the checkpoint's failures, not a collision-free hand.
"""
import argparse,hashlib,importlib.util,json,math
from pathlib import Path
import numpy as np
import cadquery as cq

HERE=Path(__file__).resolve().parent
def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def mod(name,p):
    s=importlib.util.spec_from_file_location(name,p);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    root=HERE.parents[1];manifest=read(HERE/'manifest.json');hashes=manifest['files']
    def guard():return all(sha(root/k)==h for k,h in hashes.items())
    if not guard():raise ValueError('Snapshot differs from committed closure')
    source=read(HERE/'reviews/discovery.json');trans=read(HERE/'reviews/transitions.json')
    selected=['gesture_thumb_up','four_fingers_hook','index_ordinary_extension_mcp','index_pip_+135','thumb_mcp_+90','fan_1','four_fingers_curl']
    cases={c['id']:c for c in source['cases']};probes=[dict(id=n,expected=cases[n]['result']) for n in selected]
    failures=[dict(id=p['source']+'__'+p['target']+'@'+str(o['fraction']),expected=o['result'])
              for p in trans['paths'] for o in p['observations'] if o['result']['collision_count']]
    probes+=failures[:2]
    n=mod('_core_native',root/'cad/gesture_hand_v3/audit_gestures.py');v=n.load_v2(root)
    scene=read(root/'cad/manufacturing_v2/generated/scene.json');parts={}
    for r in scene['hand']:parts[r['name']]=cq.importers.importStep(str(root/'cad/manufacturing_v2'/r['step'])).val()
    engine=n.PairScreen(v,parts,scene['hand'],source['requests']['design'],0.,True)
    results=[];differences=[]
    for p in probes:
        expected=p['expected'];actual=engine.evaluate(expected['angles_deg'])
        def signature(r):return {(h['a'],h['b']):h['material_intersection_mm3'] for h in r['collision_details']}
        e,b=signature(expected),signature(actual)
        if e.keys()!=b.keys():differences.append(dict(probe=p['id'],reason='collision_pair_set_changed'))
        for pair in e.keys() & b.keys():
            if not math.isfinite(b[pair]) or abs(e[pair]-b[pair])>1e-5:differences.append(dict(probe=p['id'],pair=pair,expected=e[pair],actual=b[pair]))
        if actual['operation_failures']:differences.append(dict(probe=p['id'],reason='operation_failure',details=actual['operation_failures']))
        actual.pop('exact_candidate_results');actual.pop('near_pairs');results.append(dict(id=p['id'],result=actual))
        print(json.dumps(dict(probe=p['id'],collisions=actual['collision_count'])),flush=True)
    # Recheck the changed-part layer of the later whole-hand candidate too.
    latest_ids=['gesture_thumb_up','thumb_mcp_+90','four_fingers_curl','fan_1']
    latest_source=read(HERE/'reviews/latest_hand.json');latest_expected={c['id']:c for c in latest_source['cases']}
    changed=set(latest_source['partition']['modified'])|set(latest_source['partition']['added'])
    latest_scene=read(root/'cad/thumb_ip_drive_v1/generated/scene.json');latest_parts={}
    for row in latest_scene['hand']:
        if row['name'] not in changed:latest_parts[row['name']]=parts[row['name']]
        else:
            path=root/row['step'] if row['location']=='repo' else root/'cad/thumb_ip_drive_v1/generated'/row['step']
            latest_parts[row['name']]=cq.importers.importStep(str(path)).val()
    full_latest=n.PairScreen(v,latest_parts,latest_scene['hand'],source['requests']['design'],0.,True)
    full_latest.cache={key:value for key,value in engine.cache.items() if key[0] not in changed and key[1] not in changed}
    latest_results=[]
    crossed=read(HERE/'reviews/crossed_fingers.json')
    cross_case=next((c for c in crossed['cases'] if c['id']==crossed['selected']),crossed['cases'][0])
    expectations=[('latest:'+ident,latest_expected[ident]) for ident in latest_ids]+[('crossed:'+cross_case['id'],cross_case['result'])]
    for ident,expected in expectations:
        c=full_latest.evaluate(expected['angles_deg']);e=signature(expected);b=signature(c)
        if e.keys()!=b.keys() or c['operation_failures']:differences.append(dict(probe=ident,reason='pair set or operation failure'))
        for pair in e.keys() & b.keys():
            if abs(e[pair]-b[pair])>1e-5:differences.append(dict(probe=ident,pair=pair,expected=e[pair],actual=b[pair]))
        c.pop('exact_candidate_results');c.pop('near_pairs');latest_results.append(dict(id=ident,result=c))
        print(json.dumps(dict(probe=ident,collisions=c['collision_count'])),flush=True)
    # Recheck the selected target's cross-digit margin, which is distinct from
    # the material-volume reporting threshold. The full path gap run is not repeated.
    gap_expected=read(HERE/'reviews/cross_clearance.json');q=cross_case['angles_deg']
    bodies={r['name']:r['body'] for r in latest_scene['hand']};frames=v.frames(source['requests']['design'],q);neutral=v.frames(source['requests']['design'],np.zeros(20))
    digit_shapes={name:v.relocate(shape,frames[bodies[name]],neutral[bodies[name]]) for name,shape in latest_parts.items() if name.startswith(('index_','middle_'))}
    boxes={name:v.bounds(shape) for name,shape in digit_shapes.items()};near=[];gap_goal=gap_expected['gap_goal_mm'];pair_count=0
    for x in digit_shapes:
        if not x.startswith('index_'):continue
        for y in digit_shapes:
            if not y.startswith('middle_'):continue
            pair_count+=1
            lower=float(np.linalg.norm(np.maximum(np.maximum(boxes[x][0]-boxes[y][1],boxes[y][0]-boxes[x][1]),0.)))
            if lower>gap_goal+1e-6:continue
            distance=float(digit_shapes[x].distance(digit_shapes[y]))
            if not math.isfinite(distance) or distance<0:raise ValueError('Invalid gap replay')
            near.append(dict(pair=[x,y],gap_mm=distance))
    min_near=min((r['gap_mm'] for r in near),default=math.inf)
    expected_target=gap_expected['samples'][-1]
    if (min_near>=gap_goal)!=expected_target['meets_half_mm_goal']:differences.append(dict(probe='crossed_target_gap',reason='margin goal result changed'))
    gap_replay=dict(goal_mm=gap_goal,meets_half_mm_goal=min_near>=gap_goal,nominal_gap_lower_bound_mm=min(gap_goal,min_near),all_cross_digit_pairs=pair_count,exact_near_results=near,scope='Selected target only; no path clearance replay')
    final_guard=guard();summary=dict(target_commit=a.commit,passed=not differences and final_guard,
        meaning='Core findings reproduced, including actual collisions. This is NOT a motion/assembly release.',
        whole_hand_motion_approved=False,independent_audit=False,mode='core_only',
        tested_static_cases=len(selected),tested_transition_intermediate_poses=len(failures[:2]),
        replayed_full72_pose_suite=False,replayed_full108_transition_suite=False,
        source_manifest_sha256=sha(HERE/'manifest.json'),guarded_files=len(hashes),input_hash_guard=final_guard,
        comparison_absolute_volume_tolerance_mm3=1e-5,differences=differences,cases=results,
        latest_hand_core=latest_results,tested_latest_hand_layer_cases=len(latest_ids),tested_crossed_target=cross_case['id'],crossed_path_replayed=False,selected_cross_target_gap=gap_replay)
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(summary,separators=(',',':'),allow_nan=False),encoding='utf-8')
    return 0 if summary['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
