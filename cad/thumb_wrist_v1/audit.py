"""Check the explicit wrist-directed profile against unchanged saved native CAD."""
from pathlib import Path
import argparse,copy,json,sys,time,traceback
import numpy as np
import wrist_profile as candidate_profile

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--mode',choices=['full','representative'],default='full')
    args=parser.parse_args();output=args.output.resolve()
    if output.exists():raise FileExistsError('Use a new output file')
    data=candidate_profile.load(args.repo)
    import audit_gestures as native
    repo,base=data['repo'],data['base'];design=data['design'];document=data['document']
    scene=native.read_json(base/'generated/scene.json');build=native.read_json(base/'generated/build_report.json')
    before=native.collect_inputs(repo,base,base/'poses.json',repo/'cad/accepted_form_v1',scene,build,data['catalog'])
    for p in [Path(__file__),candidate_profile.HERE/'wrist_profile.py',candidate_profile.HERE/'profile.json']:
        before[str(p.resolve())]=native.digest(p)
    original=copy.deepcopy(data['original_design']);original['joint_ranges_deg']['thumb_ip']=[-20,85]
    if original!=design:raise ValueError('Undeclared design change')
    observations,transitions=native.expand_poses(document,design,data['poses'].NAMES,'full',data['specification']['maximum_sample_step_deg'])
    full_observations=copy.deepcopy(observations)
    if args.mode=='representative':
        path=[x for x in observations if x['kind']=='transition']
        fractions=[0.,.25,.5,.75,data['specification']['finger_closing_phase_end'],1.]
        selected=[min(path,key=lambda x:(abs(x['path_fraction']-f),x['path_fraction'])) for f in fractions]
        wanted={x['id'] for x in selected}
        observations=[x for x in observations if x['kind']=='endpoint' or x['id'] in wanted]
    report=dict(schema='thumb_wrist_native_audit.v1',completed=False,mode=args.mode,errors=[],parts=[],mesh_topology=[],
        cases={},observations=observations,full_path_definition=transitions,direction=data['direction'],
        definition=document,full_sampling_observations=full_observations,
        geometry_changed=False,canonical_base_pose_changed=False,manufacturing_approved=False,hardware_performance_validated=False,
        range_change={'thumb_ip':{'original':[0,85],'candidate':[-20,85]}},
        thresholds=dict(material_overlap_mm3=native.OVERLAP_MM3,boolean_tolerance_mm=native.BOOLEAN_TOL_MM,
                        maximum_joint_step_deg=data['specification']['maximum_sample_step_deg']),
        scope='All349 native saved STEP parts, same-body pairs included, no exclusions. One forward finger-closing path with thumb fixed, then thumb IP0 to -15. Representative mode reruns only selected existing samples. No reverse, continuous-clearance, hardware or speed approval.')
    def save():
        after={p:native.digest(p) for p in before}
        report['input_hash_guard']=dict(passed=before==after,start=before,end=after)
        if before!=after:report['completed']=False
        native.write_json(output,report)
    try:
        v=native.load_v2(repo);parts=v.load_parts(scene,base,report)
        if len(parts)!=349 or not all(p['valid'] and p['solid_count']==1 for p in report['parts']):
            raise ValueError('Invalid or missing native part')
        report['assembly_roundtrip']=v.assembly_roundtrip(base/'generated/hand.step',scene['hand'],parts)
        if not report['assembly_roundtrip']['passed']:raise ValueError('Saved assembly mismatch')
        engine=native.PairScreen(v,parts,scene['hand'],design,1.,True)
        known={};saved=time.monotonic()
        for observation in observations:
            key=tuple(float(x).hex() for x in observation['angles_deg'])
            if key not in known:
                name='pose_'+str(len(known)).zfill(4);known[key]=name
                result=engine.evaluate(observation['angles_deg'])
                report['cases'][name]=result
                print(json.dumps(dict(case=name,observation=observation['id'],collisions=result['collision_count'],
                                      errors=len(result['operation_failures']))),flush=True)
            observation['case_id']=known[key]
            if time.monotonic()-saved>15:save();saved=time.monotonic()
        report['summary']=dict(parts=len(parts),unique_pose_count=len(known),observation_count=len(observations),
            full_path_sample_count=transitions[0]['sample_count'],
            audited_path_observations=sum(x['kind']=='transition' for x in observations),
            cases_with_material_collision=sum(bool(x['collision_details']) for x in report['cases'].values()),
            operation_failure_count=sum(len(x['operation_failures']) for x in report['cases'].values()),
            full_sampled_path_executed=args.mode=='full',maximum_actual_joint_step_deg=transitions[0]['maximum_actual_joint_step_deg'])
        report['completed']=True
    except Exception as exc:
        report['errors'].append(dict(error=repr(exc),traceback=traceback.format_exc()))
    save()
    print(json.dumps(dict(completed=report['completed'],summary=report.get('summary'),errors=report['errors'])),flush=True)
    return 0 if report['completed'] and report['input_hash_guard']['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
