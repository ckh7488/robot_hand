"""Exact-commit bounded replay of the new support, phase and local geometry."""
import argparse
import importlib.util
import subprocess
import sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm057_review_model',Path(__file__).with_name('model.py'))
a=importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)
n,np,d,ROOT,HERE=a.n,a.np,a.d,a.ROOT,a.HERE
measure=n.m.drive.module('_arm057_metrics',ROOT/'cad/long_housing_route_v1/review_core.py')
wrap=n.m.drive.module('_arm057_wrap',HERE/'wrap_diagnostic.py')
attachment=n.m.drive.module('_arm057_attachment',HERE/'attachment.py')
classify=n.m.drive.module('_arm057_classify',HERE/'classify.py')
from OCP.OSD import OSD_ThreadPool


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--commit',required=True)
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args()
    def git(*xs):
        return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1')
    assert not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
    manifest=n.p.read(ROOT/'checkpoints/ARM057.manifest.json')
    frozen={}
    for k in ['geometry_sha256','code_sha256','report_sha256']:
        frozen.update(n.p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[k])
    def guard():
        return all(n.m.drive.sha(ROOT/f)==v['sha256'] for f,v in manifest['files'].items()) and all(n.m.drive.sha(ROOT/f)==h for f,h in frozen.items())
    assert guard() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    raw=n.p.read(HERE/'reviews/check.json')
    _,baseline,fixed,packet,T,S0,S,prior,source=a.context()
    new,moved,clock,design=a.build(packet,T,S,baseline,source)
    fresh,routes=a.cords(T,S0,baseline)
    saved=n.c.load(n.p.read(ROOT/raw['scene'])['parts'])
    cord0=n.c.load(n.p.read(ROOT/raw['cord_scene'])['parts'])
    changed=saved+moved+clock
    result=dict(completed=False,target_commit=args.commit,differences=[],native_samples=[],geometry_comparisons=[],route_comparisons=[],
        manifest_files=len(manifest['files']),frozen_files=len(frozen),OCC_pool_threads=1,
        current862_installation_replaced=False,candidate_adopted=False,same_agent_not_independent_audit=True,
        prior_ARM051_to_ARM056_or_LINK050_reexecuted=False,whole_drive_or_exterior_approved=False)
    def save(): n.a.save(args.output,result)
    def geometry(left,right):
        for x,y in zip(left,right):
            assert x['name']==y['name']
            item=dict(name=x['name'],**measure.match(measure.metrics(x['shape']),measure.metrics(y['shape'])))
            result['geometry_comparisons'].append(item)
            if not item['matched']: result['differences'].append('geometry_'+x['name']+'_'+str(len(result['geometry_comparisons'])))
            save()
    def compare_routes(got,want,label):
        error=max(float(np.max(abs(np.array(x[k])-y[k]))) for x,y in zip(got,want) for k in ['tangent_exit_world_mm','R5_entry_world_mm','R5_exit_world_mm','shoulder_world_mm','mouth_world_mm','wrap_angle_deg','local_cord_length_mm','endpoint_gap_max_mm'])
        ok=error<1e-10
        result['route_comparisons'].append(dict(state=label,matched=bool(ok),maximum_difference=error))
        if not ok: result['differences'].append('routes_'+label)
    save()
    geometry(new,saved)
    geometry(fresh,cord0)
    compare_routes(routes,raw['local_routes'],'command0')
    errors=[]
    for x,y in zip(design['ports'],raw['design']['ports']):
        assert x['moved_names']==y['moved_names']
        errors += [float(np.max(abs(np.array(x[k])-y[k]))) for k in ['Q','transform_from_ARM055','free_port_world_mm','shoulder_world_mm','original_jaw_removed_mm3']]
    errors.append(float(np.max(abs(np.array(design['clock_transform_proposal'])-raw['design']['clock_transform_proposal']))))
    ok=max(errors)<1e-10 and design['four_PCD12_axes_permuted'] and design['clocked_names']==raw['design']['clocked_names']
    result['installation_contract']=dict(matched=bool(ok),relocated_fixed_rows=16,clocked_output_rows=7,PCD12_axes=4,maximum_difference=max(errors),original_M3_stack_mm=1.)
    if not ok: result['differences'].append('installation_contract')
    diagnostic=wrap.evaluate(routes)
    expected=n.p.read(HERE/'reviews/wrap_diagnostic.json')
    ok=all(diagnostic[k]==expected[k] for k in diagnostic)
    result['wrap_diagnostic_matched']=bool(ok)
    if not ok: result['differences'].append('wrap_diagnostic')
    def compare(label,got):
        want=raw['native'][label]
        l,r=d.overlap_map(got),d.overlap_map(want)
        error=max([abs(l[k]['material_intersection_mm3']-r[k]['material_intersection_mm3']) for k in l.keys()&r.keys()]+[0.])
        ok=l.keys()==r.keys() and error<1e-5 and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope']
        result['native_samples'].append(dict(id=label,native=got,matched=bool(ok),maximum_volume_difference_mm3=error))
        if not ok: result['differences'].append(label)
        save()
        print(label,bool(ok),flush=True)
    def internal(rows): return d.k.flat([n.c.material([z],rows[i+1:]) for i,z in enumerate(rows)])
    unchanged=[z for rows in fixed.values() for z in rows]
    compare('changed26__unchanged846_neutral',n.c.material(changed,unchanged))
    compare('new3__relocated16',n.c.material(saved,moved))
    compare('clocked7__fixed_terminal19',n.c.material(clock,saved+moved))
    compare('new3_internal',internal(saved))
    compare('local_cords12__hardware872',n.c.material(cord0,changed+unchanged))
    compare('local_cords12_internal',internal(cord0))
    arm=n.p.read(ROOT/'cad/modular_roots_v9/design.json')['arm']
    for state in raw['arm_motion']:
        F=d.transforms(state['angles_deg'],arm)
        assert max(np.max(abs(F[k]-state['transforms'][k])) for k in F)<1e-10
        moving=[dict(z,shape=n.g.original.transform(z['shape'],F['forearm'])) for z in changed+cord0]
        for target in state['compared_frames']:
            obstacle=[dict(z,shape=n.g.original.transform(z['shape'],F[target])) for z in fixed[target]]
            compare(state['id']+'__new26_and_local12__'+target,n.c.material(moving,obstacle))
    stock_names={'thumb_ip_stock_output_horn','thumb_ip_stock_central_retainer'}
    stock=[z for z in unchanged if z['name'] in stock_names]
    statics=saved+moved+[z for z in unchanged if z['name'] not in stock_names]
    for state in raw['motor_command_states']:
        fresh_local,info=a.cords(T,S0,baseline,state['command_deg'])
        local=n.c.load(n.p.read(ROOT/state['cord_scene'])['parts'])
        geometry(fresh_local,local)
        label='command_'+str(int(state['command_deg']))
        compare_routes(info,state['routes'],label)
        centre=(T@np.r_[a.p.terminal.M[:3,3],1.])[:3]
        C=d.around(centre,(0,1,0),-state['command_deg'])
        assert np.max(abs(C-state['transform']))<1e-10
        moving=[dict(z,shape=n.g.original.transform(z['shape'],C)) for z in clock+stock]
        compare(label+'__output9__stationary863',n.c.material(moving,statics))
        compare(label+'__cords12__hardware872',n.c.material(local,moving+statics))
        compare(label+'__cords_internal',internal(local))
    adapter=next(z['shape'] for z in unchanged if z['name']=='thumb_ip_actuator_adapter')
    distance=float(saved[0]['shape'].distance(adapter))
    result['frame_adapter_distance_matched']=abs(distance-raw['frame_adapter_distance_mm'])<1e-5
    if not result['frame_adapter_distance_matched']: result['differences'].append('frame_adapter_distance')
    got,_=attachment.query(raw)
    expected_attachment=n.p.read(HERE/'reviews/attachment.json')['result']
    error=max([abs(x['distance_mm']-y['distance_mm']) for x,y in zip(got['distances'],expected_attachment['distances'])]+
        [abs(x['retained_seat_outside_new_frame_mm3']-y['retained_seat_outside_new_frame_mm3']) for x,y in zip(got['pads'],expected_attachment['pads'])])
    ok=error<1e-5 and [z['outside_solids'] for z in got['pads']]==[z['outside_solids'] for z in expected_attachment['pads']]
    result['attachment']=dict(matched=bool(ok),actual=got,maximum_difference=error)
    if not ok: result['differences'].append('attachment')
    expected_class=n.p.read(HERE/'reviews/classification.json')
    got_class=classify.evaluate(raw,n.p.read(ROOT/'cad/motor_spool_coupling_v1/reviews/assessment.json'))
    result['saved_contact_classification_matched']=all(got_class[k]==expected_class[k] for k in got_class)
    if not result['saved_contact_classification_matched']: result['differences'].append('saved_contact_classification')
    result.update(completed=True,initial_core_passed=not result['differences'],native_groups=len(result['native_samples']),
        native_matches=sum(z['matched'] for z in result['native_samples']),geometry_matches=sum(z['matched'] for z in result['geometry_comparisons']),
        route_matches=sum(z['matched'] for z in result['route_comparisons']),input_hash_guard=guard(),checkout_clean=not git('status','--porcelain=v1'),
        replay_scope='Allnew36groups171676pairs/43records:changed26/846,new3/16,clock7/fixed19,new3internal,local12/872/66internal;24relationsat12finitearmposeswithnew26hardware+12command0cordreferences;independentmotorcommands-90/+90withactual9movingoutputparts/863fixed andlocalcords.39sourceSTEPmetrics epsilon1e-10/absolute1e-5;16fixed+7clockedplacements/4PCD12axes/actualM3stack,3statesofR10/R5routes,wrapbranch,1adapterdistance andseparate6stackdistances/2originalpadinclusioncuts.43rawcontactclassificationuses savedmetadata/baselineswithoutregionproofreplay. NooldARM051..056/LINK050native/renderpixels/alltrimBoolean/servicepaths/samewholelongmaterialmotion/0.77mapping/dynamic electrical/20torque/physical/exteriorapproval.')
    save()


if __name__=='__main__': main()
