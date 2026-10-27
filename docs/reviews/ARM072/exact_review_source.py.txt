"""Exact-SHA bounded special48 service replay and original inventory contracts."""
import argparse,importlib.util,subprocess,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm072_review',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,d,ROOT,HERE=m.n,m.np,m.d,m.ROOT,m.HERE
metrics=n.m.drive.module('_arm072_metrics',ROOT/'cad/recessed_mounting_root_v1/review_core.py')
from OCP.OSD import OSD_ThreadPool
from OCP.BRepAdaptor import BRepAdaptor_Surface


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1') and not args.output.exists()
    man=n.p.read(ROOT/'checkpoints/ARM072.manifest.json');frozen={}
    for k in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(n.p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[k])
    def guard():return all(n.m.drive.sha(ROOT/f)==z['sha256'] for f,z in man['files'].items()) and all(n.m.drive.sha(ROOT/f)==h for f,h in frozen.items())
    assert guard() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    rows,_,source=m.context();lookup={z['name']:z for z in rows};raw=n.p.read(HERE/'reviews/check.json');selected=n.p.read(HERE/'reviews/selected_check.json');continuation=n.p.read(HERE/'reviews/continuation_check.json');ev=n.p.read(HERE/'reviews/evidence.json');ev2=n.p.read(HERE/'reviews/selected_evidence_final.json');ev3=n.p.read(HERE/'reviews/continuation_evidence.json');survey=n.p.read(HERE/'reviews/survey.json');prior=n.p.read(ROOT/'cad/datum_plane_packet_entry_v1/reviews/check.json')
    result=dict(completed=False,target_commit=args.commit,differences=[],native_samples=[],common_samples=[],geometry_comparisons=[],contracts={},manifest_files=len(man['files']),frozen_files=len(frozen),OCC_pool_threads=1,same_agent_not_independent_audit=True)
    def save():n.a.save(args.output,result)
    for dataset in [raw,selected,continuation]:
      for p in dataset['paths']:
        for s in m.replay_samples(p):
            got=n.c.material(m.moved([lookup[k] for k in p['moving_names']],np.array(s['transform'])),[lookup[k] for k in p['present_names']]);want=dataset['native'][s['id']]
            a,b=d.overlap_map(got),d.overlap_map(want);err=max([abs(a[k]['material_intersection_mm3']-b[k]['material_intersection_mm3']) for k in a.keys()&b.keys()]+[0.])
            ok=a.keys()==b.keys() and err<1e-5 and got['pairs_in_scope']==want['pairs_in_scope'] and got['operation_failures']==want['operation_failures']
            result['native_samples'].append(dict(id=s['id'],native=got,matched=bool(ok),max_volume_difference_mm3=err))
            if not ok:result['differences'].append(s['id'])
            save();print(s['id'],bool(ok),flush=True)
    contracts=result['contracts'];packet=m.packets(rows);special=next(p for p in packet if p['module']=='thumb_ip_actuator')
    contracts['original16packet408_final886_start670_prior8_retained']=packet==raw['original16packets'] and sum(len(p['names']) for p in packet)==408 and sorted(lookup)==raw['original_final886_names'] and raw['initial_world_present_names']==prior['packet_final_present_names'] and len(raw['restored_before'])==8
    contracts['special48_body44_rack4_no_generic_rows']=special==raw['original_special_packet']==survey['original_packet'] and len(special['names'])==48 and len(special['body_names'])==44 and len(special['mounting_names'])==4 and not any('thumb_ip_actuator'+s in special['names'] for s in ['_rotor','_spool','_guide_spacer_1','_guide_spacer_2'])
    core={special['module']+'_motor',special['module']+'_body_screws','thumb_ip_stock_output_horn','thumb_ip_stock_central_retainer','thumb_ip_stock_rear_idler','thumb_ip_two_tail_spool',*[f'thumb_ip_spool_tap6_{i}' for i in range(1,5)],'thumb_ip_tail_clamp_1','thumb_ip_tail_clamp_2'}
    contracts['original_preassembled_reference12_internal_not_newly_approved']=sorted(core)==raw['unit_reference12_names'] and len(core)==12 and not raw['internal_reference_unit_retested'] and raw['old_native_groups_repeated']==0 and raw['inherited_ARM065_corefalse_ARM059_M3plus13_and_all_rawfalse_holds_unresolved']
    placement=next(p for p in n.p.read(ROOT/'cad/eight_bank_layout_v1/reviews/check.json')['placements'] if p['module']==special['module']);P=np.array(placement['proposed_installation_transform']);delta=np.array(placement['transform_from_current'])
    guide=next(p for p in n.p.read(ROOT/'cad/wire_guides_v1/generated/scene.json')['groups'] if p['module']==special['module']);frame=delta@np.array(guide['frame_world_mm']);r=-P[:3,2];normal=P[:3,0];opening=delta[:3,:3]@np.array([0.,0.,1.])
    contracts['actual_special_basis_and_open_groove_frame']=all(np.max(abs(a-np.array(b)))<1e-10 for a,b in [(P,survey['placement']),(frame,raw['guide_actual_frame']),(r,raw['radial']),(normal,raw['normal']),(opening,raw['guide_opening_axis'])])
    world=set(raw['initial_world_present_names']);bench={special['module']+'_cradle'};carrier={special['module']+'_adapter'};seq=True;outside=True;matrix_error=0.;identity_error=0.;failed=False
    for p in raw['paths']:
        present=world if p['stage']=='world' else bench if p['stage']=='motor_bench' else carrier
        seq=seq and not failed and set(p['present_names'])==present and not present&set(p['moving_names']) and set(p['absent_names'])==set(lookup)-present
        for s in p['samples']:matrix_error=max(matrix_error,float(np.max(abs(m.translate(s['translation_mm'])-np.array(s['transform'])))))
        axis=np.array(p['approach_axis']);moving=m.moved([lookup[k] for k in p['moving_names']],np.array(p['samples'][0]['transform']));fixed=[lookup[k] for k in present];mb=m.project_bounds(moving,axis);fb=m.project_bounds(fixed,axis);gap=max(fb[0]-mb[1],mb[0]-fb[1]);outside=outside and gap>0 and abs(gap-p['initial_projection_separation_mm'])<1e-5
        if p['completed']:
            present.update(p['moving_names']);identity_error=max(identity_error,float(np.max(abs(np.array(p['samples'][-1]['transform'])-np.eye(4)))))
        else:
            failed=True;seq=seq and p['samples'][-1]['id']==raw.get('first_blocked_sample') and bool(raw['native'][p['samples'][-1]['id']]['overlaps'] or raw['native'][p['samples'][-1]['id']]['operation_failures'])
    contracts['all_paths_full_stage_outside_starts']=bool(outside)
    contracts['explicit_omissions_sequence_stop_first_block_final_identity']=bool(seq and matrix_error<1e-10 and identity_error<1e-10 and bench==set(raw['final_motor_bench_present_names']) and carrier==set(raw['final_carrier_bench_present_names']))
    contracts['actual_present_pending_and_remaining_front7']=world==set(raw['packet_final_present_names']) and world|set(raw['packet_final_pending_names'])==set(lookup) and not world&set(raw['packet_final_pending_names']) and [p['module'] for p in packet if set(p['names'])<=world]==raw['restored_modules'] and raw['explicitly_later_front7_modules']==[p['module'] for p in packet if p['row']==1 and p['module']!=special['module']]
    prefix=selected['inherited_prefix_paths'];inherit=prefix==raw['paths'][:10] and sum(len(p['samples']) for p in prefix)==selected['inherited_native_sample_scope']==78 and selected['inherited_native_groups_repeated']==0
    inherit=inherit and all(p['completed'] and not raw['native'][s['id']]['overlaps'] and not raw['native'][s['id']]['operation_failures'] for p in prefix for s in p['samples'])
    inherit=inherit and all(n.m.drive.sha(ROOT/f)==h for f,h in raw['source_sha256'].items()) and selected['initial_world_present_names']==raw['initial_world_present_names']
    carrier=set(selected['initial_carrier_bench_present_names']);seq2=carrier==set(raw['paths'][10]['present_names']) and len(carrier)==25 and 'thumb_ip_barrel_adjuster_0' not in carrier;world2=set(selected['initial_world_present_names']);failed2=False;outside2=True
    for p in selected['paths']:
        present=world2 if p['stage']=='world' else carrier
        seq2=seq2 and not failed2 and present==set(p['present_names']) and not present&set(p['moving_names']) and set(p['absent_names'])==set(lookup)-present
        for s in p['samples']:matrix_error=max(matrix_error,float(np.max(abs(m.translate(s['translation_mm'])-np.array(s['transform'])))))
        axis=np.array(p['approach_axis']);moving=m.moved([lookup[k] for k in p['moving_names']],np.array(p['samples'][0]['transform']));mb=m.project_bounds(moving,axis);fb=m.project_bounds([lookup[k] for k in present],axis);gap=max(fb[0]-mb[1],mb[0]-fb[1]);outside2=outside2 and gap>0 and abs(gap-p['initial_projection_separation_mm'])<1e-5
        if '_nut_lateral_before_adjuster' in p['id']:
            line=p['id'].split('_')[1];seq2=seq2 and 'thumb_ip_barrel_adjuster_'+line not in present and np.max(abs(axis-r))<1e-10
        if p['completed']:present.update(p['moving_names']);identity_error=max(identity_error,float(np.max(abs(np.array(p['samples'][-1]['transform'])-np.eye(4)))))
        else:failed2=True;seq2=seq2 and p['samples'][-1]['id']==selected.get('first_blocked_sample') and bool(selected['native'][p['samples'][-1]['id']]['overlaps'] or selected['native'][p['samples'][-1]['id']]['operation_failures'])
    contracts['same_successful78_prefix_proof_no_native_repeat']=bool(inherit)
    contracts['selected_both_nuts_before_adjuster_outside_lists_identity_stop']=bool(seq2 and outside2 and matrix_error<1e-10 and identity_error<1e-10 and carrier==set(selected['final_carrier_bench_present_names']))
    contracts['selected_world670_and_original_pending_contract']=world2==set(selected['packet_final_present_names']) and world2|set(selected['packet_final_pending_names'])==set(lookup) and not world2&set(selected['packet_final_pending_names']) and [p['module'] for p in packet if set(p['names'])<=world2]==selected['restored_modules']
    prefix3=continuation['inherited_prefix_paths'];proof=prefix3==raw['paths'][:10]+selected['paths'][:3] and sum(len(p['samples']) for p in prefix3)==continuation['inherited_native_sample_scope']==99 and continuation['inherited_native_groups_repeated']==0
    for p in prefix3:
        owner=raw if p['id'] in {z['id'] for z in raw['paths'][:10]} else selected
        proof=proof and p['completed'] and all(not owner['native'][s['id']]['overlaps'] and not owner['native'][s['id']]['operation_failures'] for s in p['samples'])
    proof=proof and all(n.m.drive.sha(ROOT/f)==h for f,h in selected['source_sha256'].items())
    carrier3=set(continuation['initial_carrier_bench_present_names']);seq3=carrier3==set(selected['paths'][3]['present_names']) and len(carrier3)==28;world3=set(continuation['initial_world_present_names']);outside3=True;failed3=False
    for p in continuation['paths']:
        present=world3 if p['stage']=='world' else carrier3
        seq3=seq3 and not failed3 and present==set(p['present_names']) and not present&set(p['moving_names']) and set(p['absent_names'])==set(lookup)-present
        for s in p['samples']:matrix_error=max(matrix_error,float(np.max(abs(m.translate(s['translation_mm'])-np.array(s['transform'])))))
        axis=np.array(p['approach_axis']);moving=m.moved([lookup[k] for k in p['moving_names']],np.array(p['samples'][0]['transform']));mb=m.project_bounds(moving,axis);fb=m.project_bounds([lookup[k] for k in present],axis);gap=max(fb[0]-mb[1],mb[0]-fb[1]);outside3=outside3 and gap>0 and abs(gap-p['initial_projection_separation_mm'])<1e-5
        if p['id'].startswith('jaw_outer_side_first_arm059_straight_jaw_'):
            line=p['id'].rsplit('_',1)[1];seq3=seq3 and 'motor_housing_clamp_'+line+'_jaw_1' not in present and np.max(abs(axis-r))<1e-10
        if p['completed']:present.update(p['moving_names']);identity_error=max(identity_error,float(np.max(abs(np.array(p['samples'][-1]['transform'])-np.eye(4)))))
        else:failed3=True;seq3=seq3 and p['samples'][-1]['id']==continuation.get('first_blocked_sample') and bool(continuation['native'][p['samples'][-1]['id']]['overlaps'] or continuation['native'][p['samples'][-1]['id']]['operation_failures'])
    contracts['same99_prefix_and_outer_jaw_first_wholeoutside_contract']=bool(proof and seq3 and outside3 and matrix_error<1e-10 and identity_error<1e-10 and carrier3==set(continuation['final_carrier_bench_present_names']))
    contracts['continuation_original_present_pending_rear8_retained']=world3==set(continuation['packet_final_present_names']) and world3|set(continuation['packet_final_pending_names'])==set(lookup) and not world3&set(continuation['packet_final_pending_names']) and [p['module'] for p in packet if set(p['names'])<=world3]==continuation['restored_modules']
    properties=True;axes=0
    for z in survey['actual48_parts']:
        sh=lookup[z['name']]['shape'];properties=properties and np.max(abs(np.array(n.c.m.pack._bbox(sh))-np.array(z['bbox'])))<1e-5
        for q in z['cylinders']:
            face=sh.Faces()[q['index']];c=BRepAdaptor_Surface(face.wrapped).Cylinder();axes+=1
            properties=properties and face.geomType()=='CYLINDER' and abs(c.Radius()-q['radius_mm'])<1e-5 and np.max(abs(np.array(c.Axis().Direction().Coord())-q['axis']))<1e-5 and np.max(abs(np.array(c.Axis().Location().Coord())-q['point']))<1e-5
        for q in z['normal_planes']:
            face=sh.Faces()[q['index']];p=BRepAdaptor_Surface(face.wrapped).Plane();properties=properties and abs(face.Area()-q['area_mm2'])<1e-5 and np.max(abs(np.array(p.Location().Coord())-q['point']))<1e-5
    contracts['actual48_bounds_and_selected_cylindrical_axes_planes']=bool(properties)
    for evidence in [ev,ev2,ev3]:
      saved=n.c.load(n.p.read(ROOT/evidence['common_scene'])['parts']) if evidence['common_scene'] else []
      for q,z in zip(evidence['common_queries'],saved):
        sh=n.g.original.transform(lookup[q['lead']]['shape'],np.array(q['lead_transform'])).intersect(lookup[q['part']]['shape'],tol=1e-6);ok=sh.isValid() and len(sh.Solids())==q['solids'] and abs(sh.Volume()-q['volume_mm3'])<1e-5
        result['common_samples'].append(dict(name=q['name'],matched=bool(ok),volume_mm3=sh.Volume(),solids=len(sh.Solids())));g=dict(name=q['name'],**metrics.geometry_match(sh,z['shape']));result['geometry_comparisons'].append(g)
        if not ok:result['differences'].append('common_'+q['name'])
        if not g['matched']:result['differences'].append('geometry_'+q['name'])
    for k,v in contracts.items():
        if not v:result['differences'].append(k)
    result.update(completed=True,native_groups=len(result['native_samples']),native_matches=sum(z['matched'] for z in result['native_samples']),geometry_shapes=len(result['geometry_comparisons']),geometry_matches=sum(z['matched'] for z in result['geometry_comparisons']),common_matches=sum(z['matched'] for z in result['common_samples']),actual_cylindrical_faces_compared=axes,all_translation_matrix_max_error=matrix_error,completed_final_identity_max_error=identity_error,input_hash_guard=guard(),checkout_clean=not git('status','--porcelain=v1'),
        replay_scope='Initial/selected/continuation new completed service path endpoints and every reached sample of each first blocked path;world first+last5 if reached. Exact78/99 prefix proofs inherited,no native repeats. Original16packet408/886/start670/rear8,special48=body44+rack4,reference12 internal untested,actual48 bounds/selected interface cylinderaxes/planes/frame,present/absent/wholeoutside starts/stopfirstfailure/finalidentity. Saved representative commons/signatures if produced. No old rear paths/old output internal or ARM05931/oldcommon/PCD615 repeated. Not all intermediate paths/trimBoolean/pixels/armvisualunits/continuousfactory/tools/torque/captive/mating/harnessflex/physical20drive/fullarm/exterior approval.')
    result['initial_core_passed']=not result['differences'] and result['input_hash_guard'] and result['checkout_clean'];save();print('core',result['initial_core_passed'],result['differences'],flush=True)


if __name__=='__main__':main()
