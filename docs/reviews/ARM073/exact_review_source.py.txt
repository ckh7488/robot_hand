"""Fresh exact-SHA bounded kit replay with input/pose continuity contracts."""
import argparse,importlib.util,subprocess,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm073_review',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,d,ROOT,HERE=m.n,m.np,m.d,m.ROOT,m.HERE
continuity=n.m.drive.module('_arm073_continuity_review',HERE/'conflict_continuity.py')
metrics=n.m.drive.module('_arm073_metrics',ROOT/'cad/recessed_mounting_root_v1/review_core.py')
from OCP.OSD import OSD_ThreadPool
from OCP.BRepAdaptor import BRepAdaptor_Surface


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1') and not args.output.exists()
    man=n.p.read(ROOT/'checkpoints/ARM073.manifest.json');frozen={}
    for k in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(n.p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[k])
    def guard():return all(n.m.drive.sha(ROOT/f)==z['sha256'] for f,z in man['files'].items()) and all(n.m.drive.sha(ROOT/f)==h for f,h in frozen.items())
    assert guard() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    rows,_,source=m.context();lookup={z['name']:z for z in rows};raw=n.p.read(HERE/'reviews/check.json');ev=n.p.read(HERE/'reviews/evidence.json');survey=n.p.read(HERE/'reviews/survey.json');prior=n.p.read(ROOT/'cad/special_thumb_module_service_v1/reviews/check.json');sel=n.p.read(ROOT/'cad/special_thumb_module_service_v1/reviews/selected_check.json')
    result=dict(completed=False,target_commit=args.commit,differences=[],native_samples=[],common_samples=[],geometry_comparisons=[],contracts={},manifest_files=len(man['files']),frozen_files=len(frozen),OCC_pool_threads=1,same_agent_not_independent_audit=True)
    def save():n.a.save(args.output,result)
    for p in raw['paths']:
        for s in m.replay_samples(p):
            got=n.c.material(m.moved([lookup[k] for k in p['moving_names']],np.array(s['transform'])),[lookup[k] for k in p['present_names']]);want=raw['native'][s['id']]
            a,b=d.overlap_map(got),d.overlap_map(want);err=max([abs(a[k]['material_intersection_mm3']-b[k]['material_intersection_mm3']) for k in a.keys()&b.keys()]+[0.])
            ok=a.keys()==b.keys() and err<1e-5 and got['pairs_in_scope']==want['pairs_in_scope'] and got['operation_failures']==want['operation_failures']
            result['native_samples'].append(dict(id=s['id'],native=got,matched=bool(ok),max_volume_difference_mm3=err))
            if not ok:result['differences'].append(s['id'])
            save();print(s['id'],bool(ok),flush=True)
    c=result['contracts'];packet=m.packets(rows)
    c['original16packet408_final886_start670_prior8_retained']=packet==raw['original16packets'] and sum(len(p['names']) for p in packet)==408 and sorted(lookup)==raw['original_final886_names'] and raw['initial_world_present_names']==prior['initial_world_present_names'] and len(raw['restored_modules'])==8
    special=next(p for p in packet if p['module']=='thumb_ip_actuator');c['special48_body44_rack4_all_original_rows_pending']=special==raw['original_special_packet'] and len(special['names'])==48 and len(special['body_names'])==44 and len(special['mounting_names'])==4 and set(special['names'])<=set(raw['packet_final_pending_names'])
    prefix=prior['paths'][:10]+sel['paths'][:2];inherit=prefix==raw['inherited_prefix_paths'] and sum(len(p['samples']) for p in prefix)==raw['inherited_native_sample_scope']==92 and raw['inherited_native_groups_repeated']==0
    for p in prefix:
        owner=prior if p['id'] in {z['id'] for z in prior['paths'][:10]} else sel
        inherit=inherit and p['completed'] and all(not owner['native'][s['id']]['overlaps'] and not owner['native'][s['id']]['operation_failures'] for s in p['samples'])
    inherit=inherit and all(n.m.drive.sha(ROOT/f)==h for f,h in prior['source_sha256'].items()) and all(n.m.drive.sha(ROOT/f)==h for f,h in sel['source_sha256'].items())
    c['same78_electrical_and14_barrelnuts_exact92_prefix_no_repeat']=bool(inherit)
    kit=set(raw['kit7_names']);expected={'thumb_ip_barrel_adjuster_0','arm059_straight_jaw_0','motor_housing_clamp_0_jaw_1',*[f'motor_housing_clamp_0_{k}_{i}' for k in ['nut','bolt'] for i in [0,1]]}
    c['whole_original_kit7_and_two_jamnuts_not_generic_template']=kit==expected and len(kit)==7 and {'thumb_ip_barrel_top_nut_0','thumb_ip_barrel_bottom_nut_0'}<=set(raw['initial_carrier_bench_present_names']) and not kit&set(raw['initial_carrier_bench_present_names'])
    bench=set(raw['initial_kit_bench_present_names']);carrier=set(raw['initial_carrier_bench_present_names']);seq=bench=={'thumb_ip_barrel_adjuster_0'} and carrier==set(sel['paths'][2]['present_names']) and len(carrier)==27;outside=True;failed=False;matrix_error=0.;identity_error=0.
    for p in raw['paths']:
        present=bench if p['stage']=='kit_bench' else carrier
        seq=seq and not failed and present==set(p['present_names']) and not present&set(p['moving_names']) and set(p['absent_names'])==set(lookup)-present
        if p['stage']=='carrier_bench':seq=seq and bench==kit and set(p['moving_names'])==kit and np.max(abs(np.array(p['approach_axis'])-np.array(raw['guide_opening_axis'])))<1e-10
        for s in p['samples']:matrix_error=max(matrix_error,float(np.max(abs(m.translate(s['translation_mm'])-np.array(s['transform'])))))
        axis=np.array(p['approach_axis']);moving=m.moved([lookup[k] for k in p['moving_names']],np.array(p['samples'][0]['transform']));mb=m.project_bounds(moving,axis);fb=m.project_bounds([lookup[k] for k in present],axis);gap=max(fb[0]-mb[1],mb[0]-fb[1]);outside=outside and gap>0 and abs(gap-p['initial_projection_separation_mm'])<1e-5
        if p['completed']:present.update(p['moving_names']);identity_error=max(identity_error,float(np.max(abs(np.array(p['samples'][-1]['transform'])-np.eye(4)))))
        else:failed=True;seq=seq and p['samples'][-1]['id']==raw.get('first_blocked_sample') and bool(raw['native'][p['samples'][-1]['id']]['overlaps'] or raw['native'][p['samples'][-1]['id']]['operation_failures'])
    c['bench_sequence_explicit_absence_stop_first_block_and_finalidentity']=bool(seq and matrix_error<1e-10 and identity_error<1e-10 and bench==set(raw['final_kit_bench_present_names']) and carrier==set(raw['final_carrier_bench_present_names']))
    c['all_whole_stage_obstacles_outside_entry_starts']=bool(outside)
    c['world670_pending216_no_new_world_restoration']=raw['initial_world_present_names']==raw['packet_final_present_names']==prior['initial_world_present_names'] and len(raw['packet_final_pending_names'])==216 and set(raw['packet_final_present_names'])|set(raw['packet_final_pending_names'])==set(lookup) and not raw['new_restored_modules'] and not raw['special48_restoration_completed']
    old59=n.p.read(ROOT/'cad/integrated_motor_end_v1/reviews/check.json');Q=np.array(old59['design']['ports'][0]['Q']);property_ok=np.max(abs(Q-np.array(survey['port0_frame'])))<1e-10;count=0
    for z in survey['actual_capture_parts']:
        sh=n.g.original.transform(lookup[z['name']]['shape'],np.linalg.inv(Q))
        for q in z['capture_frame_faces']:
            f=sh.Faces()[q['index']];a=BRepAdaptor_Surface(f.wrapped);count+=1;property_ok=property_ok and f.geomType()==q['type'] and np.max(abs(np.array(n.c.m.pack._bbox(f))-np.array(q['bbox'])))<1e-5
            if q['type']=='CYLINDER':
                cy=a.Cylinder();property_ok=property_ok and abs(cy.Radius()-q['radius_mm'])<1e-5 and np.max(abs(np.array(cy.Axis().Direction().Coord())-q['axis']))<1e-5 and np.max(abs(np.array(cy.Axis().Location().Coord())-q['point']))<1e-5
            else:property_ok=property_ok and abs(f.Area()-q['area_mm2'])<1e-5 and np.max(abs(np.array(a.Plane().Location().Coord())-q['point']))<1e-5
    c['actual_new_capture_bore_lip_planes_in_original_port_frame']=bool(property_ok)
    old72=n.p.read(ROOT/'cad/special_thumb_module_service_v1/reviews/continuation_common_prepared.json');stored=n.c.load(n.p.read(ROOT/old72['common_scene'])['parts'])[0]['shape']
    c['saved_ARM072_common_property_read_no_old_Boolean']=abs(stored.Volume()-survey['stored_old_common_volume_mm3'])<1e-5 and np.max(abs(np.array(n.c.m.pack._bbox(n.g.original.transform(stored,np.linalg.inv(Q))))-np.array(survey['stored_old_common_bbox_in_capture_frame'])))<1e-5 and survey['old_common_Boolean_repeated']==0
    conflicts=[z for z in old59['native']['new4__moved16']['overlaps'] if z['lead']=='arm059_integrated_electrical_R5_base' and z['part'] in ['thumb_ip_barrel_adjuster_1','thumb_ip_barrel_top_nut_1','thumb_ip_barrel_bottom_nut_1']]
    c['existing_three_ARM059_final_port1_conflicts_preserved_no_retest']=conflicts==survey['existing_ARM059_port1_final_conflicts']==raw['existing_ARM059_port1_final_conflicts'] and len(conflicts)==3 and raw['port1_existing_conflicts_retested']==0 and raw['port1_attempt_deferred_due_to_existing_final_geometry_conflicts']
    continuity_now,_=continuity.records();want=n.p.read(HERE/'reviews/conflict_continuity.json')['records'];same=len(want)==len(continuity_now)==4
    for a,b in zip(continuity_now,want):same=same and a['matched'] and a['original_ARM059_step']==b['original_ARM059_step'] and a['sha256_step']==b['sha256_step'] and np.max(abs(a['original_ARM059_transform']-np.array(b['original_ARM059_transform'])))<1e-10 and np.max(abs(a['current_transform']-np.array(b['current_transform'])))<1e-10
    c['same_four_STEP_sources_and_registered_poses_for_old_port1_pairs']=bool(same)
    saved=n.c.load(n.p.read(ROOT/ev['common_scene'])['parts']) if ev['common_scene'] else []
    for q,z in zip(ev['common_queries'],saved):
        sh=n.g.original.transform(lookup[q['lead']]['shape'],np.array(q['lead_transform'])).intersect(lookup[q['part']]['shape'],tol=1e-6);ok=sh.isValid() and len(sh.Solids())==q['solids'] and abs(sh.Volume()-q['volume_mm3'])<1e-5
        result['common_samples'].append(dict(name=q['name'],matched=bool(ok),volume_mm3=sh.Volume(),solids=len(sh.Solids())));g=dict(name=q['name'],**metrics.geometry_match(sh,z['shape']));result['geometry_comparisons'].append(g)
        if not ok:result['differences'].append('common_'+q['name'])
        if not g['matched']:result['differences'].append('geometry_'+q['name'])
    for k,v in c.items():
        if not v:result['differences'].append(k)
    result.update(completed=True,native_groups=len(result['native_samples']),native_matches=sum(z['matched'] for z in result['native_samples']),geometry_shapes=len(result['geometry_comparisons']),geometry_matches=sum(z['matched'] for z in result['geometry_comparisons']),common_matches=sum(z['matched'] for z in result['common_samples']),new_capture_faces_compared=count,all_translation_matrix_max_error=matrix_error,completed_final_identity_max_error=identity_error,input_hash_guard=guard(),checkout_clean=not git('status','--porcelain=v1'),
        replay_scope='New original kit bench endpoints and wholekit7 carrier first+last3,or all reached first failure samples. Same92 prefix proof only,no oldnative repeat. Original16packet408/886/start670/rear8/special48/reference12 inherited limits,actual capture lip/bore/planes22 in portframe,old savedcommon property read only,explicit omissions/wholeoutside starts/finalidentity. Four sameSTEP/registeredpose continuities preserve existingARM059 port1 conflicts without repeated31native/common. Newcommon/signature only ifsaved. Not all middle samples/trimBoolean/pixels/armvisualunits/continuousfactory/transport/tool/captive/mating/harnessflex/force/tension/torque/physical20drive/fullarm/exterior.')
    result['initial_core_passed']=not result['differences'] and result['input_hash_guard'] and result['checkout_clean'];save();print('core',result['initial_core_passed'],result['differences'],flush=True)


if __name__=='__main__':main()
