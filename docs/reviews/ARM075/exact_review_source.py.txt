"""Bounded exact-SHA replay of mapped special48 assembly with explicit omissions."""
import argparse,importlib.util,subprocess,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm075_review',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,d,ROOT,HERE=m.n,m.np,m.d,m.ROOT,m.HERE
metrics=n.m.drive.module('_arm075_metrics',ROOT/'cad/recessed_mounting_root_v1/review_core.py')
from OCP.OSD import OSD_ThreadPool


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1') and not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
    man=n.p.read(ROOT/'checkpoints/ARM075.manifest.json');frozen={}
    for k in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(n.p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[k])
    def guard():return all(n.m.drive.sha(ROOT/f)==z['sha256'] for f,z in man['files'].items()) and all(n.m.drive.sha(ROOT/f)==h for f,h in frozen.items())
    assert guard() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    rows,_,source,packets=m.context();lookup={z['name']:z for z in rows};q=n.p.read(HERE/'reviews/check.json');selected=n.p.read(HERE/'reviews/selected_check.json');ev=n.p.read(HERE/'reviews/evidence.json');ev2=n.p.read(HERE/'reviews/evidence_selected.json');survey=n.p.read(HERE/'reviews/survey.json')
    result=dict(completed=False,target_commit=args.commit,differences=[],native_samples=[],common_samples=[],geometry_comparisons=[],contracts={},manifest_files=len(man['files']),frozen_files=len(frozen),OCC_pool_threads=1,same_agent_not_independent_audit=True)
    def save():n.a.save(args.output,result)
    for report,owner in [('initial',q),('selected',selected)]:
        for p in owner['paths']:
            for s in m.replay_samples(p):
                got=n.c.material(m.moved([lookup[k] for k in p['moving_names']],np.array(s['transform'])),[lookup[k] for k in p['present_names']]);want=owner['native'][s['id']];a,b=d.overlap_map(got),d.overlap_map(want);error=max([abs(a[k]['material_intersection_mm3']-b[k]['material_intersection_mm3']) for k in a.keys()&b.keys()]+[0.]);ok=a.keys()==b.keys() and error<1e-5 and got['pairs_in_scope']==want['pairs_in_scope'] and got['operation_failures']==want['operation_failures']
                result['native_samples'].append(dict(id=s['id'],report=report,native=got,matched=bool(ok),max_volume_difference_mm3=error))
                if not ok:result['differences'].append(report+'_'+s['id'])
                save();print(report,s['id'],bool(ok),flush=True)
    c=result['contracts'];old=n.p.read(ROOT/'cad/special_thumb_module_service_v1/reviews/check.json');old73=n.p.read(ROOT/'cad/captured_adjuster_kit_v1/reviews/check.json');first74=n.p.read(ROOT/'cad/branched_terminal_support_v1/reviews/check.json')
    c['original16packet408_and_explicit_one_support_replacement']=packets==q['original16packets'] and sum(len(p['names']) for p in packets)==408 and q['explicit_replacement']=={m.OLD:m.NAME} and sorted(lookup)==q['final886_names'] and m.OLD not in lookup and m.NAME in lookup
    original=old['original_special_packet'];mapped={k:(m.mapped(v) if k in ['names','body_names','mounting_names'] else v) for k,v in original.items()}
    c['special48_body44_rack4_original47_identity_mapping']=q['original_special_packet']==original and q['mapped_special_packet']==mapped and len(mapped['names'])==48 and len(mapped['body_names'])==44 and set(q['body43_names'])==set(mapped['body_names'])-{'thumb_ip_actuator_base_nuts'}
    c['same_independent_motor14_port0kit42_exactproof_no_changed_support']=m.inheritance()==q['independent_inherited_proofs']==survey['independent_inherited_proofs'] and q['inherited_sample_scope']==56 and q['inherited_native_groups_repeated']==0 and not q['old_full49_or92_inherited_to_changed_support']
    c['reference12_original_stock_and_custom_parts_not_generic_template']=q['unit_reference12_names']==old['unit_reference12_names'] and len(q['unit_reference12_names'])==12 and q['unit14_names']==old['unit14_names'] and len(q['unit14_names'])==14 and not q['internal_reference12_retested']
    carrier=set(q['initial_carrier_bench_present_names']);kit1=set(q['initial_kit1_present_names']);world=set(q['initial_world_present_names']);seq=carrier=={'thumb_ip_actuator_adapter'} and kit1=={'thumb_ip_barrel_adjuster_1'} and world==set(old73['packet_final_present_names']);outside=True;failed=False;matrix_error=0.;identity_error=0.
    for p in q['paths']:
        present=kit1 if p['stage']=='kit1_bench' else carrier if p['stage']=='carrier_bench' else world
        seq=seq and not failed and present==set(p['present_names']) and not present&set(p['moving_names']) and set(p['absent_names'])==set(lookup)-present
        if p['stage']=='world' and len(p['moving_names'])==43:seq=seq and carrier==set(q['body43_names']) and np.max(abs(np.array(p['approach_axis'])-np.array(q['radial'])))<1e-10
        for s in p['samples']:matrix_error=max(matrix_error,float(np.max(abs(m.translate(s['translation_mm'])-np.array(s['transform'])))))
        axis=np.array(p['approach_axis']);moving=m.moved([lookup[k] for k in p['moving_names']],np.array(p['samples'][0]['transform']));mb=m.project_bounds(moving,axis);fb=m.project_bounds([lookup[k] for k in present],axis);gap=max(fb[0]-mb[1],mb[0]-fb[1]);outside=outside and gap>0 and abs(gap-p['initial_projection_separation_mm'])<1e-5
        if p['completed']:present.update(p['moving_names']);identity_error=max(identity_error,float(np.max(abs(np.array(p['samples'][-1]['transform'])-np.eye(4)))))
        else:failed=True;seq=seq and p['samples'][-1]['id']==q.get('first_blocked_sample') and bool(q['native'][p['samples'][-1]['id']]['overlaps'] or q['native'][p['samples'][-1]['id']]['operation_failures'])
    c['new_benches_world_sequence_omissions_firstblock_finalidentity']=bool(seq and matrix_error<1e-10 and identity_error<1e-10 and sorted(carrier)==q['final_carrier_bench_present_names'] and sorted(kit1)==q['final_kit1_present_names'] and sorted(world)==q['packet_final_present_names'])
    c['all_stage_full_obstacle_AABB_outside_starts']=bool(outside)
    c['rear8_world670_retained_exactfinal_and_pending_inventory']=set(old73['packet_final_present_names'])<=world and len(q['restored_before'])==8 and set(q['packet_final_pending_names'])==set(lookup)-world and not world&set(q['packet_final_pending_names']) and len(world)+len(q['packet_final_pending_names'])==886
    dual_names=set(q['port0_kit7_names'])|set(q['port1_kit7_names'])|{f'thumb_ip_barrel_{tag}_nut_{i}' for tag in ['top','bottom'] for i in [0,1]}
    c['dual_original_kit7_and_jamnuts_present_when_completed']=not q['dualport_carrier_bench_completed'] or (len(carrier)==43 and dual_names<=carrier)
    c['mapped48_restoration_claim_matches_actual_world_parts']=q['mapped_special48_restoration_completed']==(set(mapped['names'])<=world) and q['restored_modules']==q['restored_before']+q['new_restored_modules']
    c['M3_upper_inclusionfalse_and216point_hold_readonly']=q['preserved_upper_M3_whole_cut']==first74['attachment_queries'][-1] and q['old_inclusion_Boolean_or_points_repeated']==0 and survey['preserved_M3_point_count']==216 and survey['preserved_all_points_inside_both']
    prefix=q['paths'][:-1];c['same_new161_successful_prefix_retained_no_native_repeat']=prefix==selected['inherited_new_successful_prefix_paths'] and sum(len(p['samples']) for p in prefix)==selected['inherited_new_prefix_samples']==161 and selected['inherited_new_prefix_native_repeated']==0 and all(p['completed'] for p in prefix) and selected['selected_world_start_present_names']==q['packet_final_present_names']
    sworld=set(selected['selected_world_start_present_names']);sseq=len(sworld)==713;sfail=False;soutside=True
    for p in selected['paths']:
        sseq=sseq and not sfail and sworld==set(p['present_names']) and not sworld&set(p['moving_names']) and set(p['absent_names'])==set(lookup)-sworld
        axis=np.array(p['approach_axis']);moving=m.moved([lookup[k] for k in p['moving_names']],np.array(p['samples'][0]['transform']));mb=m.project_bounds(moving,axis);fb=m.project_bounds([lookup[k] for k in sworld],axis);gap=max(fb[0]-mb[1],mb[0]-fb[1]);soutside=soutside and gap>0 and abs(gap-p['initial_projection_separation_mm'])<1e-5
        for s in p['samples']:matrix_error=max(matrix_error,float(np.max(abs(m.translate(s['translation_mm'])-np.array(s['transform'])))))
        if p['completed']:sworld.update(p['moving_names']);identity_error=max(identity_error,float(np.max(abs(np.array(p['samples'][-1]['transform'])-np.eye(4)))))
        else:sfail=True;sseq=sseq and p['samples'][-1]['id']==selected['first_blocked_sample'] and bool(selected['native'][p['samples'][-1]['id']]['overlaps'] or selected['native'][p['samples'][-1]['id']]['operation_failures'])
    c['selected_world_sequence_fulloutside_firstblock_identity_and_inventory']=bool(sseq and soutside and matrix_error<1e-10 and identity_error<1e-10 and sorted(sworld)==selected['packet_final_present_names'] and set(selected['packet_final_pending_names'])==set(lookup)-sworld and selected['mapped_special48_restoration_completed']==(set(mapped['names'])<=sworld))
    c['selected_rack_nuts_at_original_final_axial_datum_fulloutside']=all(abs(np.dot(np.array(s['translation_mm']),np.array(q['normal'])))<1e-10 for p in selected['paths'][:2] for s in p['samples']) and all('final_axial' in p['id'] for p in selected['paths'][:2])
    railok=True
    if 'base_nuts_actual_support_crossing_faces' in selected:
        from OCP.BRepAdaptor import BRepAdaptor_Surface
        for z in selected['base_nuts_actual_support_crossing_faces']:
            f=lookup[m.NAME]['shape'].Faces()[z['index']];pl=BRepAdaptor_Surface(f.wrapped).Plane();railok=railok and f.geomType()=='PLANE' and np.max(abs(np.array(pl.Location().Coord())-np.array(z['point'])))<1e-5 and np.max(abs(np.array(n.c.m.pack._bbox(f))-np.array(z['bbox'])))<1e-5
        nutbb=m.bbox([lookup['thumb_ip_actuator_base_nuts']]);levels=sorted({round(float(z['point'][0]-(nutbb[0,0]+nutbb[1,0])/2.),9) for z in selected['base_nuts_actual_support_crossing_faces']});events=levels+[(x+y)/2. for x,y in zip(levels,levels[1:])]
        railok=railok and np.max(abs(np.array(events)-np.array(selected['base_nuts_extra_axial_projection_samples_mm'])))<1e-5
    c['actual_thin_support_rail_event_properties_and_sample_contract']=bool(railok)
    commons=[(want,z) for evidence in [ev,ev2] for want,z in zip(evidence['common_queries'],n.c.load(n.p.read(ROOT/evidence['common_scene'])['parts']) if evidence['common_scene'] else [])]
    for want,z in commons:
        sh=n.g.original.transform(lookup[want['lead']]['shape'],np.array(want['lead_transform'])).intersect(lookup[want['part']]['shape'],tol=1e-6);ok=sh.isValid() and len(sh.Solids())==want['solids'] and abs(sh.Volume()-want['volume_mm3'])<1e-5;result['common_samples'].append(dict(name=want['name'],matched=bool(ok),volume_mm3=sh.Volume(),solids=len(sh.Solids())))
        g=dict(name=want['name'],**metrics.geometry_match(sh,z['shape']));result['geometry_comparisons'].append(g)
        if not ok:result['differences'].append('common_'+want['name'])
        if not g['matched']:result['differences'].append('geometry_'+want['name'])
    for k,v in c.items():
        if not v:result['differences'].append(k)
    result.update(completed=True,native_groups=len(result['native_samples']),native_matches=sum(z['matched'] for z in result['native_samples']),geometry_shapes=len(result['geometry_comparisons']),geometry_matches=sum(z['matched'] for z in result['geometry_comparisons']),common_matches=sum(z['matched'] for z in result['common_samples']),all_translation_matrix_max_error=matrix_error,completed_final_identity_max_error=identity_error,input_hash_guard=guard(),checkout_clean=not git('status','--porcelain=v1'),
        replay_scope='New reached bench path endpoints/wholekit carrier first+last3/world paths first+last5 or all reached firstfailedpath samples;all4interface/216points/ARM07415relations and615PCD not repeated. Exact independent motor14/port0kit42 proof only,new0;actualport1kit7,newbranchedbase/electrical/jamnuts/wholekitentries/worldmapped48 and originalM3/base_nuts only reached. Explicit1replacement/original47/16packet408/886/rear8world670start,allomissions/fulloutside/finalidentity/firstblock. Newcommon/signature only ifgenerated. Not every middle/continuousfactory/transport/tools/captive/mating/harnessflex/strength/friction/preload/tension/electrical/20drive/fullarm/exterior or alltrimBoolean/pixels/armvisualunits.')
    result['initial_core_passed']=not result['differences'] and result['input_hash_guard'] and result['checkout_clean'];save();print('core',result['initial_core_passed'],result['differences'],flush=True)


if __name__=='__main__':main()
