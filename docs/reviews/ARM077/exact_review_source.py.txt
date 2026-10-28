"""Exact-SHA replay of original-solid extraction, independent paths and compaction."""
import argparse,importlib.util,subprocess,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm077_review',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,d,ROOT,HERE=m.n,m.np,m.d,m.ROOT,m.HERE
metrics=n.m.drive.module('_arm077_metrics',ROOT/'cad/recessed_mounting_root_v1/review_core.py')
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.OSD import OSD_ThreadPool


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1') and not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
    man=n.p.read(ROOT/'checkpoints/ARM077.manifest.json');frozen={}
    for k in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(n.p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[k])
    def guard():return all(n.m.drive.sha(ROOT/f)==z['sha256'] for f,z in man['files'].items()) and all(n.m.drive.sha(ROOT/f)==h for f,h in frozen.items())
    assert guard() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    rows,_,source,packets=m.context();lookup={z['name']:z for z in rows};canonical=set(lookup);parts=m.split_nuts(lookup);lookup.update({z['name']:z for z in parts});survey=n.p.read(HERE/'reviews/survey.json');prior=n.p.read(ROOT/'cad/preloaded_rack_nut_module_v1/reviews/selected_check.json')
    result=dict(completed=False,target_commit=args.commit,differences=[],native_samples=[],common_samples=[],geometry_comparisons=[],contracts={},manifest_files=len(man['files']),frozen_files=len(frozen),OCC_pool_threads=1,same_agent_not_independent_audit=True)
    def save():n.a.save(args.output,result)
    labels=['check']+(['selected_check'] if (HERE/'reviews/selected_check.json').exists() else [])
    maxmatrix=0.;maxidentity=0.;maxprop=0.;allsolidnames={z['name'] for z in parts}
    for label in labels:
        q=n.p.read(HERE/('reviews/'+label+'.json'));ev=n.p.read(HERE/('reviews/evidence'+('_selected' if label=='selected_check' else '')+'.json'));c={}
        for p in q['paths']:
            for s in m.replay_samples(p):
                got=n.c.material(m.moved([lookup[k] for k in p['moving_names']],np.array(s['transform'])),[lookup[k] for k in p['native_present_names']]);want=q['native'][s['id']];a,b=d.overlap_map(got),d.overlap_map(want);error=max([abs(a[k]['material_intersection_mm3']-b[k]['material_intersection_mm3']) for k in a.keys()&b.keys()]+[0.]);ok=a.keys()==b.keys() and error<1e-5 and got['pairs_in_scope']==want['pairs_in_scope'] and got['operation_failures']==want['operation_failures'];result['native_samples'].append(dict(id=s['id'],report=label,native=got,matched=bool(ok),max_volume_difference_mm3=error))
                if not ok:result['differences'].append(label+'_'+s['id'])
                save();print(label,s['id'],bool(ok),flush=True)
        c['original16packet408_special48_one_replacement_886']=packets==q['original16packets']==prior['original16packets'] and sum(len(z['names']) for z in packets)==408 and q['mapped_special_packet']==prior['mapped_special_packet'] and q['original_special_packet']==prior['original_special_packet'] and q['explicit_replacement']=={m.OLD:m.NAME} and sorted(canonical)==q['final886_names'] and len(canonical)==886 and m.OLD not in canonical
        c['original_four_solid_indices_and_geometry_unmodified']=q['original_nut_solid_count']==4 and [z['source_solid_index'] for z in q['nut_solids']]==[0,1,2,3] and all(z['shape'].wrapped.IsSame(lookup[m.BASE]['shape'].Solids()[z['source_solid_index']].wrapped) for z in parts)
        c['same717_start_all_rear8_hand_arm_preserved']=q['initial_world717_names']==q['preserved_previous_world717_names']==prior['packet_final_present_names'] and len(q['initial_world717_names'])==717 and m.BASE not in q['initial_world717_names'] and q['restored_before']==prior['restored_modules']
        world=set(q['initial_world717_names']);placed=set();seq=True;outside=True;failed=False;eventok=True
        for p in q['paths']:
            names=set(p['moving_names']);present=world|placed;seq=seq and not failed and len(names)==1 and names<=allsolidnames and not names&present and set(p['native_present_names'])==present and set(p['canonical_present_names'])==world and set(p['canonical_absent_names'])==canonical-world and set(p['completed_original_solid_names'])==placed and set(p['absent_original_solid_names'])==allsolidnames-placed and m.BASE not in present and p['original_compound_absent_from_fixed']
            axis=np.array(p['approach_axis']);moving=m.moved([lookup[k] for k in names],np.array(p['samples'][0]['transform']));mb=m.project_bounds(moving,axis);fb=m.project_bounds([lookup[k] for k in present],axis);gap=max(fb[0]-mb[1],mb[0]-fb[1]);outside=outside and gap>0 and abs(gap-p['initial_projection_separation_mm'])<1e-5
            for s in p['samples']:maxmatrix=max(maxmatrix,float(np.max(abs(m.translate(s['translation_mm'])-np.array(s['transform'])))))
            if p['completed']:placed.update(names);maxidentity=max(maxidentity,float(np.max(abs(np.array(p['samples'][-1]['transform'])-np.eye(4)))))
            else:failed=True;seq=seq and p['samples'][-1]['id']==q.get('first_blocked_sample') and bool(q['native'][p['samples'][-1]['id']]['overlaps'] or q['native'][p['samples'][-1]['id']]['operation_failures'])
            for z in p['projected_obstacle_events']:
                bb=np.array(n.c.m.pack._bbox(lookup[z['name']]['shape']));eventok=eventok and np.max(abs(bb-np.array(z['bbox'])))<1e-5
        c['independent_sequence_no_compound_duplicates_omissions_firstblock']=bool(seq and sorted(placed)==q['final_completed_solid_names'] and maxmatrix<1e-10 and maxidentity<1e-10)
        c['full_obstacle_outside_start_every_new_path']=bool(outside)
        c['projected_obstacle_event_bbox_properties']=bool(eventok)
        done=len(placed)==4
        if done:world.add(m.BASE)
        c['exact_original_compound_compaction_canonical718_not721']=q['original_base_compound_restored']==done and q['mapped_special48_restoration_completed']==done and sorted(world)==q['packet_final_present_names'] and set(q['packet_final_pending_names'])==canonical-world and (not done or (q['compound_restoration']['source_solid_indices']==[0,1,2,3] and q['compound_restoration']['original_source_compound_reused'] and q['compound_restoration']['partial_virtual_solids_removed_from_native_inventory'] and len(world)==718))
        c['restored_module_claim_and_actual48_membership']=q['mapped_special48_restoration_completed']==(set(q['mapped_special_packet']['names'])<=world) and q['restored_modules']==q['restored_before']+q['new_restored_modules'] and len(q['restored_modules'])==(9 if done else 8)
        c['no_old_native_common_M3point_repeat_no_current862_adoption']=q['old_native_groups_repeated']==q['old_common_Boolean_repeated']==q['old_M3_cut_or216points_repeated']==0 and q['new_hardware_geometry']==0 and not q['current862_replaced'] and not q['proposal886_adopted'] and q['preserved_upper_M3_whole_cut']==prior['preserved_upper_M3_whole_cut']
        for want,z in zip(ev['common_queries'],n.c.load(n.p.read(ROOT/ev['common_scene'])['parts']) if ev['common_scene'] else []):
            sh=n.g.original.transform(lookup[want['lead']]['shape'],np.array(want['lead_transform'])).intersect(lookup[want['part']]['shape'],tol=1e-6);ok=sh.isValid() and len(sh.Solids())==want['solids'] and abs(sh.Volume()-want['volume_mm3'])<1e-5;result['common_samples'].append(dict(name=want['name'],matched=bool(ok),volume_mm3=sh.Volume(),solids=len(sh.Solids())));g=dict(name=want['name'],**metrics.geometry_match(sh,z['shape']));result['geometry_comparisons'].append(g)
            if not ok:result['differences'].append('common_'+want['name'])
            if not g['matched']:result['differences'].append('geometry_'+want['name'])
        result['contracts'].update({label+'_'+k:bool(v) for k,v in c.items()})
    firstev=n.p.read(HERE/'reviews/evidence.json')
    for z in n.c.load(n.p.read(ROOT/firstev['original_solids_scene'])['parts']):
        g=dict(name=z['name'],**metrics.geometry_match(lookup[z['name']]['shape'],z['shape']));result['geometry_comparisons'].append(g)
        if not g['matched']:result['differences'].append('extracted_'+z['name'])
    for name,want in survey['actual_properties'].items():
        for sh,z in [(lookup[name]['shape'],want['whole'])]+[(lookup[name]['shape'].Solids()[v['index']],v) for v in want['solids']]:
            maxprop=max(maxprop,abs(sh.Volume()-z['volume_mm3']),float(np.max(abs(np.array(sh.Center().toTuple())-np.array(z['center_mm'])))),float(np.max(abs(np.array(n.c.m.pack._bbox(sh))-np.array(z['bbox'])))))
            for f in z['faces']:
                face=sh.Faces()[f['index']];ad=BRepAdaptor_Surface(face.wrapped)
                if face.geomType()!=f['type']:result['differences'].append('face_type_'+name+'_'+str(f['index']));continue
                if f['type']=='CYLINDER':
                    cy=ad.Cylinder();maxprop=max(maxprop,abs(cy.Radius()-f['radius_mm']),float(np.max(abs(np.array(cy.Location().Coord())-np.array(f['axis_point'])))),float(np.max(abs(np.array(cy.Axis().Direction().Coord())-np.array(f['axis_direction'])))))
                else:
                    pl=ad.Plane();maxprop=max(maxprop,float(np.max(abs(np.array(pl.Location().Coord())-np.array(f['point'])))),float(np.max(abs(np.array(pl.Axis().Direction().Coord())-np.array(f['normal'])))))
    result['contracts']['actual4nut4screw_adapter_cradle_properties']=maxprop<1e-5 and survey['nut_total_volume_difference_mm3']<1e-5
    for k,v in result['contracts'].items():
        if not v:result['differences'].append(k)
    result.update(completed=True,native_groups=len(result['native_samples']),native_matches=sum(z['matched'] for z in result['native_samples']),geometry_shapes=len(result['geometry_comparisons']),geometry_matches=sum(z['matched'] for z in result['geometry_comparisons']),common_matches=sum(z['matched'] for z in result['common_samples']),contracts_passed=sum(result['contracts'].values()),actual_property_max_difference=maxprop,all_translation_matrix_max_error=maxmatrix,completed_final_identity_max_error=maxidentity,input_hash_guard=guard(),checkout_clean=not git('status','--porcelain=v1'),
        replay_scope='New individual paths start/end/phase boundaries/projected obstacle events and all reached firstfailurepath samples. Original4nut/4screw solids and adapter/cradle property reads,originalsolid extraction source/STEP typed signatures epsilon1e-10/abs1e-5,only newcommons ifgenerated. Actual717+previousnuts/native717..720,sourcecompound absent untilexact4identity compaction to canonical718,original16packet408/48/886/support1 replacement,allomissions/fulloutside/firstblock/parts/restorationclaims. No old106/162/9railfaces/7events/ARM07415/4/M3cut216/615PCD repeat. Not all intermediate/continuousassembly/tools/threads/torque/captive/mating/harnessflex/strength/force/friction/preload/tension/electrical/20drive/fullarm/exterior/alltrimBoolean/pixels/armvisualunits.')
    result['initial_core_passed']=not result['differences'] and result['input_hash_guard'] and result['checkout_clean'];save();print('core',result['initial_core_passed'],result['differences'],flush=True)


if __name__=='__main__':main()
