"""Bounded exact-commit replay and independent inventory/event contracts."""
import argparse,importlib.util,subprocess,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm078_review',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,d,ROOT,HERE=m.n,m.np,m.d,m.ROOT,m.HERE
metrics=n.m.drive.module('_arm078_metrics',ROOT/'cad/recessed_mounting_root_v1/review_core.py')
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.OSD import OSD_ThreadPool


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1') and not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
    man=n.p.read(ROOT/'checkpoints/ARM078.manifest.json');frozen={}
    for k in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(n.p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[k])
    def guard():return all(n.m.drive.sha(ROOT/f)==z['sha256'] for f,z in man['files'].items()) and all(n.m.drive.sha(ROOT/f)==h for f,h in frozen.items())
    assert guard() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    rows,_,source,packets=m.context();lookup={z['name']:z for z in rows};canonical=set(lookup);q=n.p.read(HERE/'reviews/check.json');survey=n.p.read(HERE/'reviews/survey.json');ev=n.p.read(HERE/'reviews/evidence.json');prior=n.p.read(ROOT/'cad/individual_base_nut_assembly_v1/reviews/check.json')
    result=dict(completed=False,target_commit=args.commit,differences=[],native_samples=[],common_samples=[],geometry_comparisons=[],contracts={},manifest_files=len(man['files']),frozen_files=len(frozen),OCC_pool_threads=1,same_agent_not_independent_audit=True)
    def save():n.a.save(args.output,result)
    for p in q['paths']:
        for s in m.replay_samples(p):
            got=n.c.material(m.moved([lookup[k] for k in p['moving_names']],np.array(s['transform'])),[lookup[k] for k in p['present_names']]);want=q['native'][s['id']];a,b=d.overlap_map(got),d.overlap_map(want);error=max([abs(a[k]['material_intersection_mm3']-b[k]['material_intersection_mm3']) for k in a.keys()&b.keys()]+[0.]);ok=a.keys()==b.keys() and error<1e-5 and got['pairs_in_scope']==want['pairs_in_scope'] and got['operation_failures']==want['operation_failures'];result['native_samples'].append(dict(id=s['id'],native=got,matched=bool(ok),max_volume_difference_mm3=error))
            if not ok:result['differences'].append(s['id'])
            save();print(s['id'],bool(ok),flush=True)
    c=result['contracts'];plans=[p for p in packets if p['module'] not in prior['restored_modules']]
    c['original16packet408_front7x24_886_one_support_replacement']=packets==q['original16packets'] and sum(len(z['names']) for z in packets)==408 and len(plans)==7 and all(z['row']==1 and len(z['names'])==24 for z in plans) and [z['module'] for z in plans]==q['planned_front_modules'] and len(canonical)==886 and sorted(canonical)==q['final886_names'] and q['explicit_replacement']=={m.OLD:m.NAME} and m.OLD not in canonical
    world=set(q['initial_world718_names']);c['same718_start_special48_rear8_hand_arm_preserved']=q['initial_world718_names']==prior['packet_final_present_names'] and len(world)==718 and set(q['mapped_special_packet']['names'])<=world and q['mapped_special_packet']==prior['mapped_special_packet'] and q['restored_before']==prior['restored_modules']
    boxes={k:m.corners(n.c.m.pack._bbox(z['shape'])) for k,z in lookup.items()};sequence=True;outside=True;eventok=True;identity=True;failed=False;maxmatrix=0.;benchstates={};carrierstates={};newdone=[]
    for p in q['paths']:
        name=p['module'];names=set(p['moving_names'])
        if p['stage']=='bench':present=benchstates.setdefault(name,{name+'_cradle'})
        elif p['stage']=='carrier_bench':present=carrierstates.setdefault(name,{name+'_adapter'})
        else:present=world
        sequence=sequence and not failed and set(p['present_names'])==present and not names&present and set(p['absent_names'])==canonical-present
        moving=m.moved([lookup[k] for k in names],np.array(p['samples'][0]['transform']));axis=np.array(p['approach_axis']);mb=m.project_bounds(moving,axis);fb=m.project_bounds([lookup[k] for k in present],axis);gap=max(fb[0]-mb[1],mb[0]-fb[1]);outside=outside and gap>0 and abs(gap-p['initial_projection_separation_mm'])<1e-5
        expected,events=m.enrich_vectors(p['requested_vectors'],p['moving_names'],sorted(present),boxes);eventok=eventok and len(expected)==p['planned_sample_count'] and len(events)==len(p['projected_obstacle_events'])
        for s,w in zip(p['samples'],expected):
            maxmatrix=max(maxmatrix,float(np.max(abs(m.translate(s['translation_mm'])-np.array(s['transform'])))));eventok=eventok and np.max(abs(np.array(s['translation_mm'])-np.array(w['translation_mm'])))<1e-7 and len(s['events'])==len(w['events'])
        if p['completed']:
            present.update(names);identity=identity and np.max(abs(np.array(p['samples'][-1]['transform'])-np.eye(4)))<1e-10 and len(p['samples'])==p['planned_sample_count']
        else:failed=True;sequence=sequence and p['samples'][-1]['id']==q.get('first_blocked_sample') and bool(q['native'][p['samples'][-1]['id']]['overlaps'] or q['native'][p['samples'][-1]['id']]['operation_failures'])
    placements={z['module']:z for z in n.p.read(ROOT/'cad/eight_bank_layout_v1/reviews/check.json')['placements']};guides={z['module']:z for z in n.p.read(ROOT/'cad/wire_guides_v1/generated/scene.json')['groups']};frames=True
    for f in survey['module_frames']:
        pl=placements[f['module']];P=np.array(pl['proposed_installation_transform']);delta=np.array(pl['transform_from_current']);frames=frames and np.max(abs(np.array(f['placement'])-P))<1e-10 and np.max(abs(np.array(f['delta'])-delta))<1e-10 and np.max(abs(np.array(f['radial'])+P[:3,2]))<1e-10 and np.max(abs(np.array(f['normal'])-P[:3,0]))<1e-10 and np.max(abs(np.array(f['opening'])-delta[:3,:3]@np.array([0.,0.,1.])))<1e-10 and np.max(abs(np.array(f['guide_frame'])-delta@np.array(guides[f['module']]['frame_world_mm'])))<1e-10
    for mc in q['module_contracts']:
        name=mc['module'];packet=next(z for z in plans if z['module']==name);body=set(packet['body_names'])-{name+'_base_nuts'};sequence=sequence and mc['original24_names']==packet['names'] and set(mc['body19_names'])==body and len(body)==19 and not mc['bench_inherited']
        if mc.get('body19_bench_completed'):sequence=sequence and carrierstates[name]==body
        if mc['completed']:newdone.append(name);sequence=sequence and set(packet['names'])<=world
    c['actual_frames_and_installation_bases_unchanged']=bool(frames)
    c['fresh_bench_world_sequence_no_duplicates_omissions_firstblock']=bool(sequence and maxmatrix<1e-10)
    c['each_path_whole_obstacle_AABB_outside_start']=bool(outside)
    c['projected_actual_part_AABB_event_prefix_reproduced']=bool(eventok)
    c['completed_paths_original_final_identity']=bool(identity)
    c['canonical_cumulative_world_pending_and_restoration_claims']=sorted(world)==q['packet_final_present_names'] and set(q['packet_final_pending_names'])==canonical-world and newdone==q['new_restored_modules'] and q['restored_modules']==prior['restored_modules']+newdone and q['all16_packet_restoration_completed']==(world==canonical)
    c['special48_and_previous718_never_removed']=set(prior['packet_final_present_names'])<=world and set(prior['mapped_special_packet']['names'])<=world
    propmax=0.
    for z in survey['actual_axes']:
        for f in z['cylinders']:
            cy=BRepAdaptor_Surface(lookup[z['name']]['shape'].Faces()[f['face']].wrapped).Cylinder();propmax=max(propmax,abs(cy.Radius()-f['radius_mm']),float(np.max(abs(np.array(cy.Location().Coord())-np.array(f['point'])))),float(np.max(abs(np.array(cy.Axis().Direction().Coord())-np.array(f['axis'])))))
    c['actual_front7_M2_M3_cylinder_properties']=len(survey['actual_axes'])==14 and propmax<1e-5
    c['no_old_native_or_geometry_change_current862_adoption']=q['old_native_repeated']==0 and q['new_hardware_geometry']==0 and not q['current862_replaced'] and not q['proposal886_adopted'] and q['preserved_upper_M3_whole_cut']==prior['preserved_upper_M3_whole_cut']
    for want,z in zip(ev['common_queries'],n.c.load(n.p.read(ROOT/ev['common_scene'])['parts']) if ev['common_scene'] else []):
        sh=n.g.original.transform(lookup[want['lead']]['shape'],np.array(want['lead_transform'])).intersect(lookup[want['part']]['shape'],tol=1e-6);ok=sh.isValid() and len(sh.Solids())==want['solids'] and abs(sh.Volume()-want['volume_mm3'])<1e-5;result['common_samples'].append(dict(name=want['name'],matched=bool(ok),volume_mm3=sh.Volume(),solids=len(sh.Solids())));g=dict(name=want['name'],**metrics.geometry_match(sh,z['shape']));result['geometry_comparisons'].append(g)
        if not ok:result['differences'].append('common_'+want['name'])
        if not g['matched']:result['differences'].append('geometry_'+want['name'])
    for k,v in c.items():
        if not v:result['differences'].append(k)
    result.update(completed=True,native_groups=len(result['native_samples']),native_matches=sum(z['matched'] for z in result['native_samples']),geometry_shapes=len(result['geometry_comparisons']),geometry_matches=sum(z['matched'] for z in result['geometry_comparisons']),common_matches=sum(z['matched'] for z in result['common_samples']),contracts_passed=sum(c.values()),actual_property_max_difference=propmax,all_translation_matrix_max_error=maxmatrix,input_hash_guard=guard(),checkout_clean=not git('status','--porcelain=v1'),replay_scope='Each new path first/last;world last5;all reached failedpath samples. Fourteen actualfront7 M2/M3cylinder reads,actualframes/projectedAABB event prefix/inventories/omissions/wholeoutside/identity/initial718+special48 retained/16packet408+support1/886/current862unadopted. Only newcommon typed signatures epsilon1e-10 abs1e-5. Not allmiddle/native/continuousswept/benchtransport/manualholding/tools/threads/torque/outputinternal/mating/flexibleharness/captive/strength/force/friction/preload/tension/20drive/fullarm/exterior/trimBoolean/pixels/armvisualunits. Old68/106/162/216M3/615PCD/precision/pixelrecovery not repeated.')
    result['initial_core_passed']=not result['differences'] and result['input_hash_guard'] and result['checkout_clean'];save();print('core',result['initial_core_passed'],result['differences'],flush=True)


if __name__=='__main__':main()
