"""Bounded exact-SHA native replay and original mounting-datum contracts."""
import argparse,importlib.util,subprocess,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm071_review',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,d,ROOT,HERE=m.n,m.np,m.d,m.ROOT,m.HERE
metrics=n.m.drive.module('_arm071_metrics',ROOT/'cad/recessed_mounting_root_v1/review_core.py')
from OCP.OSD import OSD_ThreadPool
from OCP.BRepAdaptor import BRepAdaptor_Surface


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1') and not args.output.exists()
    man=n.p.read(ROOT/'checkpoints/ARM071.manifest.json');frozen={}
    for k in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(n.p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[k])
    def guard():return all(n.m.drive.sha(ROOT/f)==v['sha256'] for f,v in man['files'].items()) and all(n.m.drive.sha(ROOT/f)==h for f,h in frozen.items())
    assert guard() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    rows,_,source=m.context();lookup={z['name']:z for z in rows};raw=n.p.read(HERE/'reviews/check.json');ev=n.p.read(HERE/'reviews/evidence.json');survey=n.p.read(HERE/'reviews/survey.json')
    prior=n.p.read(ROOT/'cad/open_guide_module_assembly_v1/reviews/continuation_check.json')
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
    contracts=result['contracts'];packet=m.packets(rows)
    contracts['original16packet408_final886_start598_and_prior5_retained']=packet==raw['original16packets'] and sum(len(p['names']) for p in packet)==408 and sorted(lookup)==raw['original_final886_names'] and raw['initial_world_present_names']==prior['packet_final_present_names'] and len(raw['restored_before'])==5
    placements={p['module']:p for p in n.p.read(ROOT/'cad/eight_bank_layout_v1/reviews/check.json')['placements']};guides={p['module']:p for p in n.p.read(ROOT/'cad/wire_guides_v1/generated/scene.json')['groups']}
    world=set(raw['initial_world_present_names']);seq=True;basis=True;outside=True;datum=True;inherit=True;matrix_error=0.;identity_error=0.
    for c in raw['module_contracts']:
        name=c['module'];P=np.array(placements[name]['proposed_installation_transform']);delta=np.array(placements[name]['transform_from_current']);r=-P[:3,2];normal=P[:3,0];opening=delta[:3,:3]@np.array([0.,0.,1.]);frame=delta@np.array(guides[name]['frame_world_mm'])
        basis=basis and np.max(abs(P-np.array(c['placement'])))<1e-10 and np.max(abs(frame-np.array(c['guide_actual_frame'])))<1e-10 and np.max(abs(opening-np.array(c['guide_opening_axis'])))<1e-10
        body=set(c['body19_names']);unit=set(c['unit6_names']);carrier=set(c['adapter_names']);bench={name+'_cradle'};seq=seq and set(c['world_before_names'])==world and len(body)==19 and len(c['original24_names'])==24 and c['temporarily_later_base_nuts'] not in body
        if c['bench_inherited']:
            z=c['inherited_bench_contract'];oldc=prior['module_contracts'][0];completed=[p for p in prior['paths'] if p['stage']=='carrier_bench']
            inherit=inherit and name=='ring_pip_actuator' and z['body19_names']==oldc['body19_names']==c['body19_names'] and z['sample_scope']==92 and z['new_native_groups']==0
            inherit=inherit and z['same_bench14']==prior['inherited_bench_paths'] and z['same_guide21']==prior['inherited_open_paths'] and z['same_electrical14']==prior['inherited_electrical_paths'] and z['completed_fastener43']==completed
            inherit=inherit and all(p['completed'] and not prior['native'][s['id']]['overlaps'] and not prior['native'][s['id']]['operation_failures'] for p in completed for s in p['samples'])
            inherit=inherit and all(n.m.drive.sha(ROOT/f)==h for f,h in prior['source_sha256'].items());carrier=body.copy()
        for p in [p for p in raw['paths'] if p['module']==name]:
            present=world if p['stage']=='world' else bench if p['stage']=='bench' else carrier
            seq=seq and set(p['present_names'])==present and not present&set(p['moving_names']) and set(p['absent_names'])==set(lookup)-present
            for s in p['samples']:
                matrix_error=max(matrix_error,float(np.max(abs(m.translate(s['translation_mm'])-np.array(s['transform'])))))
                if p['id'].endswith('__body19_final_datum_radial'):datum=datum and abs(np.array(s['translation_mm'])@normal)<1e-10
            axis=np.array(p['approach_axis']);moving=m.moved([lookup[k] for k in p['moving_names']],np.array(p['samples'][0]['transform']));fixed=[lookup[k] for k in present];mb=m.project_bounds(moving,axis);fb=m.project_bounds(fixed,axis);gap=max(fb[0]-mb[1],mb[0]-fb[1])
            outside=outside and gap>0 and abs(gap-p['initial_projection_separation_mm'])<1e-5
            if p['completed']:present.update(p['moving_names']);identity_error=max(identity_error,float(np.max(abs(np.array(p['samples'][-1]['transform'])-np.eye(4)))))
        if any(p['module']==name and p['stage']=='world' for p in raw['paths']):seq=seq and carrier==body
        if c.get('completed'):seq=seq and world==set(c['world_after_names']) and set(c['original24_names'])<=world
    contracts['actual_module_bases_and_guide_frames']=bool(basis)
    contracts['ringPIP_original_body19_bench92_same_input_no_repeated_native']=bool(inherit and raw['inherited_ringPIP_body19_bench_sample_scope']==92)
    contracts['new_whole_body_entry_uses_final_normal0_datum']=bool(datum)
    contracts['temporary_omissions_sequence_and_original_final_identity']=bool(seq and matrix_error<1e-10 and identity_error<1e-10)
    contracts['all_new_paths_start_outside_their_whole_stage_obstacles']=bool(outside)
    contracts['final_present_pending_and_restored_original_packets']=world==set(raw['packet_final_present_names']) and world|set(raw['packet_final_pending_names'])==set(lookup) and not world&set(raw['packet_final_pending_names']) and [p['module'] for p in packet if set(p['names'])<=world]==raw['restored_modules']
    contracts['remaining_rear3_and_front8_later']=raw['planned_rear_modules']==[p['module'] for p in packet if p['row']==0 and p['module'] not in prior['restored_modules']] and raw['explicitly_later_front8_modules']==[p['module'] for p in packet if p['row']==1]
    axes=True
    for z in raw['actual_new_axis_reads']:
        got=[]
        for face in lookup[z['name']]['shape'].Faces():
            if face.geomType()=='CYLINDER':
                cy=BRepAdaptor_Surface(face.wrapped).Cylinder();got.append(dict(radius_mm=cy.Radius(),axis=np.array(cy.Axis().Direction().Coord()),point=np.array(cy.Axis().Location().Coord())))
        axes=axes and len(got)==len(z['cylinders'])
        for a,b in zip(got,z['cylinders']):axes=axes and abs(a['radius_mm']-b['radius_mm'])<1e-5 and np.max(abs(a['axis']-b['axis']))<1e-5 and abs(a['axis']@np.array(z['expected_axis']))>1.-1e-10 and np.max(abs(a['point']-b['point']))<1e-5
    contracts['only_new_remaining_module_actual_M2_M3_axes']=bool(axes and len(raw['actual_new_axis_reads'])==2*sum(not c['bench_inherited'] for c in raw['module_contracts']))
    survey_ok=True
    for z in survey['actual_parts']:
        if z['name'] not in ['ring_pip_actuator_adapter','ring_pip_actuator_wire_guide_lid','ring_pip_actuator_wire_guide_base','arm065_shared_output_beam_rack_root']:continue
        sh=lookup[z['name']]['shape'];survey_ok=survey_ok and np.max(abs(np.array(n.c.m.pack._bbox(sh))-np.array(z['bbox'])))<1e-5
        for q in z['normal_plane_faces']:
            face=sh.Faces()[q['index']];plane=BRepAdaptor_Surface(face.wrapped).Plane()
            survey_ok=survey_ok and face.geomType()=='PLANE' and abs(face.Area()-q['area_mm2'])<1e-5 and np.max(abs(np.array(plane.Location().Coord())-q['point']))<1e-5
    contracts['actual_adapter_root_planes_and_lid_bounds_property_reads']=bool(survey_ok)
    contracts['old_ARM070_common_read_only_no_old_Boolean']=survey['old_common_Boolean_repeated']==0 and ev['old_native_path_groups_repeated']==0
    saved=n.c.load(n.p.read(ROOT/ev['common_scene'])['parts']) if ev['common_scene'] else []
    for q,z in zip(ev['common_queries'],saved):
        sh=n.g.original.transform(lookup[q['lead']]['shape'],np.array(q['lead_transform'])).intersect(lookup[q['part']]['shape'],tol=1e-6);ok=sh.isValid() and len(sh.Solids())==q['solids'] and abs(sh.Volume()-q['volume_mm3'])<1e-5
        result['common_samples'].append(dict(name=q['name'],matched=bool(ok),volume_mm3=sh.Volume(),solids=len(sh.Solids())));g=dict(name=q['name'],**metrics.geometry_match(sh,z['shape']));result['geometry_comparisons'].append(g)
        if not ok:result['differences'].append('common_'+q['name'])
        if not g['matched']:result['differences'].append('geometry_'+q['name'])
    for k,v in contracts.items():
        if not v:result['differences'].append(k)
    result.update(completed=True,native_groups=len(result['native_samples']),native_matches=sum(z['matched'] for z in result['native_samples']),geometry_shapes=len(result['geometry_comparisons']),geometry_matches=sum(z['matched'] for z in result['geometry_comparisons']),common_matches=sum(z['matched'] for z in result['common_samples']),all_translation_matrix_max_error=matrix_error,completed_final_identity_max_error=identity_error,input_hash_guard=guard(),checkout_clean=not git('status','--porcelain=v1'),
        replay_scope='New bench endpoints;world body first+last5,nut/base_nuts first+last3,M3first+last4,including first failure. Original16packet408/886/start598/prior5,actual placement/guideframes/new M2M3axes only,ARM070 exactbody19bench92 proof input only,whole outside starts/omissions/lists/original finalidentity. Read adapter/root planes/lid bounds and oldcommon withoutoldBoolean. Newcommon/signature if saved. Not all middle samples/old109/92/377/oldaxes/common/PCD615faces/holds/alltrimBoolean/pixels/armvisualunits,continuousfactory/tools/captive/supplier/mating/harnessflex/physical20drive/fullarm/exterior.')
    result['initial_core_passed']=not result['differences'] and result['input_hash_guard'] and result['checkout_clean'];save();print('core',result['initial_core_passed'],result['differences'],flush=True)


if __name__=='__main__':main()
