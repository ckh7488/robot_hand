"""Bounded exact-SHA replay, retaining failures and explicit alternate start."""
import argparse,importlib.util,subprocess,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm076_review',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,d,ROOT,HERE=m.n,m.np,m.d,m.ROOT,m.HERE
metrics=n.m.drive.module('_arm076_metrics',ROOT/'cad/recessed_mounting_root_v1/review_core.py')
from OCP.OSD import OSD_ThreadPool
from OCP.BRepAdaptor import BRepAdaptor_Surface


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1') and not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
    man=n.p.read(ROOT/'checkpoints/ARM076.manifest.json');frozen={}
    for k in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(n.p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[k])
    def guard():return all(n.m.drive.sha(ROOT/f)==z['sha256'] for f,z in man['files'].items()) and all(n.m.drive.sha(ROOT/f)==h for f,h in frozen.items())
    assert guard() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    rows,_,source,packets=m.context();lookup={z['name']:z for z in rows};final=set(lookup);q=n.p.read(HERE/'reviews/check.json');survey=n.p.read(HERE/'reviews/survey.json');ev=n.p.read(HERE/'reviews/evidence.json');prior=n.p.read(ROOT/'cad/branched_module_assembly_v1/reviews/check.json');sel=n.p.read(ROOT/'cad/branched_module_assembly_v1/reviews/selected_check.json')
    result=dict(completed=False,target_commit=args.commit,differences=[],native_samples=[],common_samples=[],geometry_comparisons=[],contracts={},manifest_files=len(man['files']),frozen_files=len(frozen),OCC_pool_threads=1,same_agent_not_independent_audit=True)
    def save():n.a.save(args.output,result)
    def replay_report(q,ev,label):
        for path in q['paths']:
            for s in m.replay_samples(path):
                got=n.c.material(m.moved([lookup[k] for k in path['moving_names']],np.array(s['transform'])),[lookup[k] for k in path['present_names']]);want=q['native'][s['id']];a,b=d.overlap_map(got),d.overlap_map(want);error=max([abs(a[k]['material_intersection_mm3']-b[k]['material_intersection_mm3']) for k in a.keys()&b.keys()]+[0.]);ok=a.keys()==b.keys() and error<1e-5 and got['pairs_in_scope']==want['pairs_in_scope'] and got['operation_failures']==want['operation_failures']
                result['native_samples'].append(dict(id=s['id'],report=label,native=got,matched=bool(ok),max_volume_difference_mm3=error))
                if not ok:result['differences'].append(label+'_'+s['id'])
                save();print(label,s['id'],bool(ok),flush=True)
        c={};mapped=q['mapped_special_packet'];body=set(q['body43_names']);nuts={f'thumb_ip_actuator_rack_nut_{i}' for i in [1,2]}
        c['original16packet408_and_one_support_replacement_886']=packets==q['original16packets']==prior['original16packets'] and sum(len(z['names']) for z in packets)==408 and q['explicit_replacement']=={m.OLD:m.NAME} and sorted(lookup)==q['final886_names'] and m.OLD not in lookup and m.NAME in lookup
        c['original47_support1_special48_and_body45']=mapped==prior['mapped_special_packet'] and q['original_special_packet']==prior['original_special_packet'] and len(mapped['names'])==48 and body==set(prior['body43_names']) and set(q['body45_names'])==body|nuts and len(q['body45_names'])==45
        c['preserved714_explicit_own44_separation_to670']=q['preserved_previous_world714_names']==sel['packet_final_present_names']==survey['preserved_previous_world714_names'] and set(q['explicitly_separated_own44_names'])==body|{'thumb_ip_actuator_rack_nut_1'} and len(q['explicitly_separated_own44_names'])==44 and set(q['initial_world_present_names'])==set(sel['packet_final_present_names'])-set(q['explicitly_separated_own44_names'])==set(prior['initial_world_present_names']) and len(q['initial_world_present_names'])==670
        proofs=m.inheritance();c['exact_independent98_electrical64_reuse162_newnative0']=proofs==q['independent_inherited_proofs']==survey['independent_inherited_proofs'] and q['inherited_sample_scope']==162 and q['inherited_native_groups_repeated']==0 and [z['sample_scope'] for z in proofs]==[14,42,42,64]
        carrier=set(q['initial_carrier_bench_present_names']);world=set(q['initial_world_present_names']);seq=carrier==set(prior['paths'][14]['present_names']) and len(carrier)==25;outside=True;failed=False;matrix_error=0.;identity_error=0.;ports_ok=True
        for path in q['paths']:
            present=carrier if path['stage']=='carrier_bench' else world;names=set(path['moving_names']);seq=seq and not failed and present==set(path['present_names']) and not present&names and set(path['absent_names'])==final-present
            if path['id'].startswith('preloaded_port'):ports_ok=ports_ok and nuts<=present
            if path['stage']=='world' and len(names)==45:seq=seq and carrier==set(q['body45_names']) and names==carrier
            axis=np.array(path['approach_axis']);moving=m.moved([lookup[k] for k in names],np.array(path['samples'][0]['transform']));mb=m.project_bounds(moving,axis);fb=m.project_bounds([lookup[k] for k in present],axis);gap=max(fb[0]-mb[1],mb[0]-fb[1]);outside=outside and gap>0 and abs(gap-path['initial_projection_separation_mm'])<1e-5
            for s in path['samples']:matrix_error=max(matrix_error,float(np.max(abs(m.translate(s['translation_mm'])-np.array(s['transform'])))))
            if path['completed']:present.update(names);identity_error=max(identity_error,float(np.max(abs(np.array(path['samples'][-1]['transform'])-np.eye(4)))))
            else:failed=True;seq=seq and path['samples'][-1]['id']==q.get('first_blocked_sample') and bool(q['native'][path['samples'][-1]['id']]['overlaps'] or q['native'][path['samples'][-1]['id']]['operation_failures'])
        c['sequence_omissions_no_duplicate_firstblock_finalidentity']=bool(seq and matrix_error<1e-10 and identity_error<1e-10 and sorted(carrier)==q['final_carrier_bench_present_names'] and sorted(world)==q['packet_final_present_names'])
        c['all_paths_full_obstacle_outside_start']=bool(outside)
        nutpaths=[z for z in q['paths'] if z['id'].startswith('preload_rack_nut_')];plan=['preload_rack_nut_1','preload_rack_nut_2'] if label=='initial' else ['preload_rack_nut_axial_1','preload_rack_nut_axial_2']
        c['both_rack_nuts_before_either_port_kit']=bool(ports_ok and [z['id'] for z in nutpaths]==plan[:len(nutpaths)] and q.get('rack_nuts_preloaded_before_both_ports',False)==(nuts<=carrier))
        c['body45_bench_claim_matches_actual_parts']=q['body45_bench_completed']==(carrier==set(q['body45_names']))
        c['rear8_retained_pending_inventory_restoration_claim']=set(prior['initial_world_present_names'])<=world and q['restored_before']==prior['restored_before'] and len(q['restored_before'])==8 and set(q['packet_final_pending_names'])==final-world and q['mapped_special48_restoration_completed']==(set(mapped['names'])<=world) and q['restored_modules']==q['restored_before']+q['new_restored_modules']
        c['reference12_stock_custom_and_current862_unchanged']=q['unit_reference12_names']==prior['unit_reference12_names'] and len(q['unit_reference12_names'])==12 and q['unit14_names']==prior['unit14_names'] and q['new_hardware_geometry']==0 and not q['current862_replaced'] and not q['proposal886_adopted']
        c['upper_M3_false216IN_readonly_no_repeat']=q['preserved_upper_M3_whole_cut']==prior['preserved_upper_M3_whole_cut'] and survey['preserved_M3_point_count']==216 and q['old_inclusion_Boolean_or_points_repeated']==0 and survey['old_Boolean_repeated']==0
        props_ok=True;maxprop=0.
        for z in survey['actual_rack_nut_and_adapter_properties']:
            sh=lookup[z['name']]['shape'];maxprop=max(maxprop,float(np.max(abs(np.array(n.c.m.pack._bbox(sh))-np.array(z['bbox'])))))
            for f in z['faces']:
                face=sh.Faces()[f['index']];ad=BRepAdaptor_Surface(face.wrapped);props_ok=props_ok and face.geomType()==f['type'];maxprop=max(maxprop,float(np.max(abs(np.array(n.c.m.pack._bbox(face))-np.array(f['bbox'])))))
                if f['type']=='CYLINDER':
                    cy=ad.Cylinder();maxprop=max(maxprop,abs(cy.Radius()-f['radius_mm']),float(np.max(abs(np.array(cy.Location().Coord())-np.array(f['axis_point'])))),float(np.max(abs(np.array(cy.Axis().Direction().Coord())-np.array(f['axis_direction'])))))
                else:
                    pl=ad.Plane();maxprop=max(maxprop,float(np.max(abs(np.array(pl.Location().Coord())-np.array(f['point'])))),float(np.max(abs(np.array(pl.Axis().Direction().Coord())-np.array(f['normal'])))))
        c['new_actual_nut_bore_capture_and_adapter_properties']=bool(props_ok and maxprop<1e-5)
        railok=True
        if 'base_nuts_actual_support_crossing_faces' in q:
            for z in q['base_nuts_actual_support_crossing_faces']:
                f=lookup[m.NAME]['shape'].Faces()[z['index']];pl=BRepAdaptor_Surface(f.wrapped).Plane();railok=railok and f.geomType()=='PLANE' and np.max(abs(np.array(pl.Location().Coord())-np.array(z['point'])))<1e-5 and np.max(abs(np.array(n.c.m.pack._bbox(f))-np.array(z['bbox'])))<1e-5
            bb=m.bbox([lookup['thumb_ip_actuator_base_nuts']]);levels=sorted({round(float(z['point'][0]-(bb[0,0]+bb[1,0])/2.),9) for z in q['base_nuts_actual_support_crossing_faces']});events=levels+[(x+y)/2. for x,y in zip(levels,levels[1:])];railok=railok and events==q['base_nuts_extra_axial_projection_samples_mm']
        c['reached_thin_rail_actual_face_event_sampling']=bool(railok)
        for want,z in zip(ev['common_queries'],n.c.load(n.p.read(ROOT/ev['common_scene'])['parts']) if ev['common_scene'] else []):
            sh=n.g.original.transform(lookup[want['lead']]['shape'],np.array(want['lead_transform'])).intersect(lookup[want['part']]['shape'],tol=1e-6);ok=sh.isValid() and len(sh.Solids())==want['solids'] and abs(sh.Volume()-want['volume_mm3'])<1e-5;result['common_samples'].append(dict(name=want['name'],matched=bool(ok),volume_mm3=sh.Volume(),solids=len(sh.Solids())));g=dict(name=want['name'],**metrics.geometry_match(sh,z['shape']));result['geometry_comparisons'].append(g)
            if not ok:result['differences'].append('common_'+want['name'])
            if not g['matched']:result['differences'].append('geometry_'+want['name'])
        result['contracts'].update({label+'_'+k:v for k,v in c.items()})
        return matrix_error,identity_error,maxprop
    selected=n.p.read(HERE/'reviews/selected_check.json');ev2=n.p.read(HERE/'reviews/evidence_selected.json')
    errors=[replay_report(q,ev,'initial'),replay_report(selected,ev2,'selected')]
    matrix_error=max(z[0] for z in errors);identity_error=max(z[1] for z in errors);maxprop=max(z[2] for z in errors);c=result['contracts']
    for k,v in c.items():
        if not v:result['differences'].append(k)
    result.update(completed=True,native_groups=len(result['native_samples']),native_matches=sum(z['matched'] for z in result['native_samples']),geometry_shapes=len(result['geometry_comparisons']),geometry_matches=sum(z['matched'] for z in result['geometry_comparisons']),common_matches=sum(z['matched'] for z in result['common_samples']),contracts_passed=sum(c.values()),all_translation_matrix_max_error=matrix_error,completed_final_identity_max_error=identity_error,actual_property_max_difference=maxprop,input_hash_guard=guard(),checkout_clean=not git('status','--porcelain=v1'),
        replay_scope='New path start/end,wholekit first+last3/world first+last5,all reached failedpath samples and reached actualrail face events. Original16packet408/explicit support1+original47/886,previous714 retained with own44 explicitly separated to670/rear8 maintained,all omissions/fulloutside/finalidentity/firstblock/pending/restoration claims. Exact independent98/electrical64 proof reuse,newnative0. Actualnut bore/capture/adapter property reads,newcommon typed signature epsilon1e-10/abs1e-5 only if generated. No oldM3 cut/216points/15motion/615PCD/old187 repeat. Not every middle/continuousfactory/transport/reverse disassembly/tools/captive/mating/harnessflex/forces/tension/electrical/20drive/fullarm/exterior/alltrimBoolean/pixels/armvisualunits.')
    result['initial_core_passed']=not result['differences'] and result['input_hash_guard'] and result['checkout_clean'];save();print('core',result['initial_core_passed'],result['differences'],flush=True)


if __name__=='__main__':main()
