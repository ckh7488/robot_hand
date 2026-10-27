"""Bounded exact-SHA replay;closed prior paths are proof inputs only."""
import argparse,importlib.util,subprocess,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm070_review',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,d,ROOT,HERE=m.n,m.np,m.d,m.ROOT,m.HERE
metrics=n.m.drive.module('_arm070_metrics',ROOT/'cad/recessed_mounting_root_v1/review_core.py')
from OCP.OSD import OSD_ThreadPool
from OCP.BRepAdaptor import BRepAdaptor_Surface


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1') and not args.output.exists()
    man=n.p.read(ROOT/'checkpoints/ARM070.manifest.json');frozen={}
    for k in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(n.p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[k])
    def guard():return all(n.m.drive.sha(ROOT/f)==v['sha256'] for f,v in man['files'].items()) and all(n.m.drive.sha(ROOT/f)==h for f,h in frozen.items())
    assert guard() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    rows,_,source=m.context();lookup={z['name']:z for z in rows};raw=n.p.read(HERE/'reviews/continuation_check.json');first=n.p.read(HERE/'reviews/check.json');selected=n.p.read(HERE/'reviews/selected_check.json');electrical_final=n.p.read(HERE/'reviews/final_check.json');ev=n.p.read(HERE/'reviews/evidence.json')
    prior=n.p.read(ROOT/'cad/rear_packet_restoration_v1/reviews/check.json')
    result=dict(completed=False,target_commit=args.commit,differences=[],native_samples=[],common_samples=[],geometry_comparisons=[],contracts={},manifest_files=len(man['files']),frozen_files=len(frozen),OCC_pool_threads=1,same_agent_not_independent_audit=True)
    def save():n.a.save(args.output,result)
    reports=[first,selected,electrical_final,raw]
    for path in [p for report in reports for p in report['paths']]:
        for s in m.replay_samples(path):
            got=n.c.material(m.moved([lookup[k] for k in s.get('native_moving_names',path['moving_names'])],np.array(s['transform'])),[lookup[k] for k in s.get('native_present_names',path['present_names'])]);want=next(z['native'][s['id']] for z in reports if s['id'] in z['native'])
            l,r=d.overlap_map(got),d.overlap_map(want);err=max([abs(l[k]['material_intersection_mm3']-r[k]['material_intersection_mm3']) for k in l.keys()&r.keys()]+[0.])
            ok=l.keys()==r.keys() and err<1e-5 and got['pairs_in_scope']==want['pairs_in_scope'] and got['operation_failures']==want['operation_failures']
            result['native_samples'].append(dict(id=s['id'],native=got,matched=bool(ok),max_volume_difference_mm3=err))
            if not ok:result['differences'].append(s['id'])
            save();print(s['id'],bool(ok),flush=True)
    contracts=result['contracts'];packets=m.packets(rows)
    contracts['original16packet408_final886_initial598_and_prior5_retained']=packets==raw['original16packets'] and sum(len(p['names']) for p in packets)==408 and sorted(lookup)==raw['original_final886_names'] and raw['initial_world_present_names']==prior['packet_final_present_names'] and len(raw['restored_before'])==5
    placements={p['module']:p for p in n.p.read(ROOT/'cad/eight_bank_layout_v1/reviews/check.json')['placements']}
    guides={p['module']:p for p in n.p.read(ROOT/'cad/wire_guides_v1/generated/scene.json')['groups']}
    wires={z['name']:z for z in n.p.read(ROOT/'cad/remote_motor_lead_v2/generated/scene.json')['parts']}
    world=set(raw['initial_world_present_names']);seq=True;basis=True;outside=True;matrix_error=0.;identity_error=0.;inherit=True;subset=True
    for report in reports:
        for p in report['paths']:
            for s in p['samples']:
                if 'native_moving_names' not in s:continue
                allpairs={(a,b) for a in p['moving_names'] for b in p['present_names']};newpairs={(a,b) for a in s['native_moving_names'] for b in s['native_present_names']};oldpairs=set();inh=s['inherited_query']
                if inh:
                    op=next(z for z in selected['paths'] if any(x['id']==inh['id'] for x in z['samples']));os=next(x for x in op['samples'] if x['id']==inh['id']);oq=selected['native'][inh['id']]
                    subset=subset and set(inh['moving_names'])<=set(op['moving_names']) and set(inh['present_names'])<=set(op['present_names']) and np.max(abs(np.array(os['transform'])-np.array(s['transform'])))<1e-10
                    subset=subset and not oq['operation_failures'] and not [z for z in oq['overlaps'] if z['lead'] in inh['moving_names'] and z['part'] in inh['present_names']]
                    oldpairs={(a,b) for a in inh['moving_names'] for b in inh['present_names']}
                subset=subset and newpairs|oldpairs==allpairs and not newpairs&oldpairs
    for c in raw['module_contracts']:
        name=c['module'];pl=placements[name];P=np.array(pl['proposed_installation_transform']);delta=np.array(pl['transform_from_current']);r=-P[:3,2];normal=P[:3,0];opening=delta[:3,:3]@np.array([0.,0.,1.])
        basis=basis and np.max(abs(P-np.array(c['placement'])))<1e-10 and np.max(abs(opening-np.array(c['guide_opening_axis'])))<1e-10 and np.max(abs(delta@np.array(guides[name]['frame_world_mm'])-np.array(c['guide_actual_frame'])))<1e-10
        for z in c['original_leads']:
            original=wires[z['name']];W=delta@np.array(original['transform_world_mm']);end=(W@np.r_[original['local']['end_mm'],1.])[:3]
            basis=basis and z['original_local']==original['local'] and np.max(abs(W-np.array(z['actual_transform'])))<1e-10 and np.max(abs(end-np.array(z['actual_end'])))<1e-10
        seq=seq and set(c['world_before_names'])==world
        body=set(c['body19_names']);unit=set(c['unit6_names']);electrical=set(c['electrical4_names']);carrier=set(c['adapter_names']);bench={name+'_cradle'}
        expected=unit|electrical|set(c['adapter_names'])|set(c['spacer_names'])|{c['guide_name'],c['lid_name']}|set(c['lid_nut_names'])|set(c['lid_screw_names'])
        seq=seq and body==expected and len(body)==19 and len(unit)==6 and len(electrical)==4 and c['temporarily_later_base_nuts'] not in body
        for inh in [z for z in raw['inherited_bench_paths'] if z['source_path'].startswith(name+'__')]:
            old=next(z for z in prior['paths'] if z['id']==inh['source_path'])
            inherit=inherit and old['completed'] and set(old['present_names'])==bench and inh['moving_names']==old['moving_names'] and inh['sample_ids']==[z['id'] for z in old['samples']] and inh['new_native_groups']==0
            inherit=inherit and all(not prior['native'][z['id']]['overlaps'] and not prior['native'][z['id']]['operation_failures'] for z in old['samples'])
            inherit=inherit and all(n.m.drive.sha(ROOT/f)==h for f,h in prior['source_sha256'].items()) and all(source[f]==h for f,h in inh['source_shape_hashes'].items())
            for s,x in zip(old['samples'],[80.,60.,40.,20.,10.,5.,0.]):inherit=inherit and np.max(abs(np.array(s['transform'])-m.translate(-x*r)))<1e-10
            bench.update(old['moving_names'])
        for inh in [z for z in raw['inherited_open_paths'] if z['path']['module']==name]:
            old=next(z for z in selected['paths'] if z['id']==inh['path']['id'])
            inherit=inherit and old==inh['path'] and old['completed'] and set(old['present_names'])==carrier and inh['new_native_groups']==0
            inherit=inherit and all(not selected['native'][z['id']]['overlaps'] and not selected['native'][z['id']]['operation_failures'] for z in old['samples'])
            carrier.update(old['moving_names'])
        for inh in [z for z in raw['inherited_electrical_paths'] if z['path']['module']==name]:
            old=next(z for z in electrical_final['paths'] if z['id']==inh['path']['id'])
            inherit=inherit and old==inh['path'] and old['completed'] and set(old['present_names'])==carrier and inh['new_native_groups']==0
            inherit=inherit and all(not electrical_final['native'][z['id']]['overlaps'] and not electrical_final['native'][z['id']]['operation_failures'] for z in old['samples'])
            carrier.update(old['moving_names'])
        for p in [p for p in raw['paths'] if p['module']==name]:
            present=world if p['stage']=='world' else bench if p['stage']=='bench' else carrier
            seq=seq and present==set(p['present_names']) and not present&set(p['moving_names']) and set(p['absent_names'])==set(lookup)-present
            for s in p['samples']:
                matrix_error=max(matrix_error,float(np.max(abs(m.translate(s['translation_mm'])-np.array(s['transform'])))))
                allpairs={(a,b) for a in p['moving_names'] for b in p['present_names']};newpairs={(a,b) for a in s['native_moving_names'] for b in s['native_present_names']};oldpairs=set()
                inh=s['inherited_query']
                if inh:
                    op=next(z for z in selected['paths'] if any(x['id']==inh['id'] for x in z['samples']));os=next(x for x in op['samples'] if x['id']==inh['id']);oq=selected['native'][inh['id']]
                    subset=subset and set(inh['moving_names'])<=set(op['moving_names']) and set(inh['present_names'])<=set(op['present_names']) and np.max(abs(np.array(os['transform'])-np.array(s['transform'])))<1e-10
                    subset=subset and not oq['operation_failures'] and not [z for z in oq['overlaps'] if z['lead'] in inh['moving_names'] and z['part'] in inh['present_names']]
                    oldpairs={(a,b) for a in inh['moving_names'] for b in inh['present_names']}
                subset=subset and newpairs|oldpairs==allpairs and not newpairs&oldpairs
            axis=np.array(p['approach_axis']);moving=m.moved([lookup[k] for k in p['moving_names']],np.array(p['samples'][0]['transform']));fixed=[lookup[k] for k in present]
            mb=m.project_bounds(moving,axis);fb=m.project_bounds(fixed,axis);gap=max(fb[0]-mb[1],mb[0]-fb[1])
            outside=outside and gap>0 and abs(gap-p['initial_projection_separation_mm'])<1e-5
            if p['completed']:
                present.update(p['moving_names']);identity_error=max(identity_error,float(np.max(abs(np.array(p['samples'][-1]['transform'])-np.eye(4)))))
        if any(p['module']==name and p['stage']=='world' for p in raw['paths']):seq=seq and body==carrier
        if c.get('completed'):seq=seq and world==set(c['world_after_names']) and body==carrier and set(c['original24_names'])<=world
    contracts['actual_groove_opening_frame_and_original3lead_paths']=bool(basis)
    contracts['same_ringPIP_M2_bench14_inheritance_no_native_repetition']=bool(inherit and raw['inherited_bench_groups']==14)
    contracts['same_open_guide21_electrical14_and_exact_unit_lead_subset12_only_new_relations']=bool(inherit and subset and raw['inherited_open_path_groups']==21 and raw['inherited_electrical_path_groups']==14 and raw['inherited_subset_sample_groups']==0 and electrical_final['inherited_subset_sample_groups']==12)
    contracts['explicit_bench_world_present_absent_sequence_and_final_original_poses']=bool(seq and matrix_error<1e-10 and identity_error<1e-10)
    contracts['every_new_bench_and_world_path_starts_outside_all_its_obstacles']=bool(outside)
    contracts['final_present_pending_partition_original_restoration']=world==set(raw['packet_final_present_names']) and world|set(raw['packet_final_pending_names'])==set(lookup) and not world&set(raw['packet_final_pending_names']) and [p['module'] for p in packets if set(p['names'])<=world]==raw['restored_modules']
    contracts['remaining_rear3_and_front8_explicitly_later']=raw['planned_rear_modules']==[p['module'] for p in packets if p['row']==0 and p['module'] not in prior['restored_modules']] and raw['explicitly_later_front8_modules']==[p['module'] for p in packets if p['row']==1]
    axes=True
    for z in ev['actual_lid_axis_reads']:
        cyl=[]
        for face in lookup[z['name']]['shape'].Faces():
            if face.geomType()=='CYLINDER':
                cy=BRepAdaptor_Surface(face.wrapped).Cylinder();cyl.append(dict(radius_mm=cy.Radius(),axis=np.array(cy.Axis().Direction().Coord()),point=np.array(cy.Axis().Location().Coord())))
        axes=axes and len(cyl)==len(z['cylinders'])
        for a,b in zip(cyl,z['cylinders']):axes=axes and abs(a['radius_mm']-b['radius_mm'])<1e-5 and np.max(abs(a['axis']-b['axis']))<1e-5 and abs(a['axis']@np.array(z['expected_axis']))>1.-1e-10 and np.max(abs(a['point']-b['point']))<1e-5
    contracts['new_actual_lid_M2_axes_match_opening_direction']=bool(axes and len(ev['actual_lid_axis_reads'])==len(raw['module_contracts']))
    common=n.c.load(n.p.read(ROOT/ev['common_scene'])['parts']) if ev['common_scene'] else []
    for q,z in zip(ev['common_queries'],common):
        sh=n.g.original.transform(lookup[q['lead']]['shape'],np.array(q['lead_transform'])).intersect(lookup[q['part']]['shape'],tol=1e-6)
        ok=sh.isValid() and len(sh.Solids())==q['solids'] and abs(sh.Volume()-q['volume_mm3'])<1e-5
        result['common_samples'].append(dict(name=q['name'],matched=bool(ok),volume_mm3=sh.Volume(),solids=len(sh.Solids())))
        g=dict(name=q['name'],**metrics.geometry_match(sh,z['shape']));result['geometry_comparisons'].append(g)
        if not ok:result['differences'].append('common_'+q['name'])
        if not g['matched']:result['differences'].append('geometry_'+q['name'])
    contracts['old_ARM069_common_read_only_no_old_boolean']=ev['old_failed_common_read_only'] and ev['old_native_path_groups_repeated']==0
    contracts['first_axial_guide_adapter_failure_preserved']=first['completed'] and first['new_native_groups']==5 and first['overlap_records']==1 and first['new_restored_modules']==[] and first['packet_final_present_names']==raw['initial_world_present_names'] and first['first_blocked_path'].endswith('__guide_base_before_motor')
    contracts['second_plug_rotor_reference_failure_preserved']=selected['completed'] and selected['new_native_groups']==33 and selected['overlap_records']==1 and selected['new_restored_modules']==[] and selected['packet_final_present_names']==raw['initial_world_present_names'] and selected['first_blocked_path'].endswith('__lay_electrical4_in_open_groove')
    contracts['third_axial_nut_guide_failure_preserved']=electrical_final['completed'] and electrical_final['new_native_groups']==19 and electrical_final['overlap_records']==1 and electrical_final['new_restored_modules']==[] and electrical_final['packet_final_present_names']==raw['initial_world_present_names'] and electrical_final['first_blocked_path'].endswith('__guide_nut_ring_pip_actuator_lid_nut_1')
    for k,v in contracts.items():
        if not v:result['differences'].append(k)
    result.update(completed=True,native_groups=len(result['native_samples']),native_matches=sum(z['matched'] for z in result['native_samples']),geometry_shapes=len(result['geometry_comparisons']),geometry_matches=sum(z['matched'] for z in result['geometry_comparisons']),common_matches=sum(z['matched'] for z in result['common_samples']),
        all_translation_matrix_max_error=matrix_error,completed_final_identity_max_error=identity_error,input_hash_guard=guard(),checkout_clean=not git('status','--porcelain=v1'),
        replay_scope='New bench path first/last;world body/nuts first and last3,M3first and last4,base_nuts first/last,including any first failure. Original16packet408/final886/initial598/prior5 retained,actual original groove/lead transforms,new actual lidM2axes,ringPIP exact14 proof input only,explicit omissions/sequence/finalidentity/whole-AABBoutside starts. New common/signature if saved. Not all middle samples,old377/axis10/common1/firstpacket/support/PCD615faces/holds,alltrimBoolean/pixels/armvisualunits,continuousfactory/tools/captive/supplier/electrical/outputinternal/physical20drive/fullarm/exterior.')
    result['initial_core_passed']=not result['differences'] and result['input_hash_guard'] and result['checkout_clean'];save();print('core',result['initial_core_passed'],result['differences'],flush=True)


if __name__=='__main__':main()
