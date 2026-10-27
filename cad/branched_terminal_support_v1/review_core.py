"""Exact clean-SHA bounded replay of a separate whole support replacement."""
import argparse,importlib.util,subprocess,sys,json
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm074_review',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,d,ROOT,HERE=m.n,m.np,m.d,m.ROOT,m.HERE
metrics=n.m.drive.module('_arm074_metrics',ROOT/'cad/recessed_mounting_root_v1/review_core.py')
from OCP.OSD import OSD_ThreadPool


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1') and not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
    man=n.p.read(ROOT/'checkpoints/ARM074.manifest.json');frozen={}
    for k in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(n.p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[k])
    def guard():return all(n.m.drive.sha(ROOT/f)==z['sha256'] for f,z in man['files'].items()) and all(n.m.drive.sha(ROOT/f)==h for f,h in frozen.items())
    assert guard() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    rows,membership,source=m.context();rebuilt,design=m.build(source);q=n.p.read(HERE/'reviews/continuation_check.json');ev=n.p.read(HERE/'reviews/evidence.json');saved=n.c.load(n.p.read(ROOT/q['scene'])['parts'])[0]
    lookup={z['name']:z for z in rows};lookup[m.NAME]=saved
    result=dict(completed=False,target_commit=args.commit,differences=[],native_samples=[],geometry_comparisons=[],common_samples=[],attachment_samples=[],contracts={},manifest_files=len(man['files']),frozen_files=len(frozen),OCC_pool_threads=1,same_agent_not_independent_audit=True)
    def save():n.a.save(args.output,result)
    g=dict(name=m.NAME,**metrics.geometry_match(rebuilt['shape'],saved['shape']));result['geometry_comparisons'].append(g)
    if not g['matched']:result['differences'].append('geometry_'+m.NAME)
    labels={'new_whole_support__all_retained885_neutral','output_-90__new_support','output_90__new_support','elbow_110__new_support_relative_bodies','roll_40__new_support_relative_bodies','flex_-35__new_support_relative_bodies','deviation_20__new_support_relative_bodies','compound_a__new_support_relative_bodies'}
    if q.get('first_new_blocked_group'):labels.add(q['first_new_blocked_group'])
    for contract in q['native_contracts']:
        if contract['id'] not in labels:continue
        L=np.array(contract['left_transform']);left=m.p.moved([lookup[k] for k in contract['left_names']],L)
        right=[dict(lookup[k],shape=n.g.original.transform(lookup[k]['shape'],np.array((contract['right_frame_transforms'] or {}).get(k,np.eye(4))))) for k in contract['right_names']]
        got=n.c.material(left,right);want=q['native'][contract['id']];a,b=d.overlap_map(got),d.overlap_map(want);err=max([abs(a[k]['material_intersection_mm3']-b[k]['material_intersection_mm3']) for k in a.keys()&b.keys()]+[0.]);ok=a.keys()==b.keys() and err<1e-5 and got['pairs_in_scope']==want['pairs_in_scope'] and got['operation_failures']==want['operation_failures']
        result['native_samples'].append(dict(id=contract['id'],native=got,matched=bool(ok),max_volume_difference_mm3=err))
        if not ok:result['differences'].append(contract['id'])
        save();print(contract['id'],bool(ok),flush=True)
    for got,want in zip(m.queries(saved,source),q['attachment_queries']):
        ok=got['id']==want['id'] and got['solids']==want['solids'] and abs(got['volume_mm3']-want['volume_mm3'])<1e-5;result['attachment_samples'].append(dict(**got,matched=bool(ok)))
        if not ok:result['differences'].append(got['id'])
    c=result['contracts'];plain=lambda x:json.loads(json.dumps(x,default=lambda z:z.tolist() if hasattr(z,'tolist') else z))
    c['new_whole_design_and_original_ports_M3_stack']=plain(design)==q['design']
    c['one_explicit_replacement_other885_unchanged']=sorted(lookup)==sorted(set(q['original_final886_names'])|{m.NAME}) and sorted((set(lookup)-{m.OLD}))==q['new_final886_names'] and q['design']['replaced_name']==m.OLD and q['design']['rows_replaced']==1
    old73=n.p.read(ROOT/'cad/captured_adjuster_kit_v1/reviews/check.json')
    c['rear8_world670_original216_pending_preserved']=q['initial_world_present_names']==old73['packet_final_present_names'] and q['pending_original_names']==old73['packet_final_pending_names'] and len(q['initial_world_present_names'])==670 and len(q['pending_original_names'])==216 and len(q['restored_modules'])==8 and not q['new_restored_modules'] and m.p.packets(rows)==q['original16packets']
    c['old49_92_not_inherited_to_changed_support_no_service_claim']=not q['previous_49_and92_assembly_to_changed_support_inherited'] and q['service_paths_executed']==0 and q['candidate_assembly_world48_front7_not_executed']
    first=n.p.read(HERE/'reviews/check.json')
    c['initial_pad_inclusion_false_not_overwritten_by_geometry_continuation']=first['new_native_groups']==0 and first['first_failed_attachment']=='whole_original_M3_pad_13.0_outside_candidate' and first['attachment_queries']==q['attachment_queries'] and q['preserved_initial_M3_pad_inclusion_false'] and q['initial_attachment_queries_repeated']==0 and not q['whole_candidate_approved']
    s=n.p.read(HERE/'reviews/survey.json');old59=n.p.read(ROOT/'cad/integrated_motor_end_v1/reviews/check.json')
    c['old3_port1_and_M3plus13_false_preserved_readonly']=s['old_port1_final_conflicts']==[z for z in old59['native']['new4__moved16']['overlaps'] if z['lead']==m.OLD] and s['old_M3_pad_queries_read']==old59['attachment_queries'][2:4] and s['old_native_Boolean_repeated']==0
    arm=n.p.read(ROOT/'cad/modular_roots_v9/design.json')['arm'];transforms_ok=True
    for contract in q['native_contracts']:
        if '__new_support_relative_bodies' in contract['id']:
            state=next(z for z in old59['arm_motion'] if contract['id']==z['id']+'__new_support_relative_bodies');F=d.transforms(state['angles_deg'],arm)
            transforms_ok=transforms_ok and np.max(abs(F['forearm']-np.array(contract['left_transform'])))<1e-10
            for name,M in contract['right_frame_transforms'].items():transforms_ok=transforms_ok and np.max(abs(F[membership.get(name,'forearm')]-np.array(M)))<1e-10
    c['actual_arm_frame_transforms_and_changed_relations']=bool(transforms_ok)
    c['first_new_block_stops_later_relations']=not q.get('first_new_blocked_group') or q['native_contracts'][-1]['id']==q['first_new_blocked_group']
    datum=n.m.drive.module('_arm074_datum_review',HERE/'read_mounting_datum.py');faces,points=datum.read(saved['shape']);want=n.p.read(HERE/'reviews/mounting_datum.json')
    ok=len(faces)==len(want['actual_upper_M3_bore_faces']) and len(points)==want['point_count']==216
    for a,b in zip(faces,want['actual_upper_M3_bore_faces']):ok=ok and a['index']==b['index'] and abs(a['radius_mm']-b['radius_mm'])<1e-5 and abs(a['area_mm2']-b['area_mm2'])<1e-5 and np.max(abs(np.array(a['bbox'])-b['bbox']))<1e-5
    for a,b in zip(points,want['material_points']):ok=ok and a['pad']==b['pad'] and a['candidate']==b['candidate'] and np.max(abs(a['point']-np.array(b['point'])))<1e-10
    c['actual_M3_faces_and216_points_do_not_override_whole_cut_false']=bool(ok and not want['whole_pad_containment_approved'] and want['new_comparison_Boolean_operations']==0)
    for want,z in zip(ev['common_queries'],n.c.load(n.p.read(ROOT/ev['common_scene'])['parts']) if ev['common_scene'] else []):
        sh=n.g.original.transform(lookup[want['lead']]['shape'],np.array(want['lead_transform'])).intersect(n.g.original.transform(lookup[want['part']]['shape'],np.array(want['part_transform'])),tol=1e-6);ok=sh.isValid() and len(sh.Solids())==want['solids'] and abs(sh.Volume()-want['volume_mm3'])<1e-5
        result['common_samples'].append(dict(name=want['name'],matched=bool(ok),volume_mm3=sh.Volume(),solids=len(sh.Solids())))
        g=dict(name=want['name'],**metrics.geometry_match(sh,z['shape']));result['geometry_comparisons'].append(g)
        if not ok:result['differences'].append('common_'+want['name'])
        if not g['matched']:result['differences'].append('geometry_'+want['name'])
    for k,v in c.items():
        if not v:result['differences'].append(k)
    result.update(completed=True,native_groups=len(result['native_samples']),native_matches=sum(z['matched'] for z in result['native_samples']),geometry_shapes=len(result['geometry_comparisons']),geometry_matches=sum(z['matched'] for z in result['geometry_comparisons']),common_matches=sum(z['matched'] for z in result['common_samples']),attachment_matches=sum(z['matched'] for z in result['attachment_samples']),input_hash_guard=guard(),checkout_clean=not git('status','--porcelain=v1'),
        replay_scope='New whole replacement signature,all4 new electrical/groove/twoM3 containment queries;static whole885,output2 and selected elbow110/roll40/flex-35/deviation20/compounda groups if reached,plus first new block. Whole originalpacket408/886 and explicit1replacement;rear8/world670/originalpending216,actualarm transforms/unchangedport-M3datums and old3/M3+13failure read only. No old49/92 assembly inheritance,no newservice/world48/front7;not allremainingmotion groups/continuouspaths/trimBoolean/pixels/armvisualunits/tools/strength/friction/preload/tension/electrical/20drive/fullarm/exterior.')
    result['initial_core_passed']=not result['differences'] and result['input_hash_guard'] and result['checkout_clean'];save();print('core',result['initial_core_passed'],result['differences'],flush=True)


if __name__=='__main__':main()
