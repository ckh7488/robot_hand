"""Replay new path endpoints and audit exact original restoration contracts."""
import argparse,importlib.util,subprocess,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm068_review',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,d,ROOT,HERE=m.n,m.np,m.d,m.ROOT,m.HERE
from OCP.OSD import OSD_ThreadPool


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1') and not args.output.exists()
    man=n.p.read(ROOT/'checkpoints/ARM068.manifest.json');frozen={}
    for k in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(n.p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[k])
    def guard():return all(n.m.drive.sha(ROOT/f)==v['sha256'] for f,v in man['files'].items()) and all(n.m.drive.sha(ROOT/f)==h for f,h in frozen.items())
    assert guard() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    rows,_,source=m.context();lookup={z['name']:z for z in rows};raw=n.p.read(HERE/'reviews/selected_check.json');first=n.p.read(HERE/'reviews/check.json');ext=n.p.read(HERE/'reviews/entry_extension.json');old=n.p.read(ROOT/'cad/oriented_fastener_entry_v1/reviews/check.json')
    result=dict(completed=False,target_commit=args.commit,manifest_files=len(man['files']),frozen_files=len(frozen),OCC_pool_threads=1,differences=[],native_samples=[],
        source_geometry_changed=False,new_geometry_replays=0,common_Boolean_replays=0,same_agent_not_independent_audit=True)
    def save():n.a.save(args.output,result)
    for path in first['paths']+raw['paths']+ext['paths']:
        for i in sorted({0,len(path['samples'])-1}):
            s=path['samples'][i];got=n.c.material(m.moved([lookup[k] for k in path['moving_names']],np.array(s['transform'])),[lookup[k] for k in path['present_names']])
            want=(first['native'] if s['id'] in first['native'] else raw['native'] if s['id'] in raw['native'] else ext['native'])[s['id']];l,r=d.overlap_map(got),d.overlap_map(want)
            error=max([abs(l[k]['material_intersection_mm3']-r[k]['material_intersection_mm3']) for k in l.keys()&r.keys()]+[0.])
            ok=l.keys()==r.keys() and error<1e-5 and got['pairs_in_scope']==want['pairs_in_scope'] and got['operation_failures']==want['operation_failures']
            result['native_samples'].append(dict(id=s['id'],native=got,matched=bool(ok),max_volume_difference_mm3=error))
            if not ok:result['differences'].append(s['id'])
            save();print(s['id'],bool(ok),flush=True)
    contracts={};packets=m.packets(rows);packet=packets[0];base=packet['module']+'_base_nuts';body=set(packet['body_names'])-{base}
    contracts['original886_and16packet408']=sorted(lookup)==raw['original_final886_names'] and packets==raw['packets'] and sum(len(z['names']) for z in packets)==408
    contracts['body19_unit10_carrier9_partition']=body==set(raw['body19_names']) and set(raw['unit10_names'])|set(raw['carrier9_names'])==body and not set(raw['unit10_names'])&set(raw['carrier9_names']) and raw['temporarily_later_base_nuts']==base
    contracts['first20_and_same_bench14_preserved']=first['new_native_groups']==20 and len(first['native'][first['first_blocked_sample']]['overlaps'])==3 and raw['inherited_bench14_paths']==first['paths'][:2] and raw['inherited_bench14_native_repeated']==0
    states={'bench':set(raw['bench_initial_present_names']),'carrier_bench':set(raw['carrier_initial_present_names']),'world':set(raw['world_initial_assumed_present_names'])}
    subset_ok=True;world=states['world'];inherited_ids=[]
    for proof in raw['inherited_clear_paths']:
        prior=next(z for z in old['paths'] if z['id']==proof['id'])
        subset_ok=subset_ok and prior['completed'] and proof['samples']==prior['samples'] and set(proof['present_names'])==world
        subset_ok=subset_ok and set(proof['moving_names'])<=set(prior['moving_names']) and world<=set(prior['present_names'])
        subset_ok=subset_ok and all(not old['native'][s['id']]['overlaps'] and not old['native'][s['id']]['operation_failures'] for s in prior['samples'])
        subset_ok=subset_ok and set(proof['original_moving_names'])==set(prior['moving_names']) and set(proof['original_present_names'])==set(prior['present_names'])
        world.update(proof['moving_names']);inherited_ids.extend(s['id'] for s in proof['samples'])
    contracts['inherited_identical_subset_proofs']=bool(subset_ok and len(inherited_ids)==raw['subset_inherited_native_groups'])
    seq=True;pose_error=0.;matrix_error=0.
    for path in raw['paths']:
        present=states[path['stage']];seq=seq and present==set(path['present_names']) and not set(path['moving_names'])&present and set(path['absent_names'])==set(lookup)-present
        for s in path['samples']:matrix_error=max(matrix_error,float(np.max(abs(np.array(s['transform'])-m.translate(s['translation_mm'])))))
        if path['completed']:
            present.update(path['moving_names']);pose_error=max(pose_error,float(np.max(abs(np.array(path['samples'][-1]['transform'])-np.eye(4)))))
    contracts['explicit_lists_and_final_original_pose_continuity']=bool(seq and states['world']==set(raw['packet_final_present_names']) and pose_error<1e-10 and matrix_error<1e-10)
    contracts['final_present_pending_partition']=set(raw['packet_final_present_names'])|set(raw['packet_final_pending_names'])==set(lookup) and not set(raw['packet_final_present_names'])&set(raw['packet_final_pending_names'])
    restored=[p['module'] for p in packets if set(p['names'])<=states['world']]
    contracts['actual_complete_packet_count']=restored==raw['restored_modules']
    ev=n.p.read(HERE/'reviews/evidence_final.json');contracts['render_has_no_new_native_or_geometry']=ev['new_native_queries']==ev['new_common_Boolean_queries']==ev['new_STEP_STL_exports']==0
    ext_ok=ext['extended_access_clear'] and len(ext['paths'])==3
    for p in ext['paths']:
        prior=next(z for z in raw['paths'] if z['id']==p['reused_suffix_path_id'])
        wanted=prior['samples'][1:] if prior['id'].startswith('rack_M3') else prior['samples']
        s0=p['samples'][0];sh=m.moved([lookup[k] for k in p['moving_names']],np.array(s0['transform']))
        gap=float(m.bbox(sh)[0,0]-m.bbox([lookup[k] for k in p['present_names']])[1,0])
        ext_ok=ext_ok and p['completed'] and p['present_names']==prior['present_names'] and p['moving_names']==prior['moving_names'] and p['reused_suffix_samples']==wanted
        ext_ok=ext_ok and gap>0 and abs(gap-p['initial_X_AABB_separation_from_all_present_mm'])<1e-5 and p['saved_segment_repeated']==0
    contracts['whole_obstacle_outside_start_and_exact_suffix_proofs']=bool(ext_ok)
    result['contracts']=contracts;result['final_pose_identity_max_error']=pose_error;result['all_translation_matrix_max_error']=matrix_error
    for k,v in contracts.items():
        if not v:result['differences'].append(k)
    result.update(completed=True,native_groups=len(result['native_samples']),native_matches=sum(z['matched'] for z in result['native_samples']),
        subset_inherited_groups=len(inherited_ids),input_hash_guard=guard(),checkout_clean=not git('status','--porcelain=v1'),
        replay_scope='Each new executed path first/last,including failure if any;identical saved ARM067 body/nut clear-path subset proofs without Boolean repetition;original16packet408/final886/body19-unit10-carrier9/temporary omissions/list continuity/final original poses. No new hardware or evidence geometry,exports or common Boolean;not all middle samples,internal supplier/electrical/output assembly,continuous factory paths,tools,captive retention,threads,torque,force,physical,whole16drive/exterior qualification.')
    result['initial_core_passed']=not result['differences'] and result['input_hash_guard'] and result['checkout_clean'];save();print('core',result['initial_core_passed'],result['differences'],flush=True)


if __name__=='__main__':main()
