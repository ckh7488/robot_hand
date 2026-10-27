"""Replay executed phase endpoints and exact original placements from a clean SHA."""
import argparse,importlib.util,subprocess,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm067_review',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,d,ROOT,HERE=m.n,m.np,m.d,m.ROOT,m.HERE
metrics=n.m.drive.module('_arm067_metrics',ROOT/'cad/recessed_mounting_root_v1/review_core.py')
from OCP.OSD import OSD_ThreadPool


def samples(path):
    indexes={0,len(path['samples'])-1}
    if path['id'].startswith('front_tilt_'):
        for phase in {s['phase'] for s in path['samples']}:
            found=[i for i,s in enumerate(path['samples']) if s['phase']==phase];indexes.update([found[0],found[-1]])
    if path['id'].startswith('packet_'):indexes.update(range(max(0,len(path['samples'])-3),len(path['samples'])))
    return [path['samples'][i] for i in sorted(indexes)]


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',required=True,type=Path);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1') and not args.output.exists()
    man=n.p.read(ROOT/'checkpoints/ARM067.manifest.json');frozen={}
    for k in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(n.p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[k])
    def guard():return all(n.m.drive.sha(ROOT/f)==z['sha256'] for f,z in man['files'].items()) and all(n.m.drive.sha(ROOT/f)==h for f,h in frozen.items())
    assert guard() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    rows,membership,source=m.context();lookup={z['name']:z for z in rows};raw=n.p.read(HERE/'reviews/check.json');ev=n.p.read(HERE/'reviews/evidence_final.json')
    result=dict(completed=False,target_commit=args.commit,differences=[],native_samples=[],common_samples=[],geometry_comparisons=[],front_route_contracts=[],
        manifest_files=len(man['files']),frozen_files=222,OCC_pool_threads=1,same_agent_not_independent_audit=True)
    def save():n.a.save(args.output,result)
    for path in raw['paths']:
        for s in samples(path):
            got=n.c.material(m.moved([lookup[k] for k in path['moving_names']],np.array(s['transform'])),[lookup[k] for k in path['present_names']])
            want=raw['native'][s['id']];l,r=d.overlap_map(got),d.overlap_map(want)
            error=max([abs(l[k]['material_intersection_mm3']-r[k]['material_intersection_mm3']) for k in l.keys()&r.keys()]+[0.])
            ok=l.keys()==r.keys() and error<1e-5 and got['pairs_in_scope']==want['pairs_in_scope'] and got['operation_failures']==want['operation_failures']
            result['native_samples'].append(dict(id=s['id'],native=got,matched=bool(ok),max_volume_difference_mm3=error))
            if not ok:result['differences'].append(s['id'])
            save();print(s['id'],bool(ok),flush=True)
        if path['id'].startswith('front_tilt_'):
            i=int(path['id'].split('_')[-1]);route=m.front_route([-25.5,25.5][i]);want=path['samples']
            err=max(float(np.max(abs(np.array(s['transform'])-np.array(t['transform'])))) for s,t in zip(route,want))
            ok=len(route)==len(want) and err<1e-10
            result['front_route_contracts'].append(dict(id=path['id'],poses=len(route),matrix_max_error=err,matched=bool(ok)))
            if not ok:result['differences'].append('route_'+path['id'])
    pose_saved=n.c.load(n.p.read(ROOT/ev['pose_scene'])['parts'])
    for q,z in zip(ev['pose_contracts'],pose_saved):
        sh=n.g.original.transform(lookup[q['source_name']]['shape'],np.array(q['transform']))
        g=dict(name=q['name'],**metrics.geometry_match(sh,z['shape']));result['geometry_comparisons'].append(g)
        if not g['matched']:result['differences'].append('geometry_'+q['name'])
    common_saved=n.c.load(n.p.read(ROOT/ev['common_scene'])['parts']) if ev['common_scene'] else []
    for q,z in zip(ev['common_queries'],common_saved):
        sh=n.g.original.transform(lookup[q['lead']]['shape'],np.array(q['lead_transform'])).intersect(lookup[q['part']]['shape'],tol=1e-6)
        ok=sh.isValid() and len(sh.Solids())==q['solids'] and abs(sh.Volume()-q['volume_mm3'])<1e-5
        result['common_samples'].append(dict(name=q['name'],matched=bool(ok),volume_mm3=sh.Volume(),solids=len(sh.Solids())))
        g=dict(name=q['name'],**metrics.geometry_match(sh,z['shape']));result['geometry_comparisons'].append(g)
        if not ok:result['differences'].append('common_'+q['name'])
        if not g['matched']:result['differences'].append('geometry_'+q['name'])
    inherited=n.p.read(ROOT/'cad/ordered_roll_root_assembly_v1/reviews/selected_check.json');packets=m.a.packets(rows)
    plain=[{k:v for k,v in p.items() if k not in ['radial_start_mm','initial_projection_separation_mm']} for p in raw['packets']]
    result['original16packet408_reference886_and_inherited10_matched']=packets==plain and sorted(lookup)==raw['original_final886_names'] and raw['initial_bench_present_names']==inherited['bench_final_present_names']
    result['bench22_and_pending_original_packet_lists_matched']=len(raw['bench_final_present_names'])==22 and set(raw['bench_final_present_names'])==set(raw['bench_names']) and set(raw['packet_final_present_names'])|set(raw['packet_final_pending_names'])==set(lookup) and not set(raw['packet_final_present_names'])&set(raw['packet_final_pending_names'])
    expected=set(raw['initial_bench_present_names']);sequence=True;max_final_pose_error=0.
    for p in raw['paths']:
        if p['stage']!='bench':continue
        sequence=sequence and expected==set(p['present_names'])
        if p['completed']:
            expected.update(p['moving_names']);max_final_pose_error=max(max_final_pose_error,float(np.max(abs(np.array(p['samples'][-1]['transform'])-np.eye(4)))))
    sequence=sequence and expected==set(raw['bench_final_present_names'])
    expected=set(raw['packet_assumed_initial_present_names'])
    for p in raw['paths']:
        if p['stage']!='packet':continue
        sequence=sequence and expected==set(p['present_names'])
        if p['completed']:
            expected.update(p['moving_names']);max_final_pose_error=max(max_final_pose_error,float(np.max(abs(np.array(p['samples'][-1]['transform'])-np.eye(4)))))
    sequence=sequence and expected==set(raw['packet_final_present_names']) and max_final_pose_error<1e-10
    result['installed_omitted_list_continuity_and_final_original_poses_matched']=bool(sequence)
    result['completed_path_final_identity_max_error']=max_final_pose_error
    pp=raw['packets'][0];normal=np.array(pp['approach_normal']);theta=np.deg2rad(pp['azimuth_deg']);radial=np.array([np.cos(theta),0.,np.sin(theta)])
    vectors=[8.*normal+t*radial for t in list(np.arange(pp['radial_start_mm'],0.,-20.))+[0.]]+[4.*normal,np.zeros(3)]
    path=next(p for p in raw['paths'] if p['id']=='packet_'+pp['module'])
    matrix_error=max(float(np.max(abs(m.translate(v)-np.array(s['transform'])))) for v,s in zip(vectors,path['samples']))
    result['first_packet11_pose_contract_matched']=len(vectors)==len(path['samples'])==11 and matrix_error<1e-10
    for key in ['original16packet408_reference886_and_inherited10_matched','bench22_and_pending_original_packet_lists_matched','installed_omitted_list_continuity_and_final_original_poses_matched','first_packet11_pose_contract_matched']:
        if not result[key]:result['differences'].append(key)
    result.update(completed=True,native_groups=len(result['native_samples']),native_matches=sum(z['matched'] for z in result['native_samples']),geometry_shapes=len(result['geometry_comparisons']),
        geometry_matches=sum(z['matched'] for z in result['geometry_comparisons']),common_matches=sum(z['matched'] for z in result['common_samples']),
        input_hash_guard=guard(),checkout_clean=not git('status','--porcelain=v1'),
        replay_scope='Executed phase endpoints of two front tilted bolt routes;other new path first/last plus packet radial-to-normal last3 samples;2posed bolt evidence signatures and new failed packet common/signature;60front pose matrices/16packet408/original886/inherited10/bench22/pending lists. No remaining middle samples/124all replay,ARM06668/common2,ARM065topology hold/front geometry/615faces/PCD,fullfactory/continuouspaths/suppliedtools/strength/preload/torque/physical/20drive/exterior qualification.')
    result['initial_core_passed']=not result['differences'] and result['input_hash_guard'] and result['checkout_clean'];save()
    print('core',result['initial_core_passed'],result['differences'],flush=True)


if __name__=='__main__':main()
