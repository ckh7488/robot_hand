"""Exact-SHA bounded replay of newly executed assembly paths and common evidence."""
import argparse,importlib.util,subprocess,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm066_review',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,d,ROOT,HERE=m.n,m.np,m.d,m.ROOT,m.HERE
metrics=n.m.drive.module('_arm066_metrics',ROOT/'cad/recessed_mounting_root_v1/review_core.py')
from OCP.OSD import OSD_ThreadPool


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',required=True,type=Path);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1') and not args.output.exists()
    man=n.p.read(ROOT/'checkpoints/ARM066.manifest.json');frozen={}
    for k in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(n.p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[k])
    def guard():return all(n.m.drive.sha(ROOT/f)==z['sha256'] for f,z in man['files'].items()) and all(n.m.drive.sha(ROOT/f)==h for f,h in frozen.items())
    assert guard() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    rows,membership,source=m.context();lookup={z['name']:z for z in rows}
    first=n.p.read(HERE/'reviews/check.json');selected=n.p.read(HERE/'reviews/selected_check.json');ev=n.p.read(HERE/'reviews/evidence.json')
    result=dict(completed=False,target_commit=args.commit,differences=[],native_samples=[],common_samples=[],geometry_comparisons=[],
        manifest_files=len(man['files']),frozen_files=len(frozen),OCC_pool_threads=1,same_agent_not_independent_audit=True)
    def save():n.a.save(args.output,result)
    for phase,raw in [('first',first),('selected',selected)]:
        for path in raw['paths']:
            samples=list({s['id']:s for s in [path['samples'][0],path['samples'][-1]]}.values())
            for s in samples:
                got=n.c.material(m.moved([lookup[k] for k in path['moving_names']],s['translation_mm']),[lookup[k] for k in path['present_names']])
                want=raw['native'][s['id']];l,r=d.overlap_map(got),d.overlap_map(want)
                error=max([abs(l[k]['material_intersection_mm3']-r[k]['material_intersection_mm3']) for k in l.keys()&r.keys()]+[0.])
                ok=l.keys()==r.keys() and error<1e-5 and got['pairs_in_scope']==want['pairs_in_scope'] and got['operation_failures']==want['operation_failures']
                result['native_samples'].append(dict(id=phase+'_'+s['id'],native=got,matched=bool(ok),max_volume_difference_mm3=error))
                if not ok:result['differences'].append(phase+'_'+s['id'])
                save();print(phase,s['id'],bool(ok),flush=True)
    saved=n.c.load(n.p.read(ROOT/ev['common_scene'])['parts'])
    for q,row in zip(ev['common_queries'],saved):
        sh=n.g.original.transform(lookup[q['lead']]['shape'],np.array(q['lead_transform'])).intersect(lookup[q['part']]['shape'],tol=1e-6)
        ok=sh.isValid() and len(sh.Solids())==q['solids'] and abs(sh.Volume()-q['volume_mm3'])<1e-5
        result['common_samples'].append(dict(name=q['name'],matched=bool(ok),volume_mm3=sh.Volume(),solids=len(sh.Solids())))
        g=dict(name=q['name'],**metrics.geometry_match(sh,row['shape']));result['geometry_comparisons'].append(g)
        if not ok:result['differences'].append('common_'+q['name'])
        if not g['matched']:result['differences'].append('geometry_'+q['name'])
        save()
    survey=n.p.read(HERE/'reviews/survey.json');packets=m.packets(rows)
    result['packet16_rows408_and_original886_names_matched']=packets==survey['packets'] and sorted(lookup)==first['final_reference_names']==selected['final_reference_names']
    inherited=[s['id'] for p in first['paths'] if p['completed'] for s in p['samples']]
    result['inherited15_exact_input_and_present_lists_matched']=inherited==selected['inherited_first15_sample_ids'] and selected['initial_bench_present_names']==first['bench_final_present_names']
    result['pending_packet_stage_unexecuted_preserved']=not selected['packet_restoration_started'] and not selected['packet_restoration_completed'] and sorted(selected['packet_pending_names'])==sorted(k for p in packets for k in p['names'])
    for k in ['packet16_rows408_and_original886_names_matched','inherited15_exact_input_and_present_lists_matched','pending_packet_stage_unexecuted_preserved']:
        if not result[k]:result['differences'].append(k)
    result.update(completed=True,native_groups=len(result['native_samples']),native_matches=sum(z['matched'] for z in result['native_samples']),
        common_matches=sum(z['matched'] for z in result['common_samples']),geometry_shapes=len(result['geometry_comparisons']),
        geometry_matches=sum(z['matched'] for z in result['geometry_comparisons']),input_hash_guard=guard(),checkout_clean=not git('status','--porcelain=v1'),
        replay_scope='First/last executed samples of each new path,including both first blockers;two posed actual commons and source/STEP metric signatures;16packet408 membership and final886/reference/pending lists;15 exact inherited beam/rear-cap sample IDs. No other middle samples,oldARM06512/12/16pads/2preservation/common topology,frontunit geometry,615faces/PCD,wholefactory/continuousassembly/suppliedtools/preload/torque/20drive/physical/exterior approval.')
    result['initial_core_passed']=not result['differences'] and result['input_hash_guard'] and result['checkout_clean'];save()
    print('core',result['initial_core_passed'],result['differences'],flush=True)


if __name__=='__main__':main()
