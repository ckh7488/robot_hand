"""Replay every saved assembly path endpoint in an exact clean checkout."""
import sys,argparse,subprocess
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import remaining_service as a
m,c,p,np,ROOT,HERE=a.m,a.c,a.p,a.np,a.ROOT,a.HERE
from OCP.OSD import OSD_ThreadPool

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert not args.output.exists() and not args.output.resolve().is_relative_to(ROOT) and git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1')
    man=p.read(ROOT/'checkpoints/ASSEMBLY032.manifest.json');frozen={}
    for k in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[k])
    def guard():return all(m.drive.sha(ROOT/z)==v['sha256'] for z,v in man['files'].items()) and all(m.drive.sha(ROOT/z)==v for z,v in frozen.items())
    assert guard();r=p.read(HERE/'reviews/selected_service.json');raw=p.read(HERE/'reviews/service.json');data,allrows,by=a.inputs();native=[];differences=[];states=[]
    def compare(label,path,j,pool):
        s=path['samples'][j];moving=[dict(pool[n],shape=pool[n]['shape'].translate(tuple(s['translation_world_mm']))) for n in path['moving']];got=c.material(moving,[pool[n] for n in path['obstacle_names']]);want=s['native']
        mapping=lambda x:{(z['lead'],z['part']):z['material_intersection_mm3'] for z in x['overlaps']}
        aa,bb=mapping(got),mapping(want);err=max([abs(aa[k]-bb[k]) for k in aa.keys()&bb.keys()]+[0.]);ok=aa.keys()==bb.keys() and err<1e-5 and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope']
        native.append(dict(id=label,native=got,max_numeric_difference=err,matched=bool(ok)));print(label,ok,flush=True)
        if not ok:differences.append(label)
    for study in r['studies']:
        isbench=study['id']==r['bench_study_id'];pool={n:dict(by[n],shape=by[n]['shape'].translate(tuple(r['bench_translation_world_mm']))) for n in r['bench_kit_names']} if isbench else by
        installed=list(study['initial_installed'])
        for path in study['paths']:
            assert path['installed_before']==installed
            assert path['obstacle_names']==[n for n in installed if n not in path['moving']]
            assert path['absent_before']==sorted(set(pool)-set(installed))
            assert all((n in installed)==(path['action']=='remove') for n in path['moving'])
            indices={0,len(path['samples'])-1}
            if not path['clear']:indices.add(max(range(len(path['samples'])),key=lambda j:sum(z['material_intersection_mm3'] for z in path['samples'][j]['native']['overlaps'])))
            for j in sorted(indices):
                compare(study['id']+'_'+path['id']+'_'+str(j),path,j,pool)
            if path['clear']:
                if path['action']=='remove':installed=[n for n in installed if n not in path['moving']]
                else:installed+=path['moving']
        assert installed==study['final_installed'];states.append(dict(id=study['id'],continuous_installed_lists_verified=True,final_current_configuration_restored=set(installed)==(set(r['bench_kit_names']) if isbench else {z['name'] for z in allrows})))
    failed=raw['studies'][1]['paths'][-1];assert not failed['clear']
    for j in sorted({0,len(failed['samples'])-1,max(range(len(failed['samples'])),key=lambda j:sum(z['material_intersection_mm3'] for z in failed['samples'][j]['native']['overlaps']))}):compare('original_radial_jaw_failure_'+str(j),failed,j,by)
    clean=not git('status','--porcelain=v1');result=dict(completed=True,target_commit=args.commit,passed=not differences and guard() and clean,differences=differences,key_samples=native,installation_state_checks=states,manifest_files=len(man['files']),frozen_files=len(frozen),input_hash_guard=guard(),checkout_clean=clean,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),independent_audit=False,whole_hand_drive_approved=False,geometry_regenerated_or_source_STEP_compared=False,inherited_OFFSET024_face_comparison_inconclusive=True,scope='Start/end of every actually executed assembly path,plus maximum failure if distinct;all installed/absent/obstacle lists and final restoration checked.Two independent conditional studies using saved STEP only.No new geometry or source/STEP equivalence claim.Not intermediate finite samples/render/continuouspaths/jigs/tools/threads/torque/gripcompression/friction/pullout/wholehousing/threading/pretension/wholehand/physical approval;inherited comparisons untouched.')
    p.write(args.output,result);assert result['passed'],differences
if __name__=='__main__':main()
