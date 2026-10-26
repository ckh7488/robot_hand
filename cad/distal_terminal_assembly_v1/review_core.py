"""Replay saved assembly stages; no regeneration of the OFFSET024 upper keeper."""
import sys,argparse,subprocess
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
from service import *
from OCP.OSD import OSD_ThreadPool
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert not args.output.exists() and not args.output.resolve().is_relative_to(ROOT) and git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1')
    manifest=p.read(ROOT/'checkpoints/ASSEMBLY025.manifest.json');frozen={}
    for key in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[key])
    def guard():return all(m.drive.sha(ROOT/z)==v['sha256'] for z,v in manifest['files'].items()) and all(m.drive.sha(ROOT/z)==h for z,h in frozen.items())
    assert guard();selection=p.read(HERE/'reviews/selection.json');r=p.read(HERE/selection['selected_report']);data,pack,hand,original,new,housing=setup();by={z['name']:z for z in original+new+housing};native=[];differences=[];seen=set()
    def compare(label,got,want):
        def mapping(q):return {(z['lead'],z['part']):z['material_intersection_mm3'] for z in q['overlaps']}
        a,b=mapping(got),mapping(want);ok=a.keys()==b.keys() and all(abs(a[x]-b[x])<1e-5 for x in a.keys()&b.keys()) and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope'];native.append(dict(id=label,native=got,matched=ok));print(label,ok,flush=True)
        if not ok:differences.append(label)
    def replay(rel,path,index):
        key=(rel,path['id'],index)
        if key in seen:return
        seen.add(key);sample=path['samples'][index];T=np.asarray(sample['transform_local']);W=g.original.WORLD@T@g.original.INV;names=path['moving'];ob=hand+pack+data[6]+data[7]+[by[n] for n in path['installed_before'] if n not in names]
        compare(rel+'_'+path['id']+'_'+str(index),c.material([dict(name=n,shape=g.original.transform(by[n]['shape'],W)) for n in names],ob),sample['native'])
    for path in r['paths']:
        replay(selection['selected_report'],path,len(path['samples'])-1)
        if not path['clear']:
            i=max(range(len(path['samples'])),key=lambda j:sum(z['material_intersection_mm3'] for z in path['samples'][j]['native']['overlaps']));replay(selection['selected_report'],path,i)
        if path['id'] in ['upper_kit_lateral','upper_keeper_after_kit','lower_bare_ferrule','lower_shoe_side_then_radial','lower_new_keeper','housing_end_0','housing_end_1']:
            replay(selection['selected_report'],path,0)
    benches=r.get('bench_groups',[r['bench_subassembly']] if 'bench_subassembly' in r else [])
    for j,bench in enumerate(benches):
        offset=translation(bench['bench_translation_neutral_IP_mm']);Wb=g.original.WORLD@offset@g.original.INV;ob=[dict(name=n,shape=g.original.transform(by[n]['shape'],Wb)) for n in bench['fixed']]
        for index in sorted({0,len(bench['samples'])-1}):
            sample=bench['samples'][index];W=g.original.WORLD@np.asarray(sample['transform_local'])@g.original.INV;compare('bench_'+str(j)+'_'+str(index),c.material([dict(name=n,shape=g.original.transform(by[n]['shape'],W)) for n in bench['moving']],ob),sample['native'])
    for rel in selection['history_reports']:
        old=p.read(HERE/rel);failed=[z for z in old['paths'] if not z['clear']]
        for path in failed:
            i=max(range(len(path['samples'])),key=lambda j:sum(z['material_intersection_mm3'] for z in path['samples'][j]['native']['overlaps']));replay(rel,path,i)
    clean=not git('status','--porcelain=v1');result=dict(completed=True,target_commit=args.commit,passed=not differences and guard() and clean,differences=differences,key_samples=native,selected_report=selection['selected_report'],selected_required_sequence_clear=r.get('sampled_sequence_clear',False),saved_geometry_reimported=True,source_STEP_equivalence_retested=False,inherited_OFFSET024_face_comparison_inconclusive=True,manifest_files=len(manifest['files']),frozen_files=len(frozen),input_hash_guard=guard(),checkout_clean=clean,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),independent_audit=False,whole_hand_drive_approved=False,scope='Stored STEP input hashes and selected finite assembly samples including original failures;no geometry regeneration or inherited torus comparison retry.No all samples/render,physical fixture/thread/tool,full long housing,cord threading,tension or motion approval.')
    p.write(args.output,result);assert result['passed'],differences
if __name__=='__main__':main()
