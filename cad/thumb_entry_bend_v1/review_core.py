"""Exact-SHA scoped native/geometry replay for the upper entry elbow."""
import sys,argparse,subprocess,ast
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import check_entry as b
a,m,g,c,p,np,cq,ROOT,HERE=b.a,b.m,b.g,b.c,b.p,b.np,b.cq,b.ROOT,b.HERE
measure=m.drive.module('_entry035_metrics',ROOT/'cad/long_housing_route_v1/review_core.py')
from OCP.OSD import OSD_ThreadPool
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert not args.output.exists() and not args.output.resolve().is_relative_to(ROOT) and git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1')
    man=p.read(ROOT/'checkpoints/ENTRY035.manifest.json');frozen={}
    for k in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[k])
    def guard():return all(m.drive.sha(ROOT/z)==v['sha256'] for z,v in man['files'].items()) and all(m.drive.sha(ROOT/z)==v for z,v in frozen.items())
    assert guard();static=p.read(HERE/'reviews/static.json');sr=p.read(HERE/'reviews/service.json');fresh,design=a.build();saved=c.load(p.read(HERE/'generated/parts/scene.json')['parts']);geometry=[];native=[];diff=[]
    for z in saved:
        source=next(s for s in fresh if s['name']==z['name']);got=dict(name=z['name'],**measure.match(measure.metrics(source['shape']),measure.metrics(z['shape'])));geometry.append(got)
        if not got['matched']:diff.append('geometry_'+z['name'])
    def combined(xs):return dict(pairs_in_scope=sum(z['pairs_in_scope'] for z in xs),overlaps=[v for z in xs for v in z['overlaps']],operation_failures=[v for z in xs for v in z['operation_failures']])
    def compare(label,got,want):
        mapping=lambda x:{(z['lead'],z['part']):z['material_intersection_mm3'] for z in x['overlaps']};aa,bb=mapping(got),mapping(want);err=max([abs(aa[k]-bb[k]) for k in aa.keys()&bb.keys()]+[0.]);ok=aa.keys()==bb.keys() and err<1e-5 and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope'];native.append(dict(id=label,native=got,max_numeric_difference=err,matched=bool(ok)));print(label,ok,flush=True)
        if not ok:diff.append(label)
    compare('internal3',combined([c.material([z],saved[j+1:]) for j,z in enumerate(saved)]),combined(static['internal']));data,pack,guide,wrist=b.inputs()
    for tag in ['hand_open','hand_parent_full']:
        case=next(z for z in static['cases'] if z['id']==tag);hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,case['angles_deg']);moving=[b.moved(z,case['angles_deg']) for z in saved]
        for k,ob in [('hand',hand),('packaging',pack),('electrical',data[6]),('arm',data[7]),('wrist_guide',wrist)]:compare(tag+'_'+k,c.material(moving,ob),case['native'][k])
    hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,np.zeros(20));body=next(z for z in saved if z['installed_part_candidate']);hand.append(body);by={z['name']:z for z in hand};shift=np.array([500.,0,0]);installed=[]
    for path in sr['bench_paths']:
        assert installed==path['installed_before'];names=path['moving'];fixed=[dict(by[n],shape=by[n]['shape'].translate(tuple(shift))) for n in path['fixed_bench']+installed]
        for j in [0,len(path['samples'])-1]:
            s=path['samples'][j];delta=tuple(s['translation_world_mm']);compare('bench_'+names[0]+'_'+str(j),c.material([dict(by[n],shape=by[n]['shape'].translate(delta)) for n in names],fixed),s['native'])
        installed+=names
    omitted=sr['hand_paths'][0]['absent_hand_parts'];kit=sr['kit_names'];base=[z for z in hand if z['name'] not in kit+omitted]+pack+data[6]+data[7]+wrist;installed=[]
    for path in sr['hand_paths']:
        assert installed==path['installed_before'];names=path['moving'];fixed=base+[by[n] for n in installed]
        for j in [0,len(path['samples'])-1]:
            s=path['samples'][j];delta=tuple(s['translation_world_mm']);compare('hand_'+path['id']+'_'+str(j),c.material([dict(by[n],shape=by[n]['shape'].translate(delta)) for n in names],fixed),s['native'])
        installed+=names
    assert set(installed)==set(kit+omitted)
    for j in [0,3,len(sr['feeding_samples'])-1]:
        sample=sr['feeding_samples'][j];ref=a.feed_window(sample['tip_material_coordinate_mm']);compare('feed_'+str(j),c.material([ref],hand+pack+data[6]+data[7]+wrist),sample['native'])
    original=HERE/'history/static_entry_model_initial.py.txt';assert m.drive.sha(original)==static['source_sha256']['cad/thumb_entry_bend_v1/entry_model.py'];functions=lambda z:{n.name:ast.dump(n,include_attributes=False) for n in ast.parse(z.read_text(encoding='utf-8')).body if isinstance(n,ast.FunctionDef)};old,new=functions(original),functions(HERE/'entry_model.py');same=all(old[k]==new[k] for k in old) and (HERE/'entry_model.py').read_bytes().startswith(original.read_bytes());assert same
    clean=not git('status','--porcelain=v1');result=dict(completed=True,target_commit=args.commit,passed=not diff and guard() and clean,differences=diff,key_samples=native,geometry_comparisons=geometry,static_original_geometry_functions_unchanged_after_feed_helper_addition=same,manifest_files=len(man['files']),frozen_files=len(frozen),input_hash_guard=guard(),checkout_clean=clean,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),independent_audit=False,whole_hand_drive_approved=False,scope='42nativegroups:internal1+2staticx5=11;6benchstart/end12=23;8handstart/end16=39;3feed=42.3source/STEP adaptive signatures abs1e-5.Jaw original geometry functions AST unchanged after adding independent feeding helper. Not all8static/49assembly/8feed/render/wholeflexiblelink/actualforcefriction/tension/physical;wristneutral only.Inherited STEP inclusion/face comparison inconclusive states untouched.')
    p.write(args.output,result);assert result['passed'],diff
if __name__=='__main__':main()
