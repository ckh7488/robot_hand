"""Exact-SHA replay of the saved retention candidate, including its failures."""
import sys,argparse,subprocess
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import retention_model as m
check=m.drive.module('_retain026_check',Path(__file__).resolve().parent/'check.py')
g,c,p,np,cq,ROOT,HERE=check.g,check.c,check.p,check.np,check.cq,check.ROOT,check.HERE
from OCP.OSD import OSD_ThreadPool
metric=m.drive.module('_retain026_metric',ROOT/'cad/thumb_tendon_route_v1/review_core.py')
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert not args.output.exists() and not args.output.resolve().is_relative_to(ROOT) and git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1')
    manifest=p.read(ROOT/'checkpoints/RETAIN026.manifest.json');frozen={}
    for key in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[key])
    def guard():return all(m.drive.sha(ROOT/z)==v['sha256'] for z,v in manifest['files'].items()) and all(m.drive.sha(ROOT/z)==h for z,h in frozen.items())
    assert guard();r=p.read(HERE/'selected/reviews/check.json');selected=m.drive.module('_retain026_selected_model',HERE/'selected_retention.py');fresh,design=selected.build();data,pack,guide,housing=check.inputs(design);saved=c.load(p.read(ROOT/r['scene'])['parts']);lookup={z['name']:z for z in fresh};geometry=[];native=[];differences=[]
    for z in saved:
        got=dict(metric.match(metric.signature(lookup[z['name']]['shape']),metric.signature(z['shape'])),name=z['name']);geometry.append(got)
        if not got['matched']:differences.append('geometry_'+z['name'])
    def compare(label,got,want):
        def mapping(x):return {(z['lead'],z['part']):z['material_intersection_mm3'] for z in x['overlaps']}
        a,b=mapping(got),mapping(want);ok=a.keys()==b.keys() and all(abs(a[k]-b[k])<1e-5 for k in a.keys()&b.keys()) and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope'];native.append(dict(id=label,native=got,matched=ok));print(label,ok,flush=True)
        if not ok:differences.append(label)
    def combined(xs):return dict(pairs_in_scope=sum(x['pairs_in_scope'] for x in xs),overlaps=[z for x in xs for z in x['overlaps']],operation_failures=[z for x in xs for z in x['operation_failures']])
    compare('new_internal325',combined([c.material([z],saved[j+1:]) for j,z in enumerate(saved)]),combined(r['internal']))
    for label in ['open','parent_full']:
        case=next(z for z in r['cases'] if z['id']==label);hand,v,f,neutral=m.s.k.e.prior.hand_rows(data,guide,case['angles_deg'])
        posed=[dict(z,shape=v.relocate(z['shape'],f[z['body']],neutral[z['body']])) if z['body']!='forearm' else z for z in saved]
        hous=[dict(z,shape=v.relocate(z['shape'],f[z['body']],neutral[z['body']])) if z['body']!='forearm' else z for z in housing]
        for key,ob in [('hand',hand),('packaging',pack),('electrical',data[6]),('arm',data[7]),('housing_reference',hous)]:compare(label+'_'+key,c.material(posed,ob),case['native'][key])
    raw=p.read(HERE/'reviews/check.json');old=c.load(p.read(ROOT/raw['scene'])['parts'])
    for label in ['parent_three_quarter','parent_full']:
        case=next(z for z in raw['cases'] if z['id']==label);hand,v,f,neutral=m.s.k.e.prior.hand_rows(data,guide,case['angles_deg']);posed=[dict(z,shape=v.relocate(z['shape'],f[z['body']],neutral[z['body']])) if z['body']!='forearm' else z for z in old];compare('original_'+label+'_hand',c.material(posed,hand),case['native']['hand'])
    service=p.read(HERE/'reviews/service.json');by={z['name']:z for z in saved};hand,_,_,_=m.s.k.e.prior.hand_rows(data,guide,np.zeros(20))
    for site in service['sites']:
        for path in site['paths']:
            indices={0,len(path['samples'])-1}
            if not path['clear']:indices.add(max(range(len(path['samples'])),key=lambda j:sum(z['material_intersection_mm3'] for z in path['samples'][j]['native']['overlaps'])))
            ob=hand+pack+data[6]+data[7]+housing+[by[n] for n in path['other_clamps_and_replacement_ferrules_present']+path['installed_before']]
            for j in sorted(indices):
                sample=path['samples'][j];z=by[path['moving']];compare('service_'+site['tag']+'_'+path['moving']+'_'+str(j),c.material([dict(z,shape=z['shape'].translate(tuple(sample['translation_world_mm'])))],ob),sample['native'])
    order=p.read(HERE/'reviews/service_order.json');by={z['name']:z for z in saved+guide+housing}
    for path in order['bench_paths'][:2]:
        ob=[dict(by[n],shape=by[n]['shape'].translate(tuple(order['bench_translation_world_mm']))) for n in path['fixed_bench']+path['installed_before']]
        for j in [0,len(path['samples'])-1]:
            sample=path['samples'][j];compare('order_bench_'+path['moving'][0]+'_'+str(j),c.material([dict(by[n],shape=by[n]['shape'].translate(tuple(sample['translation_world_mm']))) for n in path['moving']],ob),sample['native'])
    for path in order['hand_paths']:
        kept=[z for z in guide if z['name'] not in path['omitted_guide']];hr,_,_,_=m.s.k.e.prior.hand_rows(data,kept,np.zeros(20));ob=hr+pack+data[6]+data[7]+[by[n] for n in path['base_extra_names']+path['installed_before']];indices={len(path['samples'])-1}
        if path['id'] in ['upper_assembled_kit','upper_keeper_after_kit']:indices.add(0)
        if not path['clear']:indices.add(max(range(len(path['samples'])),key=lambda j:sum(z['material_intersection_mm3'] for z in path['samples'][j]['native']['overlaps'])))
        for j in sorted(indices):
            sample=path['samples'][j];compare('order_hand_'+path['id']+'_'+str(j),c.material([dict(by[n],shape=by[n]['shape'].translate(tuple(sample['translation_world_mm']))) for n in path['moving']],ob),sample['native'])
    clean=not git('status','--porcelain=v1');result=dict(completed=True,target_commit=args.commit,passed=not differences and guard() and clean,differences=differences,key_samples=native,geometry_comparisons=geometry,manifest_files=len(manifest['files']),frozen_files=len(frozen),input_hash_guard=guard(),checkout_clean=clean,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),independent_audit=False,whole_hand_drive_approved=False,inherited_OFFSET024_face_comparison_inconclusive=True,scope='26newsource/STEP adaptive metric signatures;325internal pairs,open/full2poses with5groups each,2originalfailedhandgroups and executed representative assembly path start/end/worst failure.Sameagent.Not all8poses/allservice/render,closure/friction/load,longhousing,tension or fullhand approval.Inherited keeper geometry comparison not repeated or resolved.')
    p.write(args.output,result);assert result['passed'],differences
if __name__=='__main__':main()
