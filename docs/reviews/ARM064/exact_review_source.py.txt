"""Exact-SHA replay of only the new E/EF mounting proposal."""
import argparse,importlib.util,json,subprocess,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm064_review',Path(__file__).with_name('model.py'))
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
n,np,d,ROOT,HERE=a.n,a.np,a.d,a.ROOT,a.HERE
metrics=n.m.drive.module('_arm064_metrics',ROOT/'cad/recessed_mounting_root_v1/review_core.py')
from OCP.OSD import OSD_ThreadPool


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',required=True,type=Path);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1')
    assert not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
    man=n.p.read(ROOT/'checkpoints/ARM064.manifest.json');frozen={}
    for k in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(n.p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[k])
    def guard():return all(n.m.drive.sha(ROOT/f)==v['sha256'] for f,v in man['files'].items()) and all(n.m.drive.sha(ROOT/f)==v for f,v in frozen.items())
    assert guard() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    raw=n.p.read(HERE/'reviews/check.json');ev=n.p.read(HERE/'reviews/evidence.json')
    before,membership,source=a.context();built,entry_source,res_source,design=a.build(before)
    new=n.c.load(n.p.read(ROOT/raw['scene'])['parts']);entries=n.c.load(n.p.read(ROOT/raw['entry_scene'])['parts']);reserved=n.c.load(n.p.read(ROOT/raw['outer_scene'])['parts'])
    fixed=[z for z in before if z['name'] not in a.OLD]
    result=dict(completed=False,target_commit=args.commit,differences=[],native_samples=[],geometry_comparisons=[],common_samples=[],
        manifest_files=len(man['files']),frozen_files=len(frozen),OCC_pool_threads=1,same_agent_not_independent_audit=True)
    def save():n.a.save(args.output,result)
    def geom(left,right):
        for x,y in zip(left,right):
            assert x['name']==y['name'];got=dict(name=x['name'],**metrics.geometry_match(x['shape'],y['shape']))
            result['geometry_comparisons'].append(got)
            if not got['matched']:result['differences'].append('geometry_'+x['name'])
            save()
    geom(built,new);geom(entry_source,entries);geom(res_source,reserved)
    def compare(label,got):
        want=raw['native'][label];l,r=d.overlap_map(got),d.overlap_map(want)
        error=max([abs(l[k]['material_intersection_mm3']-r[k]['material_intersection_mm3']) for k in l.keys()&r.keys()]+[0.])
        ok=l.keys()==r.keys() and error<1e-5 and got['pairs_in_scope']==want['pairs_in_scope'] and got['operation_failures']==want['operation_failures']
        result['native_samples'].append(dict(id=label,native=got,matched=bool(ok),max_volume_difference_mm3=error))
        if not ok:result['differences'].append(label)
        save();print(label,bool(ok),flush=True)
    compare('new19__retained869',n.c.material(new,fixed))
    compare('new19_internal171',d.k.flat([n.c.material([z],new[i+1:]) for i,z in enumerate(new[:-1])]))
    compare('entry16__retained869',n.c.material(entries,fixed));compare('entry16__new19',n.c.material(entries,new))
    compare('structural3__source_outer_references2',n.c.material(new[:3],reserved))
    arm=n.p.read(ROOT/'cad/modular_roots_v9/design.json')['arm'];transform_error=0.
    for state in raw['arm_motion']:
        F=d.transforms(state['angles_deg'],arm);transform_error=max(transform_error,max(float(np.max(abs(F[k]-state['transforms'][k]))) for k in F))
        for frame in ['elbow','forearm']:
            moving=[dict(z,shape=n.g.original.transform(z['shape'],F[frame])) for z in new if z['body']==frame]
            obstacles=[dict(z,shape=n.g.original.transform(z['shape'],F[membership.get(z['name'],'forearm')])) for z in fixed if membership.get(z['name'],'forearm')!=frame]
            obstacles += [dict(z,shape=n.g.original.transform(z['shape'],F[z['body']])) for z in new if z['body']!=frame]
            compare(state['id']+'__new_'+frame+'_relative',n.c.material(moving,obstacles))
    lookup={z['name']:z for z in before+new+entries};commons=[]
    for i,want in enumerate(ev['common_queries']):
        q=lookup[want['lead']]['shape'].intersect(lookup[want['part']]['shape'],tol=1e-6)
        ok=q.isValid() and len(q.Solids())==want['solids'] and abs(q.Volume()-want['volume_mm3'])<1e-5
        result['common_samples'].append(dict(**want,matched=bool(ok),STEP_volume_mm3=q.Volume()))
        if not ok:result['differences'].append('common_'+str(i))
        commons.append(dict(name='arm064_obstruction_common_'+str(i),shape=q))
    if commons:geom(commons,n.c.load(n.p.read(ROOT/ev['common_scene'])['parts']))
    plain=lambda v:json.loads(json.dumps(v,default=lambda q:q.tolist() if hasattr(q,'tolist') else q))
    result['design_and_body_contract_matched']=plain(design)==raw['design'] and transform_error<1e-10
    if not result['design_and_body_contract_matched']:result['differences'].append('design_body_contract')
    result.update(completed=True,native_groups=len(result['native_samples']),native_matches=sum(z['matched'] for z in result['native_samples']),
        geometry_shapes=len(result['geometry_comparisons']),geometry_matches=sum(z['matched'] for z in result['geometry_comparisons']),
        common_matches=sum(z['matched'] for z in result['common_samples']),arm_transform_max_error=transform_error,
        input_hash_guard=guard(),checkout_clean=not git('status','--porcelain=v1'),
        replay_scope='13new groups:neutral hardware/internal/entry/structural3 vs source-derived2outer references and4E/EF relative arm poses. 19hardware+16entry+2outer+savedcommon2 sourceSTEP adaptive signatures,2actual common queries,design/source interface/body contracts. No prior ARM063/062/061/615face/tap-pin/oldPCD/seam/+-90 groups repeated. No fulltrimBoolean/pixels/armvisualunits,thread fit/preload/strength/friction/complete axial retention/assembly/tools/wholearm/longmaterials/electrical/20torque/physical/exterior qualification.')
    result['initial_core_passed']=not result['differences'] and result['input_hash_guard'] and result['checkout_clean']
    save();print('core',result['initial_core_passed'],result['differences'],flush=True)


if __name__=='__main__':main()
