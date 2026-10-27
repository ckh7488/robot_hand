"""Replay changed shared root only, including actual failed assembly if present."""
import argparse,importlib.util,json,subprocess,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm065_review',Path(__file__).with_name('model.py'))
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
n,np,d,ROOT,HERE=a.n,a.np,a.d,a.ROOT,a.HERE
metrics=n.m.drive.module('_arm065_metrics',ROOT/'cad/recessed_mounting_root_v1/review_core.py')
from OCP.OSD import OSD_ThreadPool


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',required=True,type=Path);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1')
    assert not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
    man=n.p.read(ROOT/'checkpoints/ARM065.manifest.json');frozen={}
    for k in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(n.p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[k])
    def guard():return all(n.m.drive.sha(ROOT/f)==v['sha256'] for f,v in man['files'].items()) and all(n.m.drive.sha(ROOT/f)==v for f,v in frozen.items())
    assert guard() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    raw=n.p.read(HERE/'reviews/check.json');ev=n.p.read(HERE/'reviews/evidence_final.json')
    before,membership,source=a.context();built,entry_source,inherit,design,region,plate=a.build(before)
    new=n.c.load(n.p.read(ROOT/raw['scene'])['parts']);entry=n.c.load(n.p.read(ROOT/raw['entry_scene'])['parts']);entries=inherit+entry
    fixed=[z for z in before if z['name'] not in a.OLD];final=fixed+new;lookup={z['name']:z for z in final}
    outer=n.c.load(n.p.read(ROOT/raw['reused_outer_scene'])['parts'])
    prior=n.p.read(ROOT/'cad/roll_bolted_interface_v1/reviews/check.json');oldentry=n.c.load(n.p.read(ROOT/prior['entry_scene'])['parts'])
    bounds=n.c.load(n.p.read(ROOT/raw['reused_protection_scene'])['parts'])[:9]
    result=dict(completed=False,target_commit=args.commit,differences=[],native_samples=[],geometry_comparisons=[],rear_pad_samples=[],common_samples=[],
        manifest_files=len(man['files']),frozen_files=len(frozen),OCC_pool_threads=1,same_agent_not_independent_audit=True)
    def save():n.a.save(args.output,result)
    def geom(left,right):
        assert len(left)==len(right)
        for x,y in zip(left,right):
            assert x['name']==y['name'];got=dict(name=x['name'],**metrics.geometry_match(x['shape'],y['shape']))
            result['geometry_comparisons'].append(got)
            if not got['matched']:result['differences'].append('geometry_'+x['name'])
            save()
    geom(built,new);geom(entry_source,entry)
    def compare(label,got):
        want=raw['native'][label];l,r=d.overlap_map(got),d.overlap_map(want)
        error=max([abs(l[k]['material_intersection_mm3']-r[k]['material_intersection_mm3']) for k in l.keys()&r.keys()]+[0.])
        ok=l.keys()==r.keys() and error<1e-5 and got['pairs_in_scope']==want['pairs_in_scope'] and got['operation_failures']==want['operation_failures']
        result['native_samples'].append(dict(id=label,native=got,matched=bool(ok),max_volume_difference_mm3=error))
        if not ok:result['differences'].append(label)
        save();print(label,bool(ok),flush=True)
    compare('new6__retained880',n.c.material(new,fixed))
    compare('new6_internal15',d.k.flat([n.c.material([z],new[i+1:]) for i,z in enumerate(new[:-1])]))
    compare('entry8__retained880',n.c.material(entries,fixed));compare('entry8__new6',n.c.material(entries,new))
    compare('new_structure2__source_outer2',n.c.material(new[:2],outer))
    compare('existing_rotation_bounds9__new_structure2',n.c.material(bounds,new[:2]))
    compare('retained_output_entry6__new_structure2',n.c.material([z for z in oldentry if z['name'].startswith('arm064_output_')],new[:2]))
    arm=n.p.read(ROOT/'cad/modular_roots_v9/design.json')['arm'];transform_error=0.
    for state in raw['arm_motion']:
        F=d.transforms(state['angles_deg'],arm);transform_error=max(transform_error,max(float(np.max(abs(F[k]-state['transforms'][k]))) for k in F))
        moving=[dict(z,shape=n.g.original.transform(z['shape'],F['forearm'])) for z in new]
        obstacles=[dict(z,shape=n.g.original.transform(z['shape'],F[membership.get(z['name'],'forearm')])) for z in fixed if membership.get(z['name'],'forearm')!='forearm']
        compare(state['id']+'__new_shared_unit_relative',n.c.material(moving,obstacles))
    selected=[]
    for path in sorted({z['path_index'] for z in raw['assembly']}):
        items=[z for z in raw['assembly'] if z['path_index']==path]
        selected += list({z['id']:z for z in [items[0],items[-1]]}.values())
    for z in selected:
        M=np.eye(4);M[:3,3]=z['translation_mm'];moving=dict(lookup[z['name']],shape=n.g.original.transform(lookup[z['name']]['shape'],M))
        compare(z['id'],n.c.material([moving],[lookup[k] for k in z['present_names']]))
    for pad,want in zip(a.rear_pads(before),raw['rear_pad_queries']):
        assert pad['id']==want['id'];q=pad['shape'].cut(new[0]['shape'],tol=1e-6);v=q.Volume() if q.Solids() else 0.
        ok=q.isValid() and len(q.Solids())==want['outside_solids'] and abs(v-want['outside_mm3'])<1e-5 and abs(pad['shape'].Volume()-want['input_volume_mm3'])<1e-5
        result['rear_pad_samples'].append(dict(id=pad['id'],outside_mm3=v,matched=bool(ok)))
        if not ok:result['differences'].append('pad_'+pad['id'])
    old=next(z['shape'] for z in before if z['name']==a.p.a.CARRIER)
    missing=old.cut(new[0]['shape'],tol=1e-6).cut(region,tol=1e-6);pd=plate.cut(new[0]['shape'],tol=1e-6)
    pres=dict(old_material_missing_outside_whole_replacement_region_mm3=missing.Volume() if missing.Solids() else 0.,
        output_plate_material_missing_mm3=pd.Volume() if pd.Solids() else 0.,queries_valid=missing.isValid() and pd.isValid())
    result['preservation']=pres
    if not pres['queries_valid'] or any(abs(pres[k]-raw['preservation'][k])>=1e-5 for k in pres if k!='queries_valid'):result['differences'].append('preservation')
    commons=[]
    for i,want in enumerate(ev['common_queries']):
        left=lookup[want['lead']]['shape']
        if 'lead_transform' in want:left=n.g.original.transform(left,np.array(want['lead_transform']))
        q=left.intersect(lookup[want['part']]['shape'],tol=1e-6)
        ok=q.isValid() and len(q.Solids())==want['solids'] and abs(q.Volume()-want['volume_mm3'])<1e-5
        result['common_samples'].append(dict(**want,matched=bool(ok),STEP_volume_mm3=q.Volume()))
        if not ok:result['differences'].append('common_'+str(i))
        commons.append(dict(name='arm065_actual_common_'+str(i),shape=q))
    if commons:geom(commons,n.c.load(n.p.read(ROOT/ev['common_scene'])['parts']))
    plain=lambda v:json.loads(json.dumps(v,default=lambda q:q.tolist() if hasattr(q,'tolist') else q))
    result['design_body_and_fastener_contract_matched']=plain(design)==raw['design'] and transform_error<1e-10
    if not result['design_body_and_fastener_contract_matched']:result['differences'].append('design_body_contract')
    result.update(completed=True,native_groups=len(result['native_samples']),native_matches=sum(z['matched'] for z in result['native_samples']),
        geometry_shapes=len(result['geometry_comparisons']),geometry_matches=sum(z['matched'] for z in result['geometry_comparisons']),
        rear_pad_matches=sum(z['matched'] for z in result['rear_pad_samples']),common_matches=sum(z['matched'] for z in result['common_samples']),
        arm_transform_max_error=transform_error,input_hash_guard=guard(),checkout_clean=not git('status','--porcelain=v1'),
        replay_scope='11changed shared-unit static/relative groups plus actual executed assembly path first/last samples only;new6hardware/new4entries and any newcommon signatures;16rear pad inputs/outsidecuts,whole replacement-region and plate preservation2;design/body/4explicit fastener moves. No previous ARM06413/39/2,ARM063frontpad/615face/PCD/seam groups repeated. No alltrimBoolean/pixels/armvisualunits/continuousassembly/tools/threadfit/preload/friction/strength/axialpullout/wholearm/longmaterials/electrical/20torque/physical/exterior qualification.')
    result['initial_core_passed']=not result['differences'] and result['input_hash_guard'] and result['checkout_clean']
    save();print('core',result['initial_core_passed'],result['differences'],flush=True)


if __name__=='__main__':main()
