"""Exact-SHA replay of the explicitly new support/entry tests, including raw failures."""
import argparse
import importlib.util
import subprocess
import sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm060_review_model',Path(__file__).with_name('model.py'))
a=importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)
n,np,d,ROOT,HERE=a.n,a.np,a.d,a.ROOT,a.HERE
measure=n.m.drive.module('_arm060_metrics',ROOT/'cad/long_housing_route_v1/review_core.py')
follow=n.m.drive.module('_arm060_follow',HERE/'followup.py')
hub_follow=n.m.drive.module('_arm060_hub',HERE/'hub_followup.py')
from OCP.OSD import OSD_ThreadPool


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--commit',required=True)
    ap.add_argument('--output',required=True,type=Path)
    args=ap.parse_args()
    def git(*xs): return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1')
    assert not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
    manifest=n.p.read(ROOT/'checkpoints/ARM060.manifest.json')
    frozen={}
    for k in ['geometry_sha256','code_sha256','report_sha256']: frozen.update(n.p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[k])
    def guard(): return all(n.m.drive.sha(ROOT/f)==v['sha256'] for f,v in manifest['files'].items()) and all(n.m.drive.sha(ROOT/f)==v for f,v in frozen.items())
    assert guard() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    raw=n.p.read(HERE/'reviews/check.json')
    before,original,membership,prior,source,matrices=a.context()
    fresh,design=a.build(original)
    saved=n.c.load(n.p.read(ROOT/raw['scene'])['parts'])
    fixed=[z for z in before if z['name'] not in a.OLD]
    lookup={z['name']:z for z in before}
    output=[z for z in before if z['name'] in a.OUTPUT_NAMES]
    bounds,proof=a.rotation_bounds(output,[51.1,-154.4,7.5])
    nut=lookup['middle_dip_actuator_rack_nut_2']
    approach,approach_proof=a.nut_approach(nut,design['carrier']['M3_axis_world_mm'])
    protected=n.c.load(n.p.read(ROOT/raw['protection_scene'])['parts'])
    result=dict(completed=False,target_commit=args.commit,differences=[],geometry_comparisons=[],native_samples=[],attachment_samples=[],
        manifest_files=len(manifest['files']),frozen_files=len(frozen),OCC_pool_threads=1,
        current862_replaced=False,proposal871_adopted=False,same_agent_not_independent_audit=True)
    def save(): n.a.save(args.output,result)
    for x,y in zip(fresh+bounds+[approach],saved+protected):
        assert x['name']==y['name']
        got=dict(name=x['name'],**measure.match(measure.metrics(x['shape']),measure.metrics(y['shape'])))
        result['geometry_comparisons'].append(got)
        if not got['matched']: result['differences'].append('geometry_'+x['name'])
        save()
    def compare(label,got):
        want=raw['native'][label]
        l,r=d.overlap_map(got),d.overlap_map(want)
        err=max([abs(l[k]['material_intersection_mm3']-r[k]['material_intersection_mm3']) for k in l.keys()&r.keys()]+[0.])
        ok=l.keys()==r.keys() and err<1e-5 and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope']
        result['native_samples'].append(dict(id=label,native=got,matched=bool(ok),maximum_volume_difference_mm3=err))
        if not ok: result['differences'].append(label)
        save()
        print(label,bool(ok),flush=True)
    compare('new2__fixed869_neutral',n.c.material(saved,fixed))
    compare('new2_internal',n.c.material([saved[0]],[saved[1]]))
    compare('full_angle_bounds9__new2',n.c.material(protected[:9],saved))
    compare('nut_approach__new2',n.c.material([protected[-1]],saved))
    def query(label,operation,left,right):
        sh=left.cut(right,tol=1e-6) if operation=='cut' else left.intersect(right,tol=1e-6)
        volume=sh.Volume() if sh.Solids() else 0.
        want=next(z for z in raw['queries'] if z['id']==label)
        ok=sh.isValid() and len(sh.Solids())==want['solids'] and abs(volume-want['volume_mm3'])<1e-5
        result['attachment_samples'].append(dict(id=label,volume_mm3=volume,solids=len(sh.Solids()),matched=bool(ok)))
        if not ok: result['differences'].append(label)
        save()
    R=d.around((0,0,0),(0,1,0),-45.)
    pad=n.g.original.transform(a.a.a.box([66.5,-162,-17],[75.4,-159,-9]),R).intersect(original['shape'],tol=1e-6)
    query('original_middle_dip_outer_M3_pad__outside_new_carrier','cut',pad,saved[0]['shape'])
    hub=a.cq.Solid.makeCylinder(38.,3.,a.cq.Vector(0,-162,0),a.cq.Vector(0,1,0)).cut(
        a.cq.Solid.makeCylinder(34.,3.02,a.cq.Vector(0,-162.01,0),a.cq.Vector(0,1,0))).intersect(original['shape'],tol=1e-6)
    query('original_front_hub__outside_new_carrier','cut',hub,saved[0]['shape'])
    seat=lookup['compact_saddle_nested_turns']['shape'].intersect(a.a.a.box([54,-9,-58],[62,-6,-46]),tol=1e-6)
    query('original_wrist_M2_seat_material__outside_new_guide','cut',seat,saved[1]['shape'])
    for state in raw['output_commands']:
        C=np.array(state['transform'])
        compare('command_'+str(int(state['command_deg']))+'__output9__new2',n.c.material(
            [dict(z,shape=n.g.original.transform(z['shape'],C)) for z in output],saved))
    for state in raw['nut_insertion']:
        now=dict(nut,shape=nut['shape'].translate(tuple(state['translation_mm'])))
        compare('nut_entry_'+str(int(state['travel_mm']))+'__all_other870',n.c.material([now],saved+[z for z in fixed if z['name']!=nut['name']]))
    arm=n.p.read(ROOT/'cad/modular_roots_v9/design.json')['arm']
    transform_error=0.
    for state in raw['arm_motion']:
        F=d.transforms(state['angles_deg'],arm)
        transform_error=max(transform_error,max(float(np.max(abs(F[k]-state['transforms'][k]))) for k in F))
        moving=[dict(z,shape=n.g.original.transform(z['shape'],F[membership[z['name']]])) for z in saved]
        for i,z in enumerate(moving):
            body=membership[saved[i]['name']]
            obstacles=[dict(t,shape=n.g.original.transform(t['shape'],F[membership.get(t['name'],'forearm')]))
                for t in fixed if membership.get(t['name'],'forearm')!=body]+moving[i+1:]
            compare(state['id']+'__'+saved[i]['name']+'__relative_bodies',n.c.material([z],obstacles))
    def plain(v): return __import__('json').loads(__import__('json').dumps(v,default=lambda x:x.tolist() if hasattr(x,'tolist') else x))
    result['design_and_bound_contracts_matched']=plain(design)==raw['design'] and plain(proof)==raw['rotation_bounds'] and plain(approach_proof)==raw['nut_approach'] and transform_error<1e-10
    if not result['design_and_bound_contracts_matched']: result['differences'].append('contracts')
    extra,common=follow.calculate(raw)
    wanted=n.p.read(HERE/'reviews/followup.json')
    result['focused_followup_samples']=[]
    for x,y in zip(extra['native_queries'],wanted['native_queries']):
        ok=x['id']==y['id'] and x['solids']==y['solids'] and abs(x['volume_mm3']-y['volume_mm3'])<1e-5
        result['focused_followup_samples'].append(dict(id=x['id'],native=x,matched=bool(ok)))
        if not ok: result['differences'].append(x['id'])
    for x,y in zip(common,n.c.load(n.p.read(ROOT/wanted['scene'])['parts'])):
        got=dict(name=x['name'],**measure.match(measure.metrics(x['shape']),measure.metrics(y['shape'])))
        result['geometry_comparisons'].append(got)
        if not got['matched']: result['differences'].append('geometry_'+x['name'])
    extra=hub_follow.calculate();want=n.p.read(HERE/'reviews/hub_followup.json')
    ok=extra['solids']==want['solids'] and abs(extra['volume_mm3']-want['volume_mm3'])<1e-5
    result['hub_followup_matched']=bool(ok)
    result['hub_followup_native']=extra
    if not ok: result['differences'].append('hub_followup')
    result.update(completed=True,native_groups=len(result['native_samples']),native_matches=sum(z['matched'] for z in result['native_samples']),
        geometry_shapes=len(result['geometry_comparisons']),geometry_matches=sum(z['matched'] for z in result['geometry_comparisons']),
        attachment_matches=sum(z['matched'] for z in result['attachment_samples']),arm_transform_max_error=transform_error,
        focused_followup_matches=sum(z['matched'] for z in result['focused_followup_samples']),
        input_hash_guard=guard(),checkout_clean=not git('status','--porcelain=v1'),
        replay_scope='New33groups only;new2hardware+9full-angle conservative cylinders+1nut approach probe+1actualcommon source-STEP13 adaptive1e-10/absolute1e-5;3actual seat/hub cuts plus3input common constructions;3focused remaining-obstruction queries+1hub query;actual9bounds/data/body transforms/entry direction. No previousARM05931/20/5/4/M3pad+13/seam or all-surface trim Boolean equivalence;not independent audit,physical strength,actual tool,continuous arm,wholelongmaterials or wholehand drive approval.')
    result['initial_core_passed']=not result['differences'] and result['input_hash_guard'] and result['checkout_clean']
    save()
    print('core',result['initial_core_passed'],'differences',result['differences'],flush=True)


if __name__=='__main__': main()
