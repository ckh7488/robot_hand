"""Exact-SHA bounded replay of central frame,retained seats and reaction interfaces."""
import argparse
import importlib.util
import json
import subprocess
import sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm061_review_model',Path(__file__).with_name('model.py'))
a=importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)
n,np,d,ROOT,HERE=a.n,a.np,a.d,a.ROOT,a.HERE
metrics=n.m.drive.module('_arm061_metrics',ROOT/'cad/long_housing_route_v1/review_core.py')
follow=n.m.drive.module('_arm061_followup',HERE/'followup.py')
from OCP.OSD import OSD_ThreadPool


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',required=True,type=Path);args=ap.parse_args()
    def git(*xs): return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1')
    assert not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
    manifest=n.p.read(ROOT/'checkpoints/ARM061.manifest.json')
    frozen={}
    for k in ['geometry_sha256','code_sha256','report_sha256']: frozen.update(n.p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[k])
    def guard(): return all(n.m.drive.sha(ROOT/f)==v['sha256'] for f,v in manifest['files'].items()) and all(n.m.drive.sha(ROOT/f)==v for f,v in frozen.items())
    assert guard() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    raw=n.p.read(HERE/'reviews/check.json')
    before,membership,prior,source=a.context()
    fresh,design=a.build(before)
    new=n.c.load(n.p.read(ROOT/raw['scene'])['parts'])[0]
    fixed=[z for z in before if z['name']!=a.OLD]
    protection=n.c.load(n.p.read(ROOT/raw['protection_input_scene'])['parts'])
    result=dict(completed=False,target_commit=args.commit,differences=[],native_samples=[],geometry_comparisons=[],seat_samples=[],reaction_samples=[],
        manifest_files=len(manifest['files']),frozen_files=len(frozen),OCC_pool_threads=1,same_agent_not_independent_audit=True,
        current862_replaced=False,proposal871_adopted=False)
    def save(): n.a.save(args.output,result)
    def geometry(left,right):
        for x,y in zip(left,right):
            assert x['name']==y['name']
            got=dict(name=x['name'],**metrics.match(metrics.metrics(x['shape']),metrics.metrics(y['shape'])))
            result['geometry_comparisons'].append(got)
            if not got['matched']: result['differences'].append('geometry_'+x['name'])
            save()
    geometry([fresh],[new])
    def compare(label,got):
        want=raw['native'][label];l,r=d.overlap_map(got),d.overlap_map(want)
        error=max([abs(l[k]['material_intersection_mm3']-r[k]['material_intersection_mm3']) for k in l.keys()&r.keys()]+[0.])
        ok=l.keys()==r.keys() and error<1e-5 and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope']
        result['native_samples'].append(dict(id=label,native=got,matched=bool(ok),maximum_volume_difference_mm3=error))
        if not ok: result['differences'].append(label)
        save();print(label,bool(ok),flush=True)
    compare('new_carrier__all_retained870_neutral',n.c.material([new],fixed))
    compare('unchanged_full_angle_bounds9__new_carrier',n.c.material(protection[:9],[new]))
    compare('unchanged_nut_approach__new_carrier',n.c.material([protection[-1]],[new]))
    for seat in a.seat_inputs(before):
        got=seat['shape'].cut(new['shape'],tol=1e-6);volume=got.Volume() if got.Solids() else 0.
        want=next(z for z in raw['seat_queries'] if z['id']==seat['id'])
        ok=got.isValid() and len(got.Solids())==want['outside_solids'] and abs(volume-want['outside_mm3'])<1e-5
        result['seat_samples'].append(dict(id=seat['id'],outside_mm3=volume,outside_solids=len(got.Solids()),matched=bool(ok)))
        if not ok: result['differences'].append(seat['id'])
        save()
    for got,want in zip(a.reaction_interfaces(before),raw['reaction_interfaces']):
        ok=got['id']==want['id'] and all(abs(got[k]-want[k])<1e-5 for k in ['distance_mm','common_volume_mm3','common_area_mm2']) and all(got[k]==want[k] for k in ['common_solids','common_faces'])
        result['reaction_samples'].append(dict(native=got,matched=bool(ok)))
        if not ok: result['differences'].append(got['id'])
    distance=new['shape'].distance(next(z['shape'] for z in before if z['name']=='Forearm_box_beam'))
    result['new_carrier_beam_distance_mm']=distance
    if abs(distance-raw['new_carrier_beam_distance_mm'])>=1e-5: result['differences'].append('carrier_beam_distance')
    output=[z for z in before if z['name'] in a.p.OUTPUT_NAMES]
    for state in raw['output_commands']:
        C=np.array(state['transform'])
        compare('command_'+str(int(state['command_deg']))+'__actual_output9__new_carrier',n.c.material(
            [dict(z,shape=n.g.original.transform(z['shape'],C)) for z in output],[new]))
    arm=n.p.read(ROOT/'cad/modular_roots_v9/design.json')['arm'];transform_error=0.
    for state in raw['arm_motion']:
        F=d.transforms(state['angles_deg'],arm)
        transform_error=max(transform_error,max(float(np.max(abs(F[k]-state['transforms'][k]))) for k in F))
        moving=dict(new,shape=n.g.original.transform(new['shape'],F['forearm']))
        obstacles=[dict(z,shape=n.g.original.transform(z['shape'],F[membership.get(z['name'],'forearm')]))
            for z in fixed if membership.get(z['name'],'forearm')!='forearm']
        compare(state['id']+'__new_carrier__relative_bodies',n.c.material([moving],obstacles))
    extra,common=follow.calculate(raw);wanted=n.p.read(HERE/'reviews/followup.json')
    result['followup_samples']=[]
    for got,want in zip(extra['native_queries'],wanted['native_queries']):
        ok=got['id']==want['id'] and got['solids']==want['solids'] and abs(got['volume_mm3']-want['volume_mm3'])<1e-5
        result['followup_samples'].append(dict(id=got['id'],native=got,matched=bool(ok)))
        if not ok: result['differences'].append(got['id'])
    geometry(common,n.c.load(n.p.read(ROOT/wanted['scene'])['parts']))
    plain=lambda x:json.loads(json.dumps(x,default=lambda v:v.tolist() if hasattr(v,'tolist') else v))
    result['design_and_transform_contract_matched']=plain(design)==raw['design'] and transform_error<1e-10
    if not result['design_and_transform_contract_matched']: result['differences'].append('design_transform_contract')
    result.update(completed=True,native_groups=len(result['native_samples']),native_matches=sum(z['matched'] for z in result['native_samples']),
        geometry_shapes=len(result['geometry_comparisons']),geometry_matches=sum(z['matched'] for z in result['geometry_comparisons']),
        seat_matches=sum(z['matched'] for z in result['seat_samples']),reaction_matches=sum(z['matched'] for z in result['reaction_samples']),
        followup_matches=sum(z['matched'] for z in result['followup_samples']),arm_transform_max_error=transform_error,
        input_hash_guard=guard(),checkout_clean=not git('status','--porcelain=v1'),
        replay_scope='New17groups;newcarrier1+focusedcommon2 source-STEP3 adaptive1e-10/absolute1e-5;32actualM3pad/2lowercollar outside queries and34input commons;2reaction-interface commons/distances+1carrierbeam distance;5focused location queries;design/body transforms. PriorARM06033/13/3/4/shortentry24/oldfailedM3/seams not repeated. Not allsurface trim Boolean equivalence,independent audit,actual hub torque/assembly/tools/strength/friction/preload/tension/wholelongmaterials/continuous arm/upperarm/physical/manufacture/exterior approval.')
    result['initial_core_passed']=not result['differences'] and result['input_hash_guard'] and result['checkout_clean']
    save();print('core',result['initial_core_passed'],'differences',result['differences'],flush=True)


if __name__=='__main__': main()
