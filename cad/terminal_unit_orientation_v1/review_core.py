"""Exact-commit replay of the declared terminal and command-zero cord proposal."""
import argparse
import importlib.util
import subprocess
import sys
from pathlib import Path

sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm056_review_model',Path(__file__).with_name('model.py'))
a=importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)
n,np,d,ROOT,HERE=a.n,a.np,a.d,a.ROOT,a.HERE
measure=n.m.drive.module('_arm056_metrics',ROOT/'cad/long_housing_route_v1/review_core.py')
partition=n.m.drive.module('_arm056_partition',HERE/'support_partition.py')
wrap=n.m.drive.module('_arm056_wrap',HERE/'wrap_diagnostic.py')
from OCP.OSD import OSD_ThreadPool


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--commit',required=True)
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args()
    def git(*xs):
        return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1')
    assert not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
    manifest=n.p.read(ROOT/'checkpoints/ARM056.manifest.json')
    frozen={}
    for key in ['geometry_sha256','code_sha256','report_sha256']:
        frozen.update(n.p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[key])
    def guard():
        return all(n.m.drive.sha(ROOT/f)==v['sha256'] for f,v in manifest['files'].items()) and all(n.m.drive.sha(ROOT/f)==h for f,h in frozen.items())
    assert guard() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    raw=n.p.read(HERE/'reviews/check.json')
    _,baseline,fixed,packet,T,S0,S,prior,older,source=a.context()
    fresh,design=a.build(packet,T,S0,S)
    fresh_cords,routes=a.cords(T,S0)
    saved=n.c.load(n.p.read(ROOT/raw['scene'])['parts'])
    saved_cords=n.c.load(n.p.read(ROOT/raw['cord_scene'])['parts'])
    changed=saved+fresh[1:]
    result=dict(completed=False,target_commit=args.commit,differences=[],native_samples=[],geometry_comparisons=[],
        route_comparisons=[],placement_contract={},manifest_files=len(manifest['files']),frozen_files=len(frozen),
        OCC_pool_threads=1,current862_installation_replaced=False,candidate_adopted=False,
        inherited12_native_records_retested=False,same_agent_not_independent_audit=True,
        full_arm_material_motion_or_exterior_approved=False)
    def save():
        n.a.save(args.output,result)
    save()
    for x,y in zip(fresh[:1]+fresh_cords,saved+saved_cords):
        assert x['name']==y['name']
        item=dict(name=x['name'],**measure.match(measure.metrics(x['shape']),measure.metrics(y['shape'])))
        result['geometry_comparisons'].append(item)
        if not item['matched']:
            result['differences'].append('geometry_'+x['name'])
        save()
    error=float(np.max(abs(S-np.array(raw['design']['transform_from_ARM055']))))
    names=design['moved_hardware_names']==raw['design']['moved_hardware_names']
    seats=max(float(np.max(abs(np.array(got[k])-want[k]))) for got,want in zip(design['actual_M3_seats'],raw['design']['actual_M3_seats']) for k in ['original_seat','rotated_frame_hole','bridge_radius_mm'])
    ok=error<1e-10 and names and seats<1e-10 and design['retained_original_frame_material_removed_mm3']<1e-8
    result['placement_contract']=dict(matched=bool(ok),rigid_rows=18,maximum_transform_difference=error,maximum_seat_difference_mm=seats,original_rotated_frame_removed_mm3=design['retained_original_frame_material_removed_mm3'])
    if not ok:
        result['differences'].append('placement_and_M3_seats')
    for got,want in zip(routes,raw['local_routes']):
        error=max(float(np.max(abs(np.array(got[k])-want[k]))) for k in ['tangent_exit_world_mm','R5_entry_world_mm','shoulder_world_mm','adjuster_mouth_world_mm','local_cord_length_mm','wrap_angle_deg'])
        ok=error<1e-10
        result['route_comparisons'].append(dict(line=got['line'],maximum_difference=error,matched=bool(ok)))
        if not ok:
            result['differences'].append('route_'+str(got['line']))
    def compare(label,got):
        want=raw['native'][label]
        l,r=d.overlap_map(got),d.overlap_map(want)
        error=max([abs(l[k]['material_intersection_mm3']-r[k]['material_intersection_mm3']) for k in l.keys()&r.keys()]+[0.])
        ok=l.keys()==r.keys() and error<1e-5 and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope']
        result['native_samples'].append(dict(id=label,native=got,matched=bool(ok),maximum_volume_difference_mm3=error))
        if not ok:
            result['differences'].append(label)
        save()
        print(label,bool(ok),flush=True)
    others=[z for v in fixed.values() for z in v]
    compare('changed19__unchanged853_neutral',n.c.material(changed,others))
    compare('new_frame__rotated18',n.c.material(changed[:1],changed[1:]))
    compare('local_cords12__hardware872',n.c.material(saved_cords,changed+others))
    compare('local_cords_internal',d.k.flat([n.c.material([z],saved_cords[i+1:]) for i,z in enumerate(saved_cords)]))
    arm=n.p.read(ROOT/'cad/modular_roots_v9/design.json')['arm']
    for state in raw['motion']:
        F=d.transforms(state['angles_deg'],arm)
        assert max(np.max(abs(F[key]-state['transforms'][key])) for key in F)<1e-10
        moving=[dict(z,shape=n.g.original.transform(z['shape'],F['forearm'])) for z in changed]
        for target in state['compared_frames']:
            obstacle=[dict(z,shape=n.g.original.transform(z['shape'],F[target])) for z in fixed[target]]
            compare(state['id']+'__changed19__'+target,n.c.material(moving,obstacle))
    adapter=next(z for z in others if z['name']=='thumb_ip_actuator_adapter')['shape']
    distance=float(changed[0]['shape'].distance(adapter))
    distance_ok=abs(distance-raw['bridge_adapter_distance_mm'])<1e-5
    if not distance_ok:
        result['differences'].append('adapter_distance')
    part,_=partition.query(raw,n.p.read(HERE/'reviews/evidence.json'))
    expected=n.p.read(HERE/'reviews/partition.json')['result']
    error=max(abs(part[k]-expected[k]) for k in ['common_volume_mm3','common_outside_rotated_original_frame_mm3'])
    part_ok=error<1e-5 and part['outside_solids']==expected['outside_solids']
    result['support_partition']=dict(actual=part,maximum_difference_mm3=error,matched=bool(part_ok))
    if not part_ok:
        result['differences'].append('support_partition')
    diagnostic=wrap.evaluate(routes)
    expected_wrap=n.p.read(HERE/'reviews/wrap_diagnostic.json')
    wrap_ok=all(diagnostic[k]==expected_wrap[k] for k in diagnostic)
    result['wrap_branch_diagnostic_matched']=bool(wrap_ok)
    if not wrap_ok:
        result['differences'].append('wrap_branch_diagnostic')
    result.update(completed=True,initial_core_passed=not result['differences'],
        native_groups=len(result['native_samples']),native_matches=sum(z['matched'] for z in result['native_samples']),
        geometry_matches=sum(z['matched'] for z in result['geometry_comparisons']),route_matches=sum(z['matched'] for z in result['route_comparisons']),
        bridge_adapter_distance_mm=distance,bridge_adapter_distance_matched=distance_ok,
        input_hash_guard=guard(),checkout_clean=not git('status','--porcelain=v1'),
        replay_scope='All28new groups77941pairs/24records,including19changed-vs853,newframe-vs18,12localcords-vs872 and66internalpairs,24changedhardware/body relations at12finitearmposes.13sourceSTEPmetrics epsilon1e-10/absolute1e-5,18rowtransform/M3seats,2R10/R5routes,1adapterdistance and1savedcommon-minus-rotated-original-frame cut. Localcords are command-zero only,not material-motion tests.12unchanged old records carried by identity,not native replay. No priorARM051..055/LINK050/assembly/renderpixels/alltrimBoolean/wholelongmaterials/dynamic electrical/20torque/continuousarm/physical/exterior approval.')
    save()


if __name__=='__main__':
    main()
