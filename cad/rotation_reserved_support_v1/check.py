"""New support and joint entry geometry against actual hardware and motion bodies."""
import importlib.util
import sys
import time
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm060_model',Path(__file__).with_name('model.py'))
a=importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)
n,np,d,ROOT,HERE=a.n,a.np,a.d,a.ROOT,a.HERE
from OCP.OSD import OSD_ThreadPool


def main():
    out=HERE/'reviews/check.json'
    assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    started=time.monotonic()
    before,original,membership,prior,source,matrices=a.context()
    new,design=a.build(original)
    fixed=[z for z in before if z['name'] not in a.OLD]
    lookup={z['name']:z for z in before}
    output=[z for z in before if z['name'] in a.OUTPUT_NAMES]
    centre=[51.1,-154.4,7.5]
    bounds,proof=a.rotation_bounds(output,centre)
    nut=lookup['middle_dip_actuator_rack_nut_2']
    approach,approach_proof=a.nut_approach(nut,design['carrier']['M3_axis_world_mm'])
    for f in [Path(__file__),HERE/'model.py']:
        source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    scene=n.a.a.prior.export(new,HERE/'generated/hardware')
    bound_scene=n.a.a.prior.export(bounds+[approach],HERE/'generated/protection')
    raw=dict(completed=False,source_sha256=source,scene=scene,protection_scene=bound_scene,design=design,
        rotation_bounds=proof,nut_approach=approach_proof,native={},queries=[],arm_motion=[],
        current862_replaced=False,candidate_adopted=False,proposal_rows=871,new_hardware_rows=2,unchanged_rows=869,
        previous_ARM059_base_M3pad_plus13_failure_retested=False,previous_base_and_16_geometry_unchanged=True,
        reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,OCC_pool_threads=1,services_executed=0,
        long_materials_rebuilt=False,motor_command_or_clocking_changed=False)
    def save(): n.a.save(out,raw)
    def run(label,left,right):
        got=n.c.material(left,right)
        raw['native'][label]=got
        save()
        print(label,got['exact_pairs'],len(got['overlaps']),flush=True)
    def query(label,operation,left,right):
        sh=left.cut(right,tol=1e-6) if operation=='cut' else left.intersect(right,tol=1e-6)
        assert sh.isValid()
        got=dict(id=label,operation=operation,volume_mm3=sh.Volume() if sh.Solids() else 0.,solids=len(sh.Solids()))
        raw['queries'].append(got)
        save()
        return got
    save()
    run('new2__fixed869_neutral',new,fixed)
    run('new2_internal',[new[0]],[new[1]])
    run('full_angle_bounds9__new2',bounds,new)
    run('nut_approach__new2',[approach],new)
    R=d.around((0,0,0),(0,1,0),-45.)
    pad=n.g.original.transform(a.a.a.box([66.5,-162,-17],[75.4,-159,-9]),R).intersect(original['shape'],tol=1e-6)
    query('original_middle_dip_outer_M3_pad__outside_new_carrier','cut',pad,new[0]['shape'])
    # The hub is carried by identity outside the explicitly removed spoke slab.
    hub=a.cq.Solid.makeCylinder(38.,3.,a.cq.Vector(0,-162,0),a.cq.Vector(0,1,0)).cut(
        a.cq.Solid.makeCylinder(34.,3.02,a.cq.Vector(0,-162.01,0),a.cq.Vector(0,1,0)))
    hub=hub.intersect(original['shape'],tol=1e-6)
    query('original_front_hub__outside_new_carrier','cut',hub,new[0]['shape'])
    # Existing clamp faces and the two M2 seats are retained as actual material.
    faceblock=a.a.a.box([54,-9,-58],[62,-6,-46])
    seat=lookup['compact_saddle_nested_turns']['shape'].intersect(faceblock,tol=1e-6)
    query('original_wrist_M2_seat_material__outside_new_guide','cut',seat,new[1]['shape'])
    raw['output_commands']=[]
    for state in prior['output_command_checks']:
        C=np.array(state['transform'])
        assert np.max(abs((C@np.r_[centre,1.])[:3]-centre))<1e-10
        assert np.max(abs(C[:3,:3]@np.array([0,1,0])-np.array([0,1,0])))<1e-10
        run('command_'+str(int(state['command_deg']))+'__output9__new2',
            [dict(z,shape=n.g.original.transform(z['shape'],C)) for z in output],new)
        raw['output_commands'].append(dict(command_deg=state['command_deg'],transform=C))
    # Actual nut poses are finite translations; the larger cylinder above is a
    # support-only space reservation, not permission to remove obstacles.
    raw['nut_insertion']=[]
    for travel in [20.,10.,0.]:
        now=dict(nut,shape=nut['shape'].translate((0,-travel,0)))
        label='nut_entry_'+str(int(travel))+'__all_other870'
        run(label,[now],new+[z for z in fixed if z['name']!=nut['name']])
        raw['nut_insertion'].append(dict(travel_mm=travel,translation_mm=[0,-travel,0]))
    arm=n.p.read(ROOT/'cad/modular_roots_v9/design.json')['arm']
    for state in prior['arm_motion']:
        F=d.transforms(state['angles_deg'],arm)
        moving=[dict(z,shape=n.g.original.transform(z['shape'],F[membership[z['name']]])) for z in new]
        # Every changed part is checked against all hardware in a different
        # rigid body, and against the other changed part, at these finite poses.
        for i,z in enumerate(moving):
            name=new[i]['name'];body=membership[name]
            obstacles=[dict(t,shape=n.g.original.transform(t['shape'],F[membership.get(t['name'],'forearm')]))
                for t in fixed if membership.get(t['name'],'forearm')!=body]
            obstacles+=moving[i+1:]
            run(state['id']+'__'+name+'__relative_bodies',[z],obstacles)
        raw['arm_motion'].append(dict(id=state['id'],angles_deg=state['angles_deg'],transforms=F))
    totals=d.k.flat(list(raw['native'].values()))
    raw.update(completed=True,native_groups=len(raw['native']),pairs_in_scope=totals['pairs_in_scope'],
        AABB_native_pairs=totals['exact_pairs'],overlap_records=len(totals['overlaps']),
        operation_failures=len(totals['operation_failures']),raw_new_tests_clear=not totals['overlaps'] and not totals['operation_failures'],
        full_proposal_raw_clear=False,previous_failures_not_overridden=True,
        input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()),elapsed_s=time.monotonic()-started)
    save()
    print('complete',raw['native_groups'],raw['overlap_records'],raw['operation_failures'],flush=True)


if __name__=='__main__': main()
