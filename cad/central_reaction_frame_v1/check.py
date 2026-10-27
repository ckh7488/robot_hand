"""Changed central support only;all retained hardware and reaction gaps stay visible."""
import importlib.util
import sys
import time
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm061_model',Path(__file__).with_name('model.py'))
a=importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)
n,np,d,ROOT,HERE=a.n,a.np,a.d,a.ROOT,a.HERE
from OCP.OSD import OSD_ThreadPool


def main():
    out=HERE/'reviews/check.json'
    assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    started=time.monotonic()
    before,membership,prior,source=a.context()
    new,design=a.build(before)
    fixed=[z for z in before if z['name']!=a.OLD]
    protection=n.c.load(n.p.read(ROOT/prior['protection_scene'])['parts'],source)
    assert len(protection)==10
    for f in [Path(__file__),HERE/'model.py']:
        source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    raw=dict(completed=False,source_sha256=source,design=design,scene=n.a.a.prior.export([new],HERE/'generated/hardware'),
        native={},seat_queries=[],arm_motion=[],current862_replaced=False,proposal871_adopted=False,
        new_hardware_rows=1,unchanged_rows=870,protection_input_scene=prior['protection_scene'],
        unchanged9_rotation_bounds_rebuilt=False,ARM060_short_entry_tests_repeated=False,
        reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,OCC_pool_threads=1,service_paths_executed=0,
        long_materials_or_previous_failed_M3_pad_retested=False)
    def save(): n.a.save(out,raw)
    def run(label,left,right):
        got=n.c.material(left,right);raw['native'][label]=got;save()
        print(label,got['exact_pairs'],len(got['overlaps']),flush=True)
    save()
    run('new_carrier__all_retained870_neutral',[new],fixed)
    run('unchanged_full_angle_bounds9__new_carrier',protection[:9],[new])
    run('unchanged_nut_approach__new_carrier',[protection[-1]],[new])
    for seat in a.seat_inputs(before):
        outside=seat['shape'].cut(new['shape'],tol=1e-6)
        assert outside.isValid()
        raw['seat_queries'].append(dict(id=seat['id'],outside_mm3=outside.Volume() if outside.Solids() else 0.,
            outside_solids=len(outside.Solids()),original_actual_seat_volume_mm3=seat['shape'].Volume()))
        save()
    raw['reaction_interfaces']=a.reaction_interfaces(before)
    beam=next(z['shape'] for z in before if z['name']=='Forearm_box_beam')
    raw['new_carrier_beam_distance_mm']=new['shape'].distance(beam)
    save()
    output=[z for z in before if z['name'] in a.p.OUTPUT_NAMES]
    assert len(output)==9
    raw['output_commands']=prior['output_commands']
    for state in prior['output_commands']:
        C=np.array(state['transform'])
        run('command_'+str(int(state['command_deg']))+'__actual_output9__new_carrier',
            [dict(z,shape=n.g.original.transform(z['shape'],C)) for z in output],[new])
    arm=n.p.read(ROOT/'cad/modular_roots_v9/design.json')['arm']
    for state in prior['arm_motion']:
        F=d.transforms(state['angles_deg'],arm)
        moving=dict(new,shape=n.g.original.transform(new['shape'],F['forearm']))
        obstacles=[dict(z,shape=n.g.original.transform(z['shape'],F[membership.get(z['name'],'forearm')]))
            for z in fixed if membership.get(z['name'],'forearm')!='forearm']
        run(state['id']+'__new_carrier__relative_bodies',[moving],obstacles)
        raw['arm_motion'].append(dict(id=state['id'],angles_deg=state['angles_deg'],transforms=F))
    total=d.k.flat(list(raw['native'].values()))
    raw.update(completed=True,native_groups=len(raw['native']),pairs_in_scope=total['pairs_in_scope'],
        AABB_native_pairs=total['exact_pairs'],overlap_records=len(total['overlaps']),operation_failures=len(total['operation_failures']),
        raw_changed_carrier_clear=not total['overlaps'] and not total['operation_failures'],whole_proposal_raw_clear=False,
        previous_failures_not_overridden=True,input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()),
        elapsed_s=time.monotonic()-started)
    save();print('complete',raw['native_groups'],raw['overlap_records'],raw['operation_failures'],flush=True)


if __name__=='__main__': main()
