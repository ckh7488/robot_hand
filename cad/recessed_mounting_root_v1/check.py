"""Changed complete mounting unit and connected roots against all retained hardware."""
import importlib.util
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm063_model',Path(__file__).with_name('model.py'))
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
n,np,d,ROOT,HERE=a.n,a.np,a.d,a.ROOT,a.HERE
from OCP.OSD import OSD_ThreadPool


def main():
    out=HERE/'reviews/check.json';assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    start=time.monotonic();before,membership,source=a.context();new,entries,design=a.build(before)
    fixed=[z for z in before if z['name'] not in a.OLD];after=fixed+new
    assert len(fixed)==865 and len(after)==871
    for f in [Path(__file__),HERE/'model.py']:source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    protection_scene='cad/rotation_reserved_support_v1/generated/protection/scene.json'
    protection=n.c.load(n.p.read(ROOT/protection_scene)['parts'],source)
    raw=dict(completed=False,source_sha256=source,design=design,scene=n.a.a.prior.export(new,HERE/'generated/hardware'),
        entry_scene=n.a.a.prior.export(entries,HERE/'generated/entry_reference'),protection_input_scene=protection_scene,
        native={},front_pad_queries=[],arm_motion=[],OCC_pool_threads=1,reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,
        current862_replaced=False,proposal871_adopted=False,new_installed_hardware=0,service_paths_executed=0,
        old_completed_ARM062_or_ARM061_groups_repeated=0,old_output_plusminus90_samples_repeated=0,
        failed_ARM059_M3pad_or_old_seams_retested=False)
    def save():n.a.save(out,raw)
    def run(label,left,right):
        q=n.c.material(left,right);raw['native'][label]=q;save();print(label,q['pairs_in_scope'],q['exact_pairs'],len(q['overlaps']),flush=True)
    save()
    run('new_whole_unit6__retained865_neutral',new,fixed)
    internals=[n.c.material([z],new[i+1:]) for i,z in enumerate(new[:-1])]
    raw['native']['new_whole_unit6_internal15']=d.k.flat(internals);save()
    run('existing_rotation_bounds9__new_whole_unit6',protection[:9],new)
    run('existing_nut_approach__new_whole_unit6',[protection[-1]],new)
    run('new_entry4__whole_proposal871',entries,after)
    run('new_entry4__existing_rotation_bounds9',entries,protection[:9])
    for z in a.front_pad_inputs(before):
        q=z['shape'].cut(new[0]['shape'],tol=1e-6);assert q.isValid()
        raw['front_pad_queries'].append(dict(id=z['id'],actual_input_volume_mm3=z['shape'].Volume(),outside_mm3=q.Volume() if q.Solids() else 0.,outside_solids=len(q.Solids())))
        save()
    beam=next(z['shape'] for z in fixed if z['name']=='Forearm_box_beam')
    raw['new_clamp_beam_distances_mm']=[new[i]['shape'].distance(beam) for i in [0,1]]
    raw['stack_distances_mm']=[dict(name=z['name'],carrier_mm=z['shape'].distance(new[0]['shape']),cap_mm=z['shape'].distance(new[1]['shape'])) for z in new[2:]]
    prior=n.p.read(ROOT/'cad/central_reaction_frame_v1/reviews/check.json')
    arm=n.p.read(ROOT/'cad/modular_roots_v9/design.json')['arm']
    for state in prior['arm_motion']:
        F=d.transforms(state['angles_deg'],arm)
        moving=[dict(z,shape=n.g.original.transform(z['shape'],F['forearm'])) for z in new]
        obstacles=[dict(z,shape=n.g.original.transform(z['shape'],F[membership.get(z['name'],'forearm')]))
            for z in fixed if membership.get(z['name'],'forearm')!='forearm']
        run(state['id']+'__new6_relative_bodies',moving,obstacles)
        raw['arm_motion'].append(dict(id=state['id'],angles_deg=state['angles_deg'],transforms=F))
    total=d.k.flat(list(raw['native'].values()))
    raw.update(completed=True,native_groups=len(raw['native']),pairs_in_scope=total['pairs_in_scope'],AABB_native_pairs=total['exact_pairs'],
        overlap_records=len(total['overlaps']),operation_failures=len(total['operation_failures']),raw_new_unit_clear=not total['overlaps'] and not total['operation_failures'],
        whole_proposal_raw_clear=False,input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()),elapsed_s=time.monotonic()-start)
    save();print('complete',raw['native_groups'],raw['overlap_records'],raw['operation_failures'],flush=True)


if __name__=='__main__':main()
