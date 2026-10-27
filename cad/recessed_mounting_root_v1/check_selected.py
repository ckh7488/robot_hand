"""Only changed carrier relations after the bounded whole-root correction."""
import importlib.util
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm063_selected',Path(__file__).with_name('selected_model.py'))
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
n,np,d,ROOT,HERE=a.n,a.np,a.d,a.ROOT,a.HERE
from OCP.OSD import OSD_ThreadPool


def main():
    out=HERE/'reviews/selected_check.json';assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    start=time.monotonic();initial=n.p.read(HERE/'reviews/check.json');assert initial['completed'] and initial['input_hash_guard']
    before,membership,source=a.context();new,design=a.build(before)
    first=n.c.load(n.p.read(ROOT/initial['scene'])['parts'],source)
    other=[z for z in first if z['name']!=a.CARRIER];assert len(other)==5
    entries=n.c.load(n.p.read(ROOT/initial['entry_scene'])['parts'],source)
    fixed=[z for z in before if z['name'] not in a.OLD]
    protection=n.c.load(n.p.read(ROOT/initial['protection_input_scene'])['parts'],source)
    for f in [Path(__file__),HERE/'model.py',HERE/'selected_model.py',HERE/'reviews/check.json']:
        source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    raw=dict(completed=False,source_sha256=source,design=design,scene=n.a.a.prior.export([new],HERE/'generated/selected'),
        unchanged_initial_hardware5=[z['name'] for z in other],unchanged_entry_scene=initial['entry_scene'],
        native={},front_pad_queries=[],arm_motion=[],OCC_pool_threads=1,reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,
        current862_replaced=False,proposal871_adopted=False,initial_source_preserved=True,initial_groups_repeated=0,
        source_initial_group_relations_for_other5_reused=True,old_completed_stage_groups_repeated=0)
    def save():n.a.save(out,raw)
    def run(label,left,right):
        q=n.c.material(left,right);raw['native'][label]=q;save();print(label,q['pairs_in_scope'],q['exact_pairs'],len(q['overlaps']),flush=True)
    save()
    run('selected_carrier__retained865',[new],fixed)
    run('selected_carrier__unchanged_new_cap_fasteners5',[new],other)
    run('existing_rotation_bounds9__selected_carrier',protection[:9],[new])
    run('existing_nut_approach__selected_carrier',[protection[-1]],[new])
    run('unchanged_new_entry4__selected_carrier',entries,[new])
    for z in a.front_pad_inputs(before):
        q=z['shape'].cut(new['shape'],tol=1e-6);assert q.isValid()
        raw['front_pad_queries'].append(dict(id=z['id'],actual_input_volume_mm3=z['shape'].Volume(),outside_mm3=q.Volume() if q.Solids() else 0.,outside_solids=len(q.Solids())))
    beam=next(z['shape'] for z in fixed if z['name']=='Forearm_box_beam')
    raw['new_carrier_beam_distance_mm']=new['shape'].distance(beam)
    raw['new_carrier_fastener_distances_mm']=[dict(name=z['name'],distance_mm=new['shape'].distance(z['shape'])) for z in other[1:]]
    arm=n.p.read(ROOT/'cad/modular_roots_v9/design.json')['arm']
    for state in initial['arm_motion']:
        F=d.transforms(state['angles_deg'],arm)
        moving=dict(new,shape=n.g.original.transform(new['shape'],F['forearm']))
        obstacles=[dict(z,shape=n.g.original.transform(z['shape'],F[membership.get(z['name'],'forearm')]))
            for z in fixed if membership.get(z['name'],'forearm')!='forearm']
        run(state['id']+'__selected_carrier_relative_bodies',[moving],obstacles)
        raw['arm_motion'].append(dict(id=state['id'],angles_deg=state['angles_deg'],transforms=F))
    total=d.k.flat(list(raw['native'].values()))
    raw.update(completed=True,native_groups=len(raw['native']),pairs_in_scope=total['pairs_in_scope'],AABB_native_pairs=total['exact_pairs'],
        overlap_records=len(total['overlaps']),operation_failures=len(total['operation_failures']),raw_changed_carrier_clear=not total['overlaps'] and not total['operation_failures'],
        whole_proposal_raw_clear=False,input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()),elapsed_s=time.monotonic()-start)
    save();print('complete selected',raw['native_groups'],raw['overlap_records'],raw['operation_failures'],flush=True)


if __name__=='__main__':main()
