"""New terminal, real indexed spool and finite local geometry diagnostics."""
import importlib.util
import sys
import time
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm057_model',Path(__file__).with_name('model.py'))
a=importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)
n,np,d,ROOT,HERE=a.n,a.np,a.d,a.ROOT,a.HERE
from OCP.OSD import OSD_ThreadPool


def main():
    out=HERE/'reviews/check.json'
    assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    started=time.monotonic()
    _,baseline,fixed,packet,T,S0,S,prior,source=a.context()
    new,moved,clock,design=a.build(packet,T,S,baseline,source)
    cords,routes=a.cords(T,S0,baseline)
    for f in [Path(__file__),HERE/'model.py']:
        source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    scene=n.a.a.prior.export(new,HERE/'generated/hardware')
    cord_scene=n.a.a.prior.export(cords,HERE/'generated/command0')
    unchanged=[z for rows in fixed.values() for z in rows]
    changed=new+moved+clock
    assert len(changed)==26 and len(unchanged)==846
    before=n.p.read(ROOT/'cad/terminal_unit_orientation_v1/reviews/check.json')
    prior_records=list(before['inherited_unchanged_overlap_records'])
    prior_records += [dict(group=k,record=z) for k,t in before['native'].items() for z in t['overlaps'] if k!='local_cords_internal']
    changed_names={z['name'] for z in changed}|{a.p.NEW_FRAME,'arm055_split_return_0','arm055_split_return_1'}
    inherited=[z for z in prior_records if not ({z['record']['lead'],z['record']['part']} & changed_names)]
    invalidated=[z for z in prior_records if ({z['record']['lead'],z['record']['part']} & changed_names)]
    raw=dict(completed=False,source_sha256=source,scene=scene,cord_scene=cord_scene,design=design,local_routes=routes,
        current862_installation_replaced=False,candidate_adopted=False,proposal_rows=872,unchanged_rows=846,
        new_hardware_rows=3,relocated_fixed_rows=16,clocked_output_rows=7,local_cord_reference_rows=12,
        long_materials_installed=False,motor_installation_changed=False,service_paths_executed=0,
        inherited_unchanged_records=inherited,old_changed_records_not_carried=invalidated,inherited_native_retested=False,
        native={},arm_motion=[],motor_command_states=[],reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,OCC_pool_threads=1)
    def save(): n.a.save(out,raw)
    def run(label,left,right):
        raw['native'][label]=n.c.material(left,right)
        save()
        print(label,raw['native'][label]['exact_pairs'],len(raw['native'][label]['overlaps']),flush=True)
    def internal(label,rows):
        raw['native'][label]=d.k.flat([n.c.material([z],rows[i+1:]) for i,z in enumerate(rows)])
        save()
    save()
    run('changed26__unchanged846_neutral',changed,unchanged)
    run('new3__relocated16',new,moved)
    run('clocked7__fixed_terminal19',clock,new+moved)
    internal('new3_internal',new)
    run('local_cords12__hardware872',cords,changed+unchanged)
    internal('local_cords12_internal',cords)
    arm=n.p.read(ROOT/'cad/modular_roots_v9/design.json')['arm']
    for state in prior['motion']:
        F=d.transforms(state['angles_deg'],arm)
        moving=[dict(z,shape=n.g.original.transform(z['shape'],F['forearm'])) for z in changed+cords]
        for target in state['compared_frames']:
            obstacle=[dict(z,shape=n.g.original.transform(z['shape'],F[target])) for z in fixed[target]]
            run(state['id']+'__new26_and_local12__'+target,moving,obstacle)
        raw['arm_motion'].append(dict(id=state['id'],angles_deg=state['angles_deg'],compared_frames=state['compared_frames'],transforms=F))
        save()
    centre=(T@np.r_[a.p.terminal.M[:3,3],1.])[:3]
    stock_names={'thumb_ip_stock_output_horn','thumb_ip_stock_central_retainer'}
    stock=[z for z in unchanged if z['name'] in stock_names]
    statics=new+moved+[z for z in unchanged if z['name'] not in stock_names]
    assert len(stock)==2 and len(statics)==863
    for command in [-90.,90.]:
        C=d.around(centre,(0,1,0),-command)
        moving=[dict(z,shape=n.g.original.transform(z['shape'],C)) for z in clock+stock]
        local,info=a.cords(T,S0,baseline,command)
        folder='command_minus90' if command<0 else 'command_plus90'
        cord_path=n.a.a.prior.export(local,HERE/('generated/'+folder))
        label='command_'+str(int(command))
        run(label+'__output9__stationary863',moving,statics)
        run(label+'__cords12__hardware872',local,moving+statics)
        internal(label+'__cords_internal',local)
        raw['motor_command_states'].append(dict(command_deg=command,transform=C,cord_scene=cord_path,routes=info,
            independent_local_geometry_only=True,whole_cord_material_motion_or_joint_mapping=False))
        save()
    adapter=next(z['shape'] for z in unchanged if z['name']=='thumb_ip_actuator_adapter')
    raw['frame_adapter_distance_mm']=float(new[0]['shape'].distance(adapter))
    totals=d.k.flat(list(raw['native'].values()))
    raw.update(completed=True,native_groups=len(raw['native']),pairs_in_scope=totals['pairs_in_scope'],
        AABB_native_pairs=totals['exact_pairs'],overlap_records=len(totals['overlaps']),operation_failures=len(totals['operation_failures']),
        raw_changed_geometry_clear=not totals['overlaps'] and not totals['operation_failures'],full_proposal_raw_clear=False,
        input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()),elapsed_s=time.monotonic()-started)
    save()


if __name__=='__main__': main()
