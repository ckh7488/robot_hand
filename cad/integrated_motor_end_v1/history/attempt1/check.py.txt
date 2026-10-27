"""New actual-support candidate: changed hardware, protected groove and finite arm relations."""
import importlib.util
import sys
import time
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm059_model',Path(__file__).with_name('model.py'))
a=importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)
n,np,d,ROOT,HERE=a.n,a.np,a.d,a.ROOT,a.HERE
from OCP.OSD import OSD_ThreadPool


def main():
    out=HERE/'reviews/check.json'
    assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    started=time.monotonic()
    rows,membership,prior,delta,source=a.context()
    new,moved,fixed,design=a.build(rows,prior)
    shift=np.array(design['mechanical_translation_from_ARM057'])
    cords,routes=a.cords(prior,shift)
    for f in [Path(__file__),HERE/'model.py',ROOT/'cad/motor_end_integration_basis_v1/reviews/survey.json']:
        source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    scene=n.a.a.prior.export(new,HERE/'generated/hardware')
    cord_scene=n.a.a.prior.export(cords,HERE/'generated/command0')
    raw=dict(completed=False,source_sha256=source,scene=scene,cord_scene=cord_scene,design=design,local_routes=routes,
        current862_installation_replaced=False,candidate_adopted=False,proposal_rows=871,
        new_hardware_rows=4,relocated_fixed_rows=16,unchanged_rows=851,
        integrated_two_support_functions_to_one_part=True,no_electrical_hardware_deleted_or_shifted=True,
        long_materials_installed=False,motor_installation_changed=False,service_paths_executed=0,
        native={},attachment_queries=[],arm_motion=[],reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,OCC_pool_threads=1)
    def save(): n.a.save(out,raw)
    def run(label,left,right):
        raw['native'][label]=n.c.material(left,right)
        save()
        print(label,raw['native'][label]['exact_pairs'],len(raw['native'][label]['overlaps']),flush=True)
    def internal(label,objects):
        raw['native'][label]=d.k.flat([n.c.material([z],objects[i+1:]) for i,z in enumerate(objects)])
        save()
    changed=new+moved
    save()
    run('changed20__fixed851_neutral',changed,fixed)
    run('new4__moved16',new,moved)
    internal('new4_internal',new)
    run('new_cords12__hardware871',cords,changed+fixed)
    internal('new_cords12_internal',cords)
    old={z['name']:z['shape'] for z in rows}
    fresh={z['name']:z['shape'] for z in new}
    def query(label,kind,left,right):
        got=left.cut(right,tol=1e-6) if kind=='cut' else left.intersect(right,tol=1e-6)
        assert got.isValid()
        raw['attachment_queries'].append(dict(id=label,operation=kind,volume_mm3=got.Volume() if got.Solids() else 0.,solids=len(got.Solids())))
        save()
    query('original_electrical_base__outside_integrated_base','cut',old['thumb_ip_actuator_wire_guide_base'],fresh[a.FRAME])
    survey=n.p.read(ROOT/'cad/motor_end_integration_basis_v1/reviews/survey.json')
    G=np.array(survey['electrical_frame_proposal'])
    groove=n.g.original.transform(a.a.box([-3.,-3.75,-69.75],[3.,3.75,-67.35]),G)
    query('integrated_base__protected_electrical_groove','common',fresh[a.FRAME],groove)
    for z in [-13.,13.]:
        pad=a.a.cy(3.,-127.,-126.,71.,z).cut(a.a.cy(1.7,-127.01,-125.99,71.,z))
        query('original_M3_pad_'+str(z)+'__outside_integrated_base','cut',pad,fresh[a.FRAME])
    # Preserve the original neighboring outer M3 seat, not only its hole centre.
    R=d.around((0,0,0),(0,1,0),-45.)
    pad=n.g.original.transform(a.a.box([66.5,-162.,-17.],[75.4,-159.,-9.]),R)
    pad=pad.intersect(old['arm054_eight_bank_carrier'],tol=1e-6)
    query('original_middle_dip_M3_seat__outside_dogleg_carrier','cut',pad,fresh[a.CARRIER])
    output=[z for z in fixed if z['name'] in a.a.CLOCK_NAMES|{'thumb_ip_stock_output_horn','thumb_ip_stock_central_retainer'}]
    assert len(output)==9
    raw['output_command_checks']=[]
    for state in prior['motor_command_states']:
        C=np.array(state['transform'])
        moving=[dict(z,shape=n.g.original.transform(z['shape'],C)) for z in output]
        label='command_'+str(int(state['command_deg']))+'__output9__changed20'
        run(label,moving,changed)
        raw['output_command_checks'].append(dict(command_deg=state['command_deg'],transform=C,scope='actual9 output parts vs new20 only;no old factory reference or wholelongcord replay'))
    arm=n.p.read(ROOT/'cad/modular_roots_v9/design.json')['arm']
    groups={key:[z for z in fixed if membership.get(z['name'],'forearm')==key] for key in ['fixed','elbow','forearm','wrist','hand']}
    for state in prior['arm_motion']:
        F=d.transforms(state['angles_deg'],arm)
        moving=[dict(z,shape=n.g.original.transform(z['shape'],F['forearm'])) for z in changed+cords]
        for target in state['compared_frames']:
            obstacle=[dict(z,shape=n.g.original.transform(z['shape'],F[target])) for z in groups[target]]
            run(state['id']+'__new20_local12__'+target,moving,obstacle)
        raw['arm_motion'].append(dict(id=state['id'],angles_deg=state['angles_deg'],compared_frames=state['compared_frames'],transforms=F))
        save()
    totals=d.k.flat(list(raw['native'].values()))
    old_records=[dict(group=k,record=z) for k,t in prior['native'].items() for z in t['overlaps']
        if not ('command_' in k or 'cord' in k or '__new26_and_local12__' in k)]
    affected=a.OLD|set(design['moved_names'])
    raw.update(inherited_unchanged_ARM057_records=[z for z in old_records if not({z['record']['lead'],z['record']['part']}&affected)],
        changed_ARM057_records_not_carried=[z for z in old_records if {z['record']['lead'],z['record']['part']}&affected],
        earlier_unchanged11_records_from_ARM057=prior['inherited_unchanged_records'],inherited_native_retested=False,
        completed=True,native_groups=len(raw['native']),pairs_in_scope=totals['pairs_in_scope'],
        AABB_native_pairs=totals['exact_pairs'],overlap_records=len(totals['overlaps']),operation_failures=len(totals['operation_failures']),
        raw_changed_geometry_clear=not totals['overlaps'] and not totals['operation_failures'],full_proposal_raw_clear=False,
        input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()),elapsed_s=time.monotonic()-started)
    save()


if __name__=='__main__': main()
