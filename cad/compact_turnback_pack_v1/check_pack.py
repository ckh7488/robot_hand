"""First native gate for one real packaging candidate, stopping on blockage."""
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import pack_model as a
n,k,np,ROOT,HERE=a.n,a.k,a.np,a.ROOT,a.HERE
from OCP.OSD import OSD_ThreadPool


def main():
    out=HERE/'reviews/check.json'
    assert not out.exists()
    start=time.monotonic();rows,design,source=a.build()
    ctx=k.context();source.update(ctx[0][0]);removed={z['replaces'] for z in rows}
    assert removed=={'motor_0_jaw_with_fixed_bend','motor_1_jaw_with_fixed_bend','wrist_paired_rear_S_guide'}
    for filename in ['pack_model.py','check_pack.py']:
        source[(HERE/filename).relative_to(ROOT).as_posix()]=n.m.drive.sha(HERE/filename)
    report=dict(completed=False,source_sha256=source,design=design,states=[],replaced_hardware=sorted(removed),
        replacement_hardware_rows=3,added_hardware_rows=0,hardware_rows_after_replacement=862,
        hand_rows=397,base_hand_rows=364,packaging_rows=385,electrical_rows=64,arm_rows=10,guide_hardware_rows=6,
        guide_OD_bore_mm=[5.,3.2],housing_OD_ID_mm=[3.,1.3],cord_OD_mm=.6,
        hand_proportions_axes_lengths_motor_rack_electrical_positions_unchanged=True,
        whole_flexible_housing_and_cords_connected=False,new_material_lengths_defined=False,
        old_MOTION047_lengths_preserved=True,old_motion_or_seam_tests_reexecuted=0,
        assembly_plan='Separate conditional service stage with whole housings/cords absent. Each motor starts without its own six clamp parts, other clamp complete; approach lower jaw along -Qz14/7/2/0 then opposite jaw +Qz, nuts -Qz then bolts +Qz. Saddle body/cap/two bolts/two nuts absent, all hand/motors/wires/arm remain: body Y-20/-10/-5/-1/0, cap Y+12/+6/+3/+1/0, bolts +Y20..0, nuts -Y20..0. Loose parts manually held. These are proposed paths, not inherited successes.',
        assembly_paths_executed=0,material_or_wrist_motion_checks_executed=0,
        conditional_support_strength_preload_friction_tension_equilibrium_or_tools_approved=False,
        packaging_exterior_accepted=False,whole_hand_drive_approved=False,reporting_floor_mm3=.001,
        boolean_tolerance_mm=1e-6,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads())
    def save():n.a.save(out,report)
    report['scene']=a.export(rows,'hardware');save()
    parent=np.array(n.p.read(ROOT/'cad/coupled_material_continuation_v1/reviews/analysis.json')['target_parent_angles_deg'])
    cases=[dict(id='neutral',case=n.case_at(np.zeros(20))),dict(id='parent100',case=n.case_at(parent)),
        dict(id='wrist_negative',case=n.case_at(np.zeros(20),-20,-15)),dict(id='wrist_positive',case=n.case_at(np.zeros(20),20,15))]
    report['planned_cases']=cases;save()
    for entry in cases:
        groups=[(key,[z for z in obs if z['name'] not in removed]) for key,obs in k.pose(ctx,entry['case'])]
        assert sum(len(obs) for _,obs in groups)==859
        moving=a.posed(rows,entry['case']);state=dict(**entry,native={},tested_obstacle_names={key:[z['name'] for z in obs] for key,obs in groups})
        report['states'].append(state);save()
        for key,obs in groups:
            state['native'][key]=n.c.material(moving,obs);save()
            print(entry['id'],key,state['native'][key]['overlaps'],flush=True)
        state['native']['internal']=k.flat([n.c.material([z],moving[j+1:]) for j,z in enumerate(moving)])
        state['clear']=all(n.m.drive.clear(z) for z in state['native'].values());save()
        if not state['clear']:
            report['stop_reason']='New rigid guide candidate blocks at the first saved state; no assembly, free-material length selection or motion continuation performed. This is one candidate failure, not a hand defect or proof all compact routes are impossible.'
            break
    tests=[v for state in report['states'] for v in state['native'].values()];agg=k.flat(tests)
    report.update(completed=True,static_cases_executed=len(report['states']),
        candidate_static_clear=len(report['states'])==4 and all(z['clear'] for z in report['states']),
        pairs_in_scope=agg['pairs_in_scope'],AABB_native_pairs=agg['exact_pairs'],overlap_records=len(agg['overlaps']),
        operation_failures=len(agg['operation_failures']),input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()),
        elapsed_s=time.monotonic()-start)
    save();print('summary',report['static_cases_executed'],report['candidate_static_clear'],report['overlap_records'],flush=True)


if __name__=='__main__':main()
