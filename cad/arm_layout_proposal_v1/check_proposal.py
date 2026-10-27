"""Finite native checks of changed relationships in the separate proposal."""
import sys
import time
from itertools import combinations
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))
import proposal as p

n, np, d, ROOT, HERE = p.n, p.np, p.d, p.ROOT, p.HERE
from OCP.OSD import OSD_ThreadPool
from OCP.BRepExtrema import BRepExtrema_DistShapeShape


def main():
    out = HERE/'reviews/proposal.json'
    assert not out.exists()
    started = time.monotonic()
    frames, source = d.inputs()
    moved, new, unchanged, contract, old_rack = p.assemble(frames)
    for rel in ['cad/arm_layout_proposal_v1/proposal.py',
                'cad/arm_layout_proposal_v1/check_proposal.py',
                'cad/arm_layout_proposal_v1/module_survey.py',
                'cad/arm_layout_proposal_v1/reviews/module_survey_corrected.json',
                'cad/modular_roots_v9/design.json']:
        source[rel] = n.m.drive.sha(ROOT/rel)
    assert OSD_ThreadPool.DefaultPool_s().NbThreads() == 1
    # Only the eleven new shapes are exported. Unchanged409 shapes use exact rigid placement metadata.
    scene = n.a.a.prior.export(new, HERE/'generated/carrier')
    report = dict(completed=False, source_sha256=source, placements=contract,
        current_installed_candidate_replaced=False, separate_proposal_only=True,
        old_hardware_rows=862, retained_current_rows=861, moved_rigid_rows=409,
        unchanged_rows=452, omitted_old_rack_only=old_rack['name'], new_hardware_rows=11,
        proposed_total_hardware_rows=872, remote_motors=16, original_motor_shapes_scaled=False,
        installation_axis_rotations_changed=0, motor_command_angles_changed=0,
        scene=scene, native={}, motion=[], attachment=[],
        original_remote_bbox_mm=p.bounds([z for z in frames['forearm'] if z['source_group'] != 'arm']),
        proposed_remote_bbox_mm=p.bounds(moved+new),
        carrier_beam_inner_dimensions_mm=[44., 42.], collar_stations_y_mm=p.STATIONS,
        reaction_path='Existing motor adapter flange/two M3 per module -> new shelf rails/crossbars -> four integrated side posts -> two actual split44x42 beam collars/four nominal M3x12 and nuts -> current Forearm_box_beam -> existing Roll_output_flange. Nominal contact only; axial holding and shear capacity rely on unverified preload/friction.',
        assembly_tools_threads_strength_preload_friction_not_verified=True,
        long_housing_cords_or_new_harness_present=False,
        moved64_electrical_rows_are_existing_local_shapes_not_verified_dynamic_or_mated_harness=True,
        within_each_rigid_module_pairs_retested=False,
        unchanged_arm_or_hand_pairs_retested=False,
        inherited_elbow110_existing_carrier_cradle_obstruction_not_resolved=True,
        reporting_floor_mm3=.001, boolean_tolerance_mm=1e-6, OCC_pool_threads=1,
        whole_arm_drive_manufacturing_or_exterior_approved=False)
    def save():
        n.a.save(out, report)
    save()
    def run(label, left, right):
        result = n.c.material(left, right)
        report['native'][label] = result
        save()
        print(label, 'pairs', result['pairs_in_scope'], 'AABB', result['exact_pairs'],
              'overlaps', len(result['overlaps']), flush=True)
    all_unchanged = [z for rows in unchanged.values() for z in rows]
    run('moved409__unchanged452_neutral', moved, all_unchanged)
    run('new11__all_retained861_neutral', new, moved+all_unchanged)
    for i, z in enumerate(new[:-1]):
        run('new_internal_'+str(i), [z], new[i+1:])
    modules = {z['module']: [] for z in contract}
    for z in moved:
        modules[z['proposal_module']].append(z)
    # Aggregate all120 inter-module groups without hiding any positive native result.
    tests = []
    for left, right in combinations(modules, 2):
        test = n.c.material(modules[left], modules[right])
        tests.append(dict(left=left, right=right, native=test))
    report['inter_module_groups'] = tests
    save()
    arm = n.p.read(ROOT/'cad/modular_roots_v9/design.json')['arm']
    selection = [('roll_40', [0, 40, 0, 0], ['fixed', 'elbow']),
                 ('flex_-35', [0, 0, -35, 0], ['wrist', 'hand']),
                 ('deviation_20', [0, 0, 0, 20], ['hand']),
                 ('elbow_110', [110, 0, 0, 0], ['fixed'])]
    for name, angles, target_frames in selection:
        transforms = d.transforms(angles, arm)
        left = [dict(z, shape=n.g.original.transform(z['shape'], transforms['forearm'])) for z in moved+new]
        for frame in target_frames:
            right = [dict(z, shape=n.g.original.transform(z['shape'], transforms[frame])) for z in unchanged[frame]]
            run(name+'__changed420__'+frame, left, right)
        report['motion'].append(dict(id=name, angles_deg=angles, transforms=transforms,
            compared_frames=target_frames, independent_finite_pose_not_continuous=True))
        save()
    beam = next(z for z in unchanged['forearm'] if z['name'] == 'Forearm_box_beam')['shape']
    for row in new[:3]:
        query = BRepExtrema_DistShapeShape(row['shape'].wrapped, beam.wrapped)
        query.SetMultiThread(False)
        query.Perform()
        assert query.IsDone()
        report['attachment'].append(dict(name=row['name'], actual_native_distance_to_beam_mm=query.Value(),
            positive_volume_test='new11__all_retained861_neutral', normal_contact_not_axial_friction_approval=True))
    alltests = list(report['native'].values())+[z['native'] for z in tests]
    agg = d.k.flat(alltests)
    report.update(completed=True, native_groups=len(alltests), inter_module_group_count=len(tests),
        pairs_in_scope=agg['pairs_in_scope'], AABB_native_pairs=agg['exact_pairs'],
        overlap_records=len(agg['overlaps']), operation_failures=len(agg['operation_failures']),
        raw_executed_candidate_clear=not agg['overlaps'] and not agg['operation_failures'],
        input_hash_guard=all(n.m.drive.sha(ROOT/f) == h for f, h in source.items()),
        elapsed_s=time.monotonic()-started)
    save()
    print('completed', report['native_groups'], report['overlap_records'], flush=True)


if __name__ == '__main__':
    main()
