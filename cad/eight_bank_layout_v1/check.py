"""Native checks of this separate changed-axis topology, not prior-layout replay."""
import sys
import time
import importlib.util
from itertools import combinations
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))
spec = importlib.util.spec_from_file_location('_arm054_layout_model', Path(__file__).with_name('model.py'))
p = importlib.util.module_from_spec(spec)
spec.loader.exec_module(p)

n, np, d, ROOT, HERE = p.n, p.np, p.d, p.ROOT, p.HERE
from OCP.OSD import OSD_ThreadPool
from OCP.BRepExtrema import BRepExtrema_DistShapeShape


def main():
    out = HERE/'reviews/check.json'
    assert not out.exists()
    started = time.monotonic()
    frames, source = d.inputs()
    print('building separate eight-bank carrier', flush=True)
    moved, new, unchanged, contract = p.assemble(frames)
    for rel in ['cad/eight_bank_layout_v1/model.py', 'cad/eight_bank_layout_v1/check.py',
                'cad/arm_layout_proposal_v1/proposal.py', 'cad/arm_layout_proposal_v1/module_survey.py',
                'cad/arm_layout_proposal_v1/reviews/module_survey_corrected.json',
                'cad/modular_roots_v9/design.json']:
        source[rel] = n.m.drive.sha(ROOT/rel)
    assert OSD_ThreadPool.DefaultPool_s().NbThreads() == 1
    scene = n.a.a.prior.export(new, HERE/'generated/cage')
    raw = dict(completed=False, source_sha256=source, scene=scene, placements=contract,
        current862_installation_replaced=False, separate_proposal_rows=872,
        original_rack_removed_only_in_separate_proposal=True, new_hardware_rows=11,
        retained_moved_rows=409, unchanged_rows=452, remote_motors=16,
        installation_axes_rotated=16, commanded_motor_angles_changed=0,
        canonical_motor_output_axis='+X', proposed_motor_output_axis='15 +Y, thumbIP -Y',
        nominal_radius_mm=p.RADIUS,
        radius_basis='Chosen R55 with normal packet canonicalZ pointing inward-negative, so motor outer Z+10 lies at radius41.1 and electrical base/leads face outward. Actual whole-packet native checks, not cylinder/AABB-only acceptance.',
        carrier_basis='Two inner hubsR34..38, two M3-seat spokes per module, two lower longitudinal rails, split44x42 beam collars/caps. ThumbIP entire packet and its two seats reverse together; front M3 seatY-121..-118 connects by two longitudinal bridges to front hubY-162..-159.',
        reaction_path='Actual adapter flange/two M3 -> spoke seats -> hubs/lower rails -> two split collars/caps/four M3/four nuts -> actual beam -> Roll_output_flange. Nominal contact only; holding friction/preload/strength/assembly unverified.',
        long_housing_cords_present=False, existing64_electrical_shapes_moved_not_dynamic_harness_approval=True,
        same_module_internal_pairs_retested=False, unchanged_arm_hand_internal_pairs_retested=False,
        inherited_elbow110_bench_structure_obstruction_resolved=False,
        raw_executed_clear=False, adopted=False, whole_arm_drive_or_exterior_approved=False,
        native={}, inter_module_groups=[], motion=[], contact_distances=[],
        original_remote_bbox_mm=p.prior.prior.bounds([z for z in frames['forearm'] if z['source_group'] != 'arm']),
        proposed_remote_bbox_mm=p.prior.prior.bounds(moved+new),
        reporting_floor_mm3=.001, boolean_tolerance_mm=1e-6, OCC_pool_threads=1)
    def save():
        n.a.save(out, raw)
    save()
    def run(label, left, right):
        test = n.c.material(left, right)
        raw['native'][label] = test
        save()
        print(label, test['exact_pairs'], len(test['overlaps']), flush=True)
    all_unchanged = [z for rows in unchanged.values() for z in rows]
    run('moved409__unchanged452_neutral', moved, all_unchanged)
    run('new11__retained861_neutral', new, moved+all_unchanged)
    for i, row in enumerate(new[:-1]):
        run('new_internal_'+str(i), [row], new[i+1:])
    groups = {z['module']: [] for z in contract}
    for row in moved:
        groups[row['proposal_module']].append(row)
    for left, right in combinations(groups, 2):
        test = n.c.material(groups[left], groups[right])
        raw['inter_module_groups'].append(dict(left=left, right=right, native=test))
        if test['overlaps'] or test['operation_failures']:
            print('inter_module', left, right, len(test['overlaps']), flush=True)
        save()
    arm = n.p.read(ROOT/'cad/modular_roots_v9/design.json')['arm']
    states = []
    states += [('elbow_'+str(a), [a,0,0,0], ['fixed']) for a in [55,110]]
    states += [('roll_'+str(a), [0,a,0,0], ['fixed','elbow']) for a in [-80,-40,40,80]]
    states += [('flex_'+str(a), [0,0,a,0], ['wrist','hand']) for a in [-35,55]]
    states += [('deviation_'+str(a), [0,0,0,a], ['hand']) for a in [-20,20]]
    states += [('compound_a', [55,40,-20,15], ['fixed','elbow','wrist','hand']),
               ('compound_b', [110,-40,20,-15], ['fixed','elbow','wrist','hand'])]
    for name, angles, targets in states:
        T = d.transforms(angles, arm)
        left = [dict(z, shape=n.g.original.transform(z['shape'], T['forearm'])) for z in moved+new]
        for target in targets:
            right = [dict(z, shape=n.g.original.transform(z['shape'], T[target])) for z in unchanged[target]]
            run(name+'__changed420__'+target, left, right)
        raw['motion'].append(dict(id=name, angles_deg=angles, compared_frames=targets, transforms=T))
        save()
    beam = next(z['shape'] for z in unchanged['forearm'] if z['name'] == 'Forearm_box_beam')
    for row in new:
        if row['name'] != 'arm054_eight_bank_carrier' and not row['name'].startswith('arm054_beam_cap_'):
            continue
        query = BRepExtrema_DistShapeShape(row['shape'].wrapped, beam.wrapped)
        query.SetMultiThread(False)
        query.Perform()
        assert query.IsDone()
        raw['contact_distances'].append(dict(name=row['name'], target='Forearm_box_beam', actual_distance_mm=query.Value()))
    carrier = new[0]['shape']
    for item in contract:
        target = item['module']+'_adapter'
        flange = next(z['shape'] for z in moved if z['name'] == target)
        query = BRepExtrema_DistShapeShape(carrier.wrapped, flange.wrapped)
        query.SetMultiThread(False)
        query.Perform()
        assert query.IsDone()
        raw['contact_distances'].append(dict(name=new[0]['name'], target=target, actual_distance_mm=query.Value()))
    tests = list(raw['native'].values())+[z['native'] for z in raw['inter_module_groups']]
    agg = d.k.flat(tests)
    raw.update(completed=True, native_groups=len(tests), pairs_in_scope=agg['pairs_in_scope'],
        AABB_native_pairs=agg['exact_pairs'], overlap_records=len(agg['overlaps']),
        operation_failures=len(agg['operation_failures']),
        raw_executed_clear=not agg['overlaps'] and not agg['operation_failures'],
        input_hash_guard=all(n.m.drive.sha(ROOT/f) == h for f, h in source.items()),
        elapsed_s=time.monotonic()-started)
    save()
    print('completed', raw['native_groups'], raw['overlap_records'], flush=True)


if __name__ == '__main__':
    main()
