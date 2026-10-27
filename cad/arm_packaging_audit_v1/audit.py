"""Read-only arm packaging diagnostic with explicit conditional rigid bodies."""
import sys
import time
from itertools import combinations
from pathlib import Path

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/'cad/compact_turnback_pack_v1'))
import pack_model as a

n, k, np = a.n, a.k, a.np
from OCP.OSD import OSD_ThreadPool

FRAME_NAMES = ('fixed', 'elbow', 'forearm', 'wrist', 'hand')
ARM_FRAME = {
    'AK_elbow_flex': 'fixed', 'Elbow_fixed_cradle': 'fixed',
    'Elbow_roll_carrier': 'elbow', 'AK_forearm_roll': 'elbow',
    'Roll_output_flange': 'forearm', 'Forearm_box_beam': 'forearm',
    'AK_wrist_flex': 'forearm',
    'Wrist_box_carrier': 'wrist', 'AK_wrist_deviation': 'wrist',
    'Palm_wrist_saddle': 'hand',
}


def inputs():
    ctx = k.context()
    source = ctx[0][0]
    scene = 'cad/compact_turnback_pack_v1/generated/hardware/scene.json'
    source[scene] = n.m.drive.sha(ROOT/scene)
    replacement = n.c.load(n.p.read(ROOT/scene)['parts'], source)
    removed = {z['replaces'] for z in replacement}
    neutral = n.case_at(np.zeros(20))
    groups = [(key, [z for z in rows if z['name'] not in removed])
              for key, rows in k.pose(ctx, neutral)]
    for z in replacement:
        key = 'packaging' if z['body'] == 'forearm' else 'wrist_guide'
        next(rows for name, rows in groups if name == key).append(z)
    frames = {key: [] for key in FRAME_NAMES}
    for group, rows in groups:
        for z in rows:
            if group == 'arm':
                frame = ARM_FRAME[z['name']]
            elif group == 'packaging':
                frame = 'hand' if z.get('body') == 'palm' else 'forearm'
            else:
                frame = 'forearm' if group == 'electrical' else 'hand'
            frames[frame].append(dict(z, audit_frame=frame, source_group=group))
    assert [len(frames[key]) for key in FRAME_NAMES] == [2, 2, 413, 2, 443]
    assert sum(map(len, frames.values())) == 862
    return frames, source


def around(point, axis, angle):
    v = np.asarray(axis, dtype=float)
    v /= np.linalg.norm(v)
    x, y, z = v
    K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    t = np.deg2rad(angle)
    R = np.eye(3) + np.sin(t)*K + (1-np.cos(t))*(K@K)
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = np.asarray(point)-R@np.asarray(point)
    return M


def transforms(angles, arm):
    elbow, roll, flex, deviation = angles
    wy = -arm['wrist_to_palm']-arm['wrist_axis_offset']
    ey = wy-arm['elbow_to_wrist_flex']
    E = around((0, ey, 0), (1, 0, 0), elbow)
    F = around((0, ey+arm['elbow_to_roll'], 0), (0, 1, 0), roll)
    P = around((0, wy, 0), (1, 0, 0), flex)
    Y = around((arm['wrist_lateral_offset'], -arm['wrist_to_palm'], 0),
               (0, 0, 1), deviation)
    return dict(fixed=np.eye(4), elbow=E, forearm=E@F,
                wrist=E@F@P, hand=E@F@P@Y)


def plan():
    cases = [('neutral', [0, 0, 0, 0])]
    cases += [('elbow_'+str(v), [v, 0, 0, 0]) for v in [55, 110]]
    cases += [('roll_'+str(v), [0, v, 0, 0]) for v in [-80, -40, 40, 80]]
    cases += [('flex_'+str(v), [0, 0, v, 0]) for v in [-35, 55]]
    cases += [('deviation_'+str(v), [0, 0, 0, v]) for v in [-20, 20]]
    return [dict(id=name, angles_deg=values) for name, values in cases]


def overlap_map(test):
    return {(z['lead'], z['part']): z for z in test['overlaps']}


def main():
    out = HERE/'reviews/audit.json'
    assert not out.exists()
    started = time.monotonic()
    frames, source = inputs()
    for rel in ['cad/arm_packaging_audit_v1/audit.py',
                'cad/modular_roots_v9/design.json', 'robot_config.json',
                'cad/palm_motor_layout_v1/selected/generated/layout.json']:
        source[rel] = n.m.drive.sha(ROOT/rel)
    arm = n.p.read(ROOT/'cad/modular_roots_v9/design.json')['arm']
    declared = n.p.read(ROOT/'robot_config.json')['arm']['joints']
    layout = n.p.read(ROOT/'cad/palm_motor_layout_v1/selected/generated/layout.json')
    cases = plan()
    assert OSD_ThreadPool.DefaultPool_s().NbThreads() == 1
    remote = [z for z in layout['modules'] if z['rigid_body'] == 'forearm']
    assert len(remote) == 16
    report = dict(completed=False, source_sha256=source, plan=cases, states=[],
        frame_counts={key: len(rows) for key, rows in frames.items()},
        frame_members={key: [dict(name=z['name'], source_group=z['source_group'])
                             for z in rows] for key, rows in frames.items()},
        existing_arm_dimensions_mm=arm, old_declared_angular_targets=declared,
        angles_are_diagnostic_samples_not_new_limits_or_human_range_approval=True,
        range_source_warning='Only old declared angular targets are reused. Current arm dimensions come from modular_roots_v9/design.json; robot_config arm lengths/hand geometry are not reused.',
        hand_target_angles_deg=[0.]*20, hardware_rows=862,
        conditional_attachment='All16 rear motors, their rack and64 electrical reference rows are assumed attached only to the rotating forearm output E@F. Existing nominal rack beam-face posts atY-148/-190 are distal to roll planeY-230. No validated clamp/bolts/stiffness connects the whole rack to the beam. This is a body assignment hypothesis, not a new CAD mount.',
        motion_frames='fixed:I; elbow:E; forearm:E@F; wrist:E@F@P; hand:E@F@P@Y',
        row_frame_names=['fixed', 'elbow', 'forearm', 'wrist', 'hand'],
        rack_support_basis=layout['supports'], remote_motor_modules=remote,
        hand_proportions_axes_lengths_motor_rack_electrical_neutral_geometry_changed=False,
        hardware_geometry_regenerated_or_exported=0,
        long_housings_cords_present=False, LINK050_restarted=False,
        whole20motor_torque_connections_electrical_dynamic_upper_arm_or_physical_motion_verified=False,
        within_same_frame_pairs_reexecuted=0, full_continuous_motion_or_exterior_approved=False,
        reporting_floor_mm3=.001, boolean_tolerance_mm=1e-6,
        OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads())
    def save():
        n.a.save(out, report)
    save()
    baseline = {}
    for case in cases:
        T = transforms(case['angles_deg'], arm)
        posed = {key: [dict(z, shape=n.g.original.transform(z['shape'], T[key]))
                       for z in rows] if np.max(abs(T[key]-np.eye(4))) > 1e-12 else rows
                 for key, rows in frames.items()}
        state = dict(**case, transforms=T, native={}, invariant_frame_pairs=[],
                     new_overlap_records=[], changed_baseline_overlap_records=[])
        report['states'].append(state)
        save()
        for left, right in combinations(FRAME_NAMES, 2):
            label = left+'__'+right
            relative = np.linalg.inv(T[left])@T[right]
            if case['id'] != 'neutral' and np.max(abs(relative-np.eye(4))) < 1e-12:
                state['invariant_frame_pairs'].append(label)
                continue
            test = n.c.material(posed[left], posed[right])
            state['native'][label] = test
            if case['id'] == 'neutral':
                baseline[label] = overlap_map(test)
            else:
                current = overlap_map(test)
                for key, pair in current.items():
                    if key not in baseline[label]:
                        state['new_overlap_records'].append(dict(frame_pair=label, pair=pair))
                    elif abs(pair['material_intersection_mm3']-
                             baseline[label][key]['material_intersection_mm3']) > 1e-5:
                        state['changed_baseline_overlap_records'].append(dict(
                            frame_pair=label, pair=pair,
                            neutral_pair=baseline[label][key]))
            save()
            print(case['id'], label, 'native', test['exact_pairs'],
                  'records', len(test['overlaps']), flush=True)
        state['completed'] = True
        state['raw_executed_cross_frame_clear'] = all(
            n.m.drive.clear(v) for v in state['native'].values())
        save()
    agg = k.flat([v for state in report['states'] for v in state['native'].values()])
    report.update(completed=True, states_executed=len(report['states']),
        native_frame_pair_groups=sum(len(z['native']) for z in report['states']),
        pairs_in_scope=agg['pairs_in_scope'], AABB_native_pairs=agg['exact_pairs'],
        overlap_records=len(agg['overlaps']), operation_failures=len(agg['operation_failures']),
        new_overlap_records=sum(len(z['new_overlap_records']) for z in report['states']),
        changed_baseline_records=sum(len(z['changed_baseline_overlap_records']) for z in report['states']),
        input_hash_guard=all(n.m.drive.sha(ROOT/f) == h for f, h in source.items()),
        elapsed_s=time.monotonic()-started)
    save()
    print('summary', report['states_executed'], report['new_overlap_records'],
          report['operation_failures'], flush=True)


if __name__ == '__main__':
    main()
