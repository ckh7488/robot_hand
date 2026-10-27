"""Bounded exact-commit replay of the architecture diagnostic, including failures."""
import argparse
import subprocess
import sys
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))
import audit as d
import attachment_probe as mount

n, k, np, ROOT, HERE = d.n, d.k, d.np, d.ROOT, d.HERE
from OCP.OSD import OSD_ThreadPool


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--commit', required=True)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    def git(*xs):
        return subprocess.check_output(['git', *xs], cwd=ROOT, text=True,
            encoding='utf-8', creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse', 'HEAD') == args.commit
    assert not git('status', '--porcelain=v1')
    assert not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
    manifest = n.p.read(ROOT/'checkpoints/ARM051.manifest.json')
    frozen = {}
    for key in ['geometry_sha256', 'code_sha256', 'report_sha256']:
        frozen.update(n.p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[key])
    def guard():
        return all(n.m.drive.sha(ROOT/f) == z['sha256'] for f, z in manifest['files'].items()) and all(
            n.m.drive.sha(ROOT/f) == h for f, h in frozen.items())
    assert guard() and OSD_ThreadPool.DefaultPool_s().NbThreads() == 1
    raw = n.p.read(HERE/'reviews/audit.json')
    evidence = n.p.read(HERE/'reviews/evidence.json')
    attachment = n.p.read(HERE/'reviews/attachment.json')
    frames, _ = d.inputs()
    result = dict(completed=False, target_commit=args.commit, differences=[], native_samples=[],
        arm_transform_comparisons=[], body_membership_comparisons=[],
        manifest_files=len(manifest['files']), frozen_files=len(frozen),
        OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),
        hardware_geometry_regenerated=0, new_hardware=0,
        LINK050_restarted_or_replayed=False, independent_audit=False,
        all_62_native_groups_replayed=False, whole_arm_or_drive_or_exterior_approved=False)
    def save():
        n.a.save(args.output, result)
    save()
    for frame in d.FRAME_NAMES:
        names = [dict(name=z['name'], source_group=z['source_group']) for z in frames[frame]]
        ok = names == raw['frame_members'][frame]
        result['body_membership_comparisons'].append(dict(frame=frame, rows=len(names), matched=bool(ok)))
        if not ok:
            result['differences'].append('body_'+frame)
    result['attachment_distances'] = []
    for got, want in zip(mount.probe(frames), attachment['post_faces']):
        err = abs(got['actual_post_face_to_beam_distance_mm']-want['actual_post_face_to_beam_distance_mm'])
        ok = err < 1e-5 and np.max(abs(np.array(got['post_face_center_mm'])-want['post_face_center_mm'])) < 1e-5
        result['attachment_distances'].append(dict(**got, difference_mm=err, matched=bool(ok)))
        if not ok:
            result['differences'].append('attachment_distance')
    fk = n.m.drive.module('_arm051_original_arm_fk', ROOT/'cad/modular_roots_v9/kinematics_numpy.py')
    zero_hand = {z['name']: [0., 0., 0., 0.] for z in fk.C['hand']['fingers']}
    base = fk.forward(zero_hand)
    old_indices = {z['name']: j for j, z in enumerate(fk.META) if z['name'] in d.ARM_FRAME}
    assert len(old_indices) == 10
    states = {z['id']: z for z in raw['states']}
    for state in raw['states']:
        posed = fk.forward(zero_hand, arm_q=state['angles_deg'])
        errors = {name: float(np.max(abs(posed[j]@np.linalg.inv(base[j])-
                  np.array(state['transforms'][d.ARM_FRAME[name]]))))
                  for name, j in old_indices.items()}
        ok = max(errors.values()) < 1e-10
        result['arm_transform_comparisons'].append(dict(id=state['id'], original_arm_rows=10,
            errors=errors, matched=bool(ok)))
        if not ok:
            result['differences'].append('FK_'+state['id'])
    chosen = [('neutral', key) for key in states['neutral']['native']]
    chosen += [(name, 'fixed__elbow') for name in ['elbow_55', 'elbow_110']]
    chosen += [(name, 'fixed__forearm') for name in ['roll_-80', 'roll_40', 'roll_80']]
    chosen += [(name, 'forearm__hand') for name in ['flex_-35', 'flex_55', 'deviation_-20', 'deviation_20']]
    assert len(chosen) == len(set(chosen)) == 19
    result['native_selection_rule'] = 'All10 neutral frame pairs, elbow55/110 fixed-elbow, roll-80/+40/+80 fixed-forearm, wrist-35/+55 and deviation-20/+20 forearm-hand. Includes all133 reported obstruction records but not every original empty group.'
    for state_id, label in chosen:
        state = states[state_id]
        left, right = label.split('__')
        T = state['transforms']
        L = [dict(z, shape=n.g.original.transform(z['shape'], np.array(T[left]))) for z in frames[left]]
        R = [dict(z, shape=n.g.original.transform(z['shape'], np.array(T[right]))) for z in frames[right]]
        got = n.c.material(L, R)
        want = state['native'][label]
        ga, wa = d.overlap_map(got), d.overlap_map(want)
        err = max([abs(ga[key]['material_intersection_mm3']-wa[key]['material_intersection_mm3'])
                   for key in ga.keys() & wa.keys()]+[0.])
        ok = (ga.keys() == wa.keys() and err < 1e-5 and
              got['operation_failures'] == want['operation_failures'] and
              got['pairs_in_scope'] == want['pairs_in_scope'])
        result['native_samples'].append(dict(id=state_id+'__'+label, native=got,
            maximum_basic_volume_difference_mm3=err, matched=bool(ok)))
        if not ok:
            result['differences'].append(state_id+'__'+label)
        save()
        print(state_id, label, bool(ok), flush=True)
    result.update(completed=True, core_passed=not result['differences'],
        native_groups=len(chosen), native_matches=sum(z['matched'] for z in result['native_samples']),
        reported_overlap_records_in_replayed_groups=sum(len(z['native']['overlaps']) for z in result['native_samples']),
        original_arm_FK_states=len(raw['states']), original_arm_FK_rows=10,
        common_evidence_scenes_preserved=len(evidence['actual_common_pairs']),
        common_pair_or_all_curved_surface_Boolean_equivalence_retested=False,
        input_hash_guard=guard(), workspace_clean=not git('status', '--porcelain=v1'))
    save()


if __name__ == '__main__':
    main()
