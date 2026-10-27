"""Finish only the missing port/common evidence; keep three completed views."""
import importlib.util
import sys
from pathlib import Path

sys.dont_write_bytecode = True
spec = importlib.util.spec_from_file_location('_arm053_partial_post', Path(__file__).with_name('postprocess.py'))
q = importlib.util.module_from_spec(spec)
spec.loader.exec_module(q)
n, np, ROOT, HERE, d, r, engine = q.n, q.np, q.ROOT, q.HERE, q.d, q.r, q.engine


def main():
    out = HERE/'reviews/evidence.json'
    assert not out.exists()
    raw = n.p.read(HERE/'reviews/check.json')
    source = dict(raw['source_sha256'])
    for f in [Path(__file__), HERE/'postprocess.py', HERE/'reviews/check.json']:
        source[f.relative_to(ROOT).as_posix()] = r.sha(f)
    original_images = {f.relative_to(ROOT).as_posix(): r.sha(f) for f in (HERE/'images').glob('*.png')}
    assert len(original_images) == 3
    _, base, _ = q.v.metadata()
    previous = n.p.read(ROOT/'cad/compact_turnback_pack_v1/reviews/check.json')
    replacement = n.p.read(ROOT/previous['scene'])['parts']
    current = [z for z in base if z['name'] not in previous['replaced_hardware']]+replacement
    membership = {z['name']: key for key, rows in n.p.read(ROOT/'cad/arm_packaging_audit_v1/reviews/audit.json')['frame_members'].items() for z in rows}
    new = n.p.read(ROOT/raw['scene'])['parts']
    proposed = [z for z in current if z['name'] != 'Forearm_twenty_axis_module_rack']+new
    delta = {name: np.array(z['transform_from_current']) for z in raw['placements'] for name in z['rows']}
    thumb = next(z for z in raw['placements'] if z['module'] == 'thumb_ip_actuator')
    T = np.array(thumb['transform_from_current'])
    ports = []
    for i in [0, 1]:
        old = next(z for z in previous['design'] if z['id'] == 'motor_'+str(i))
        hand = next(z for z in previous['design'] if z['id'] == 'saddle_'+str(i))
        point = (T@np.r_[old['free_port_world_mm'], 1.])[:3]
        tangent = T[:3, :3]@np.array(old['free_tangent_toward_hand'])
        displacement = np.array(hand['free_port_world_mm'])-point
        ports.append(dict(line=i, motor_command_angle_deg=0.,
            actual_transformed_motor_free_port_mm=point, actual_transformed_free_tangent=tangent,
            unchanged_hand_guide_rear_port_mm=hand['free_port_world_mm'], direct_vector_to_hand_port_mm=displacement,
            port_tangent_dot_direct_hand_direction=float(np.dot(tangent, displacement/np.linalg.norm(displacement))),
            whole_housing_cord_path_or_length_solved=False))
    candidates = [(label, z) for label, test in raw['native'].items() for z in test['overlaps']]
    candidates += [('modules__'+z['left']+'__'+z['right'], record) for z in raw['inter_module_groups'] for record in z['native']['overlaps']]
    label, record = max(candidates, key=lambda z: z[1]['material_intersection_mm3'])
    lookup = {z['name']: z for z in proposed}
    selected = [dict(lookup[record[k]]) for k in ['lead', 'part']]
    for z in selected:
        if 'step' not in z:
            assert z['name'] in d.ARM_FRAME
            rel = 'cad/modular_roots_v9/parts/'+z['name']+'.step'
            assert raw['source_sha256'][rel] == r.sha(ROOT/rel)
            z.update(step=rel, sha256_step=r.sha(ROOT/rel))
    loaded = n.c.load(selected, source)
    shapes = [n.g.original.transform(z['shape'], delta.get(z['name'], np.eye(4))) for z in loaded]
    arm = n.p.read(ROOT/'cad/modular_roots_v9/design.json')['arm']
    neutral = d.transforms([0, 0, 0, 0], arm)
    posed_T = neutral
    if '__changed420__' in label:
        state = next(z for z in raw['motion'] if label.startswith(z['id']+'__'))
        posed_T = state['transforms']
        shapes = [n.g.original.transform(shape, np.array(posed_T[membership.get(z['name'], 'forearm')]))
                  for shape, z in zip(shapes, loaded)]
    common = shapes[0].intersect(shapes[1], tol=1e-6)
    common_scene = n.a.a.prior.export([dict(name='arm053_actual_common', shape=common,
        body='forearm', evidence_only=True, installed_part_candidate=False, color=[.97, .04, .03])], HERE/'generated/actual_common')
    common_meta = n.p.read(ROOT/common_scene)['parts'][0]
    def part(z, M, color):
        path = ROOT/z['mesh']
        assert r.sha(path) == z['sha256_stl']
        source[z['mesh']] = z['sha256_stl']
        return dict(name=z['name'], body='palm', mesh_path=path, points=r.read_stl_points(path),
                    neutral_inverse=M, color=color)
    red_part = part(common_meta, np.eye(4), [.97, .04, .03])
    red = dict(parts=[red_part], design=n.g.DESIGN)
    joined = dict(parts=[part(z, np.array(posed_T[membership.get(z['name'], 'forearm')])@
        delta.get(z['name'], np.eye(4)), q.COLORS[membership.get(z['name'], 'forearm')]) for z in selected]+[red_part], design=n.g.DESIGN)
    def render(a, view):
        try:
            a['_render_state'] = r.make_renderer(a, {'v': view}, 850, engine)
            a['_render_state']['model'].geom_rgba[:, 3] = 1.
            return r.render_one(a, [0.]*20, 'v', {'v': view}, 850, engine)[0]
        finally:
            if '_render_state' in a:
                a['_render_state']['renderer'].close()
                del a['_render_state']
    one = r.view_fit([joined], [dict(angles_deg=[0.]*20)], np.array([.8, -.6, .5]), 1.)
    zoom = r.view_fit([red], [dict(angles_deg=[0.]*20)], np.array([.8, -.6, .5]), 1.)
    r.sheet([('실제 두 부품 전체 / 빨강 공통재료', render(joined, one)),
             ('같은 공통재료만 확대 / 별도 배율', render(red, zoom))], 2, 850,
        '별도 후보의 실제 최대 겹침', record['lead']+' / '+record['part']+' / '+str(record['material_intersection_mm3'])+' mm³').save(HERE/'images/actual_block.png')
    assert all(r.sha(ROOT/f) == h for f, h in original_images.items())
    n.a.save(out, dict(completed=True, source_sha256=source, transformed_real_ports=ports,
        whole_hardware_counts=[862, 872, 862, 872], all_whole_hardware_opaque=True,
        actual_common=[dict(native_group=label, record=record, scene=common_scene,
            reconstructed_STEP_common_volume_mm3=common.Volume(), bbox_mm=q.p.prior.bounds([dict(shape=common)]))],
        initial_postprocess_completed=False, original_exit_input_guard_recorded=False,
        recovery='Original three views completed; missing step metadata on a native arm reference stopped the later common-pair load before Boolean/export. Read that arm native STEP already in the check input hashes, recompute the unsaved2port records, generate only the missing common pair/picture. Whole views and138native groups not rerun.',
        inherited_three_images_hashes_unchanged=True, new_full_native_groups=0,
        separately_reconstructed_common_pairs=1,
        outputs={f.relative_to(ROOT).as_posix(): r.sha(f) for f in (HERE/'images').glob('*.png')},
        input_hash_guard=all(r.sha(ROOT/f) == h for f, h in source.items()),
        whole_drive_or_exterior_approved=False))
    print('ports', [(z['actual_transformed_motor_free_port_mm'].tolist(), z['actual_transformed_free_tangent'].tolist()) for z in ports], flush=True)
    print('actual common', label, record, common.Volume(), flush=True)


if __name__ == '__main__':
    main()
