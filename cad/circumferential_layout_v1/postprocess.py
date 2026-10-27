"""Actual full CAD views, one common-material witness and transformed real ports."""
import importlib.util
import sys
from pathlib import Path

sys.dont_write_bytecode = True
spec = importlib.util.spec_from_file_location('_arm053_post_model', Path(__file__).with_name('model.py'))
p = importlib.util.module_from_spec(spec)
spec.loader.exec_module(p)
n, np, ROOT, HERE, d = p.n, p.np, p.ROOT, p.HERE, p.d
v = n.m.drive.module('_arm053_saved_renderer', ROOT/'cad/drive_packaging_review_v1/inspect_render.py')
r, engine = v.r, v.engine
COLORS = dict(fixed=[.27, .43, .36], elbow=[.45, .65, .45],
              forearm=[.28, .43, .58], wrist=[.89, .49, .17], hand=[.64, .69, .73])


def main():
    out = HERE/'reviews/evidence.json'
    assert not out.exists()
    raw = n.p.read(HERE/'reviews/check.json')
    assert raw['completed']
    source = {'cad/circumferential_layout_v1/postprocess.py': r.sha(Path(__file__)),
        'cad/circumferential_layout_v1/reviews/check.json': r.sha(HERE/'reviews/check.json')}
    _, base, _ = v.metadata()
    prior = n.p.read(ROOT/'cad/compact_turnback_pack_v1/reviews/check.json')
    replacement = n.p.read(ROOT/prior['scene'])['parts']
    current = [z for z in base if z['name'] not in prior['replaced_hardware']]+replacement
    original = n.p.read(ROOT/'cad/arm_packaging_audit_v1/reviews/audit.json')
    membership = {z['name']: key for key, rows in original['frame_members'].items() for z in rows}
    new = n.p.read(ROOT/raw['scene'])['parts']
    proposed = [z for z in current if z['name'] != 'Forearm_twenty_axis_module_rack']+new
    assert len(current) == 862 and len(proposed) == 872
    delta = {name: np.array(z['transform_from_current']) for z in raw['placements'] for name in z['rows']}
    assert len(delta) == 409
    arm = n.p.read(ROOT/'cad/modular_roots_v9/design.json')['arm']
    neutral = d.transforms([0, 0, 0, 0], arm)
    flex = d.transforms([0, 0, -35, 0], arm)
    def assembly(rows, T, changed=False, red=False):
        parts = []
        for z in rows:
            frame = membership.get(z['name'], 'forearm')
            M = np.array(T[frame])@(delta.get(z['name'], np.eye(4)) if changed else np.eye(4))
            path = ROOT/z['mesh']
            assert r.sha(path) == z['sha256_stl']
            source[z['mesh']] = z['sha256_stl']
            parts.append(dict(name=z['name'], body='palm', mesh_path=path,
                points=r.read_stl_points(path), color=[.97, .04, .03] if red or z.get('evidence_only') else COLORS[frame],
                neutral_inverse=M))
        return dict(parts=parts, design=n.g.DESIGN)
    def render(a, view, size):
        try:
            a['_render_state'] = r.make_renderer(a, {'v': view}, size, engine)
            a['_render_state']['model'].geom_rgba[:, 3] = 1.
            return r.render_one(a, [0.]*20, 'v', {'v': view}, size, engine)[0]
        finally:
            if '_render_state' in a:
                a['_render_state']['renderer'].close()
                del a['_render_state']
    images = HERE/'images'
    images.mkdir(exist_ok=True)
    whole = [assembly(current, neutral), assembly(proposed, neutral, True),
             assembly(current, flex), assembly(proposed, flex, True)]
    view = r.view_fit(whole, [dict(angles_deg=[0.]*20)], np.array([.8, -.55, .4]), 1.)
    labels = ['현재 중립 /862개 전부', '별도 전완4방향 배치 /872개 전부',
              '현재 손목-35° /862개 전부', '별도 후보 손목-35° /872개 전부']
    r.sheet([(label, render(a, view, 850)) for label, a in zip(labels, whole)], 2, 850,
        '같은16모터와 모든 부속품: 실제 설치축을 돌린 별도 후보',
        '긴 피복·줄 없음 / 모터명령각0 유지 / 새 설치안 채택·전체 구동·외형 개선 승인 아님').save(images/'layout_before_after.png')
    view2 = r.view_fit(whole[:2], [dict(angles_deg=[0.]*20)], np.array([.05, -.25, 1.]), 1.)
    r.sheet([(label, render(a, view2, 900)) for label, a in zip(labels[:2], whole[:2])], 2, 900,
        '전체 하드웨어의 같은 배율 폭 비교',
        '대표 엄지의 두 큰 굽힘관도 그대로 표시. 모터 본체만 남겨 작게 보이게 하지 않음.').save(images/'width_comparison.png')
    beam = [z for z in current if z['name'] == 'Forearm_box_beam']
    detail = assembly(beam+new, neutral, True)
    local = r.view_fit([detail], [dict(angles_deg=[0.]*20)], np.array([.8, .45, .6]), 1.)
    r.sheet([('새 cage/두캡/체결품11 + 실제 빔1', render(detail, local, 1000))], 1, 1000,
        '실제 빔에 닿는 후보 고정부 상세, 모터·손 생략',
        '명목 반력 경로만. 실제 예압·마찰·강도·조립 미검증이며 전체 외형 그림이 아님.').save(images/'attachment_detail.png')
    # Real guide ports move with the representative motor, not with its command angle.
    thumb = next(z for z in raw['placements'] if z['module'] == 'thumb_ip_actuator')
    T = np.array(thumb['transform_from_current'])
    ports = []
    for i in [0, 1]:
        old = next(z for z in prior['design'] if z['id'] == 'motor_'+str(i))
        hand = next(z for z in prior['design'] if z['id'] == 'saddle_'+str(i))
        point = (T@np.r_[old['free_port_world_mm'], 1.])[:3]
        tangent = T[:3, :3]@np.array(old['free_tangent_toward_hand'])
        displacement = np.array(hand['free_port_world_mm'])-point
        ports.append(dict(line=i, motor_command_angle_deg=0.,
            actual_transformed_motor_free_port_mm=point, actual_transformed_free_tangent=tangent,
            unchanged_hand_guide_rear_port_mm=hand['free_port_world_mm'],
            direct_vector_to_hand_port_mm=displacement,
            port_tangent_dot_direct_hand_direction=float(np.dot(tangent, displacement/np.linalg.norm(displacement))),
            whole_housing_cord_path_or_length_solved=False))
    candidates = [(label, z) for label, test in raw['native'].items() for z in test['overlaps']]
    candidates += [('modules__'+z['left']+'__'+z['right'], record) for z in raw['inter_module_groups'] for record in z['native']['overlaps']]
    actual = []
    if candidates:
        label, record = max(candidates, key=lambda x: x[1]['material_intersection_mm3'])
        lookup = {z['name']: z for z in proposed}
        selected = [lookup[record[k]] for k in ['lead', 'part']]
        loaded = n.c.load(selected, source)
        shapes = [n.g.original.transform(z['shape'], delta.get(z['name'], np.eye(4))) for z in loaded]
        if label in raw['native'] and '__changed420__' in label:
            state = next(z for z in raw['motion'] if label.startswith(z['id']+'__'))
            shapes = [n.g.original.transform(shape, np.array(state['transforms'][membership.get(z['name'], 'forearm')]))
                      for shape, z in zip(shapes, loaded)]
            posed_T = state['transforms']
        else:
            posed_T = neutral
        common = shapes[0].intersect(shapes[1], tol=1e-6)
        common_scene = n.a.a.prior.export([dict(name='arm053_actual_common', shape=common,
            body='forearm', evidence_only=True, installed_part_candidate=False, color=[.97, .04, .03])], HERE/'generated/actual_common')
        common_meta = n.p.read(ROOT/common_scene)['parts']
        joined = assembly(selected, posed_T, True)
        red = assembly(common_meta, neutral, red=True)
        joined['parts'] += red['parts']
        one = r.view_fit([joined], [dict(angles_deg=[0.]*20)], np.array([.8, -.6, .5]), 1.)
        zoom = r.view_fit([red], [dict(angles_deg=[0.]*20)], np.array([.8, -.6, .5]), 1.)
        r.sheet([('실제 두 부품 전체 / 빨강 공통재료', render(joined, one, 850)),
                 ('같은 공통재료만 확대 / 별도 배율', render(red, zoom, 850))], 2, 850,
            '별도 후보의 실제 최대 겹침',
            record['lead']+' / '+record['part']+' / '+str(record['material_intersection_mm3'])+' mm³').save(images/'actual_block.png')
        actual.append(dict(native_group=label, record=record, scene=common_scene,
            reconstructed_STEP_common_volume_mm3=common.Volume(), bbox_mm=p.prior.bounds([dict(shape=common)])))
    n.a.save(out, dict(completed=True, source_sha256=source, transformed_real_ports=ports,
        whole_hardware_counts=[862, 872, 862, 872], all_whole_hardware_opaque=True,
        source_axis_change_not_motor_command_motion=True, actual_common=actual,
        new_full_native_groups=0, separately_reconstructed_common_pairs=len(actual),
        outputs={f.relative_to(ROOT).as_posix(): r.sha(f) for f in images.glob('*.png')},
        input_hash_guard=all(r.sha(ROOT/f) == h for f, h in source.items()),
        whole_drive_or_exterior_approved=False))


if __name__ == '__main__':
    main()
