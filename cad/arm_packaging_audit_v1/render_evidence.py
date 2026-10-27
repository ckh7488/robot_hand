"""Opaque current hardware poses and actual reported common material."""
import sys
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))
import audit as d

n, k, np, ROOT, HERE = d.n, d.k, d.np, d.ROOT, d.HERE
v = n.m.drive.module('_arm051_saved_renderer', ROOT/'cad/drive_packaging_review_v1/inspect_render.py')
r, engine = v.r, v.engine
COLORS = dict(fixed=[.27, .43, .36], elbow=[.45, .65, .45],
              forearm=[.28, .43, .58], wrist=[.89, .49, .17], hand=[.64, .69, .73])


def main():
    out = HERE/'reviews/evidence.json'
    assert not out.exists()
    raw = n.p.read(HERE/'reviews/audit.json')
    assert raw['completed']
    frames, source = d.inputs()
    source['cad/arm_packaging_audit_v1/reviews/audit.json'] = r.sha(HERE/'reviews/audit.json')
    source['cad/arm_packaging_audit_v1/render_evidence.py'] = r.sha(Path(__file__))
    _, base, _ = v.metadata()
    prior = n.p.read(ROOT/'cad/compact_turnback_pack_v1/reviews/check.json')
    new = n.p.read(ROOT/prior['scene'])['parts']
    meta = [z for z in base if z['name'] not in prior['replaced_hardware']]+new
    membership = {z['name']: frame for frame, rows in frames.items() for z in rows}
    assert len(meta) == 862 and {z['name'] for z in meta} == membership.keys()
    images = HERE/'images'
    images.mkdir(exist_ok=True)

    def make(rows, transforms, common=False):
        parts = []
        for z in rows:
            path = ROOT/z['mesh']
            assert r.sha(path) == z['sha256_stl']
            source[z['mesh']] = z['sha256_stl']
            frame = membership.get(z['name'])
            parts.append(dict(name=z['name'], body='palm', mesh_path=path,
                points=r.read_stl_points(path), color=[.97, .04, .03] if common else COLORS[frame],
                neutral_inverse=np.array(transforms[frame]) if frame else np.eye(4)))
        return dict(parts=parts, design=n.g.DESIGN)

    def render(assembly, view, size):
        try:
            assembly['_render_state'] = r.make_renderer(assembly, {'v': view}, size, engine)
            assembly['_render_state']['model'].geom_rgba[:, 3] = 1.
            return r.render_one(assembly, [0.]*20, 'v', {'v': view}, size, engine)[0]
        finally:
            if '_render_state' in assembly:
                assembly['_render_state']['renderer'].close()
                del assembly['_render_state']

    states = {z['id']: z for z in raw['states']}
    wrist_states = [z for z in raw['states'] if z['id'].startswith(('flex_', 'deviation_'))]
    worst = max(wrist_states, key=lambda z: max(
        [p['pair']['material_intersection_mm3'] for p in z['new_overlap_records']]+[0.]))
    selected = [states['neutral'], states['roll_40'], states['elbow_110'], worst]
    assemblies = [make(meta, z['transforms']) for z in selected]
    view = r.view_fit(assemblies, [dict(angles_deg=[0.]*20)], np.array([.85, -.55, .4]), 1.)
    names = {'neutral': '중립', 'elbow_110': '팔꿈치110°', 'roll_40': '전완 회전+40°',
             'flex_-35': '손목 굽힘-35°', 'flex_55': '손목 굽힘+55°',
             'deviation_-20': '손목 편위-20°', 'deviation_20': '손목 편위+20°'}
    tiles = [(names[z['id']]+' / 하드웨어862개 전부', render(assembly, view, 800))
             for z, assembly in zip(selected, assemblies)]
    r.sheet(tiles, 2, 800, '현재 모터랙과 팔 관절: 조건부 몸체 배정에 따른 실제 CAD',
        '긴 피복·줄 없음 / 회색 손·파랑 전완랙·주황 손목·초록 팔기구 / 유한 진단 자세, 전체 운동·외형 수용 아님').save(images/'arm_poses.png')
    actual = []
    lookup = {z['name']: z for rows in frames.values() for z in rows}
    metadata = {z['name']: z for z in meta}
    # Keep each mechanical issue local instead of painting the whole assembly red.
    for phase, candidates in [('roll', [states['roll_40']]),
                              ('elbow', [states['elbow_110']]), ('wrist', wrist_states)]:
        records = [(state, item) for state in candidates for item in state['new_overlap_records']]
        if not records:
            continue
        state, item = max(records, key=lambda z: z[1]['pair']['material_intersection_mm3'])
        pair = item['pair']
        names_pair = [pair['lead'], pair['part']]
        shapes = [n.g.original.transform(lookup[name]['shape'],
                  np.array(state['transforms'][membership[name]])) for name in names_pair]
        common = shapes[0].intersect(shapes[1], tol=1e-6)
        common_rows = [dict(name='arm051_'+phase+'_actual_common', shape=common,
            body='palm', evidence_only=True, installed_part_candidate=False, color=[.97, .04, .03])]
        scene = n.a.a.prior.export(common_rows, HERE/'generated'/phase)
        common_meta = n.p.read(ROOT/scene)['parts']
        original = make([metadata[name] for name in names_pair], state['transforms'])
        colored = make(common_meta, {}, common=True)
        joined = dict(parts=original['parts']+colored['parts'], design=n.g.DESIGN)
        one = r.view_fit([joined], [dict(angles_deg=[0.]*20)], np.array([.8, -.9, .5]), 1.)
        zoom = r.view_fit([colored], [dict(angles_deg=[0.]*20)], np.array([.8, -.9, .5]), 1.)
        r.sheet([('실제 두 부품 전체 / 빨강은 공통재료', render(joined, one, 850)),
                 ('같은 공통재료만 확대 / 별도 배율', render(colored, zoom, 850))], 2, 850,
            names[state['id']]+'의 실제 겹침',
            pair['lead']+' / '+pair['part']+' / '+str(pair['material_intersection_mm3'])+' mm³').save(images/(phase+'_actual_block.png'))
        actual.append(dict(state_id=state['id'], **item, evidence_scene=scene,
            evidence_common_default_volume_mm3=common.Volume(),
            evidence_common_bbox_mm=n.c.m.pack._bbox(common)))
    n.a.save(out, dict(completed=True, source_sha256=source, rendered_states=[z['id'] for z in selected],
        hardware_rows_each_whole_view=862, every_hardware_opaque=True,
        flexible_housing_or_cords_shown=False, new_native_frame_group_tests=0,
        separately_reconstructed_common_pairs=len(actual), actual_common_pairs=actual,
        outputs={f.relative_to(ROOT).as_posix(): r.sha(f) for f in images.glob('*.png')},
        input_hash_guard=all(r.sha(ROOT/f) == h for f, h in source.items()),
        whole_arm_or_drive_or_exterior_approved=False))


if __name__ == '__main__':
    main()
