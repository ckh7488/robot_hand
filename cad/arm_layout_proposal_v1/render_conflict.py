"""Render only the saved conflict, without repeating the whole-view renderer."""
import sys
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))
import render_proposal as q

n, np, ROOT, HERE, r, engine = q.n, q.np, q.ROOT, q.HERE, q.r, q.engine


def main():
    out = HERE/'reviews/conflict_evidence.json'
    assert not out.exists()
    raw = n.p.read(HERE/'reviews/proposal.json')
    detail = n.p.read(HERE/'reviews/conflict_detail.json')
    _, metadata, _ = q.v.metadata()
    carrier = next(z for z in n.p.read(ROOT/raw['scene'])['parts'] if z['name'] == 'arm052_short_wide_carrier')
    guide = next(z for z in metadata if z['name'] == detail['moved_part'])
    common = n.p.read(ROOT/detail['actual_common_scene'])['parts'][0]
    source = {'cad/arm_layout_proposal_v1/render_conflict.py': r.sha(Path(__file__)),
        'cad/arm_layout_proposal_v1/reviews/conflict_detail.json': r.sha(HERE/'reviews/conflict_detail.json')}
    def part(row, M, color):
        path = ROOT/row['mesh']
        assert r.sha(path) == row['sha256_stl']
        source[row['mesh']] = row['sha256_stl']
        return dict(name=row['name'], body='palm', mesh_path=path,
            points=r.read_stl_points(path), neutral_inverse=M, color=color)
    red_part = part(common, np.eye(4), [.97, .04, .03])
    red = dict(parts=[red_part], design=n.g.DESIGN)
    joined = dict(parts=[part(carrier, np.eye(4), [.28, .43, .58]),
        part(guide, np.array(detail['moved_part_transform']), [.3, .75, .6]), red_part], design=n.g.DESIGN)
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
    image = HERE/'images/carrier_actual_block.png'
    r.sheet([('新 받침과 전선가이드 전체 / 빨강은 공통재료', render(joined, one)),
             ('같은 공통재료만 확대 / 별도 배율', render(red, zoom))], 2, 850,
        '미채택 근거: 새 받침이 실제 기존 전선가이드에 걸림',
        str(detail['actual_common_volume_mm3'])+' mm³ / 고장 상세이며 전체 외형 판정 그림이 아님.').save(image)
    n.a.save(out, dict(completed=True, source_sha256=source,
        output=image.relative_to(ROOT).as_posix(), output_sha256=r.sha(image),
        new_native_tests_or_geometry_exports=0, whole_view_rendering_reexecuted=False,
        input_hash_guard=all(r.sha(ROOT/f) == h for f, h in source.items())))


if __name__ == '__main__':
    main()
