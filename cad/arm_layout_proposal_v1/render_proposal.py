"""Show all hardware in the current and separate proposed arrangements."""
import sys
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))
import proposal as p

n, np, ROOT, HERE, d = p.n, p.np, p.ROOT, p.HERE, p.d
v = n.m.drive.module('_arm052_saved_renderer', ROOT/'cad/drive_packaging_review_v1/inspect_render.py')
r, engine = v.r, v.engine
COLORS = dict(fixed=[.27, .43, .36], elbow=[.45, .65, .45],
              forearm=[.28, .43, .58], wrist=[.89, .49, .17], hand=[.64, .69, .73])


def main():
    out = HERE/'reviews/evidence.json'
    assert not out.exists()
    raw = n.p.read(HERE/'reviews/proposal.json')
    assert raw['completed']
    old = n.p.read(ROOT/'cad/arm_packaging_audit_v1/reviews/audit.json')
    _, base, _ = v.metadata()
    prior = n.p.read(ROOT/'cad/compact_turnback_pack_v1/reviews/check.json')
    replacement = n.p.read(ROOT/prior['scene'])['parts']
    current = [z for z in base if z['name'] not in prior['replaced_hardware']]+replacement
    members = {z['name']: key for key, rows in old['frame_members'].items() for z in rows}
    assert len(current) == 862 and {z['name'] for z in current} == members.keys()
    new = n.p.read(ROOT/raw['scene'])['parts']
    proposed = [z for z in current if z['name'] != raw['omitted_old_rack_only']]+new
    assert len(proposed) == 872
    delta = {name: np.array(m['transform_from_current']) for m in raw['placements'] for name in m['rows']}
    source = {str(Path(__file__).relative_to(ROOT)): r.sha(Path(__file__)),
        'cad/arm_layout_proposal_v1/reviews/proposal.json': r.sha(HERE/'reviews/proposal.json')}
    def assembly(rows, T, changed=False):
        parts = []
        for z in rows:
            frame = members.get(z['name'], 'forearm')
            M = np.array(T[frame])@(delta.get(z['name'], np.eye(4)) if changed else np.eye(4))
            path = ROOT/z['mesh']
            assert r.sha(path) == z['sha256_stl']
            source[z['mesh']] = z['sha256_stl']
            parts.append(dict(name=z['name'], body='palm', mesh_path=path,
                points=r.read_stl_points(path), color=COLORS[frame], neutral_inverse=M))
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
    arm = n.p.read(ROOT/'cad/modular_roots_v9/design.json')['arm']
    neutral = d.transforms([0, 0, 0, 0], arm)
    flex = d.transforms([0, 0, -35, 0], arm)
    whole = [assembly(current, neutral), assembly(proposed, neutral, True),
             assembly(current, flex), assembly(proposed, flex, True)]
    view = r.view_fit(whole, [dict(angles_deg=[0.]*20)], np.array([.8, -.55, .4]), 1.)
    labels = ['현재 중립 /862개 전부', '별도 짧고 넓은 후보 /872개 전부',
              '현재 손목-35° /862개 전부', '별도 후보 손목-35° /872개 전부']
    images = HERE/'images'
    images.mkdir(exist_ok=True)
    r.sheet([(label, render(a, view, 850)) for label, a in zip(labels, whole)], 2, 850,
        '설치안을 바꾸지 않은 배치 비교: 2열8행 → 4열4행',
        '동일16모터·손 본체·기존 부속품 전부 표시 / 긴 피복·줄 없음 / 폭 증가와 남은 간섭은 원시 검사 참조').save(images/'layout_before_after.png')
    # A second full view exposes the increased width; neither panel hides hardware.
    view2 = r.view_fit(whole[:2], [dict(angles_deg=[0.]*20)], np.array([.05, -.25, 1.]), 1.)
    r.sheet([(label, render(a, view2, 900)) for label, a in zip(labels[:2], whole[:2])], 2, 900,
        '같은 배율의 전체 하드웨어 위쪽 보기',
        '길이를 줄이는 대신 폭이 커지는 비교 후보. 충돌0만으로 외형 개선·수용을 주장하지 않음.').save(images/'width_tradeoff.png')
    beam = [z for z in current if z['name'] == 'Forearm_box_beam']
    detail = assembly(beam+new, neutral, True)
    local = r.view_fit([detail], [dict(angles_deg=[0.]*20)], np.array([.85, .4, .6]), 1.)
    r.sheet([('새 carrier/분할 캡/체결 외곽11 + 실제 전완 빔1', render(detail, local, 1000))], 1, 1000,
        '실제 빔44×42mm를 감싸는 별도 마운트 후보의 국부 상세',
        '모터와 손은 이 상세에 생략. 전체 외형 판정용 그림이 아님. 명목 접촉이며 예압·마찰·강도·조립 미검증.').save(images/'attachment_detail.png')
    n.a.save(out, dict(completed=True, source_sha256=source, actual_CAD_mesh_placements=True,
        whole_panel_counts=[862, 872, 862, 872], whole_hardware_opaque=True,
        new_native_tests=0, geometry_regenerated=0,
        attachment_detail_is_explicit_subset_not_exterior_evidence=True,
        input_hash_guard=all(r.sha(ROOT/f) == h for f, h in source.items()),
        outputs={f.relative_to(ROOT).as_posix(): r.sha(f) for f in images.glob('*.png')},
        exterior_accepted_or_installed=False))


if __name__ == '__main__':
    main()
