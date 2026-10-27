"""Opaque whole assemblies, actual fixed terminal/cord geometry and native witness."""
import importlib.util
import sys
from pathlib import Path

sys.dont_write_bytecode = True
spec = importlib.util.spec_from_file_location('_arm056_post_model', Path(__file__).with_name('model.py'))
a = importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)
n, np, d, ROOT, HERE = a.n, a.np, a.d, a.ROOT, a.HERE
q = n.m.drive.module('_arm056_renderer_basis', ROOT/'cad/eight_bank_layout_v1/postprocess.py')
v, r, engine = q.v, q.r, q.engine


def metadata(raw):
    _, base, _ = v.metadata()
    pack = n.p.read(ROOT/'cad/compact_turnback_pack_v1/reviews/check.json')
    current = [z for z in base if z['name'] not in pack['replaced_hardware']]+n.p.read(ROOT/pack['scene'])['parts']
    layout = n.p.read(ROOT/'cad/eight_bank_layout_v1/reviews/check.json')
    guide = n.p.read(ROOT/'cad/radial_turnback_proposal_v1/reviews/check.json')
    previous = [z for z in current if z['name'] != 'Forearm_twenty_axis_module_rack']+n.p.read(ROOT/layout['scene'])['parts']
    previous = [z for z in previous if z['name'] not in a.a.OLD_NAMES]+n.p.read(ROOT/guide['scene'])['parts']
    proposed = [z for z in previous if z['name'] != a.FRAME]+n.p.read(ROOT/raw['scene'])['parts']
    assert len(current) == 862 and len(previous) == len(proposed) == 872
    membership = {z['name']: key for key,rows in n.p.read(ROOT/'cad/arm_packaging_audit_v1/reviews/audit.json')['frame_members'].items() for z in rows}
    delta = {name: np.array(z['transform_from_current']) for z in layout['placements'] for name in z['rows']}
    return current, previous, proposed, membership, delta, guide


def main():
    out = HERE/'reviews/evidence.json'
    assert not out.exists()
    raw = n.p.read(HERE/'reviews/check.json')
    assert raw['completed']
    current, previous, proposed, membership, delta, guide = metadata(raw)
    S = np.array(raw['design']['transform_from_ARM055'])
    moved = set(raw['design']['moved_hardware_names'])
    source = dict(raw['source_sha256'])
    for f in [Path(__file__),HERE/'reviews/check.json']:
        source[f.relative_to(ROOT).as_posix()] = r.sha(f)
    arm = n.p.read(ROOT/'cad/modular_roots_v9/design.json')['arm']
    neutral, flex = [d.transforms(q, arm) for q in [[0,0,0,0],[0,0,-35,0]]]
    def transform(z, F, after=False, placement=True):
        frame = membership.get(z['name'],'forearm')
        M = delta.get(z['name'],np.eye(4)) if placement else np.eye(4)
        if after and z['name'] in moved:
            M = S@M
        return np.array(F[frame])@M
    def assembly(rows, F=neutral, after=False, placement=True, red=False):
        parts=[]
        for z in rows:
            path = ROOT/z['mesh']
            assert r.sha(path) == z['sha256_stl']
            source[z['mesh']] = z['sha256_stl']
            frame = membership.get(z['name'],'forearm')
            color = z.get('color',q.COLORS[frame]) if z['name'].startswith('arm056_cord') else q.COLORS[frame]
            if z['name'] == a.NEW_FRAME:
                color = [.93,.48,.1]
            parts.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),
                neutral_inverse=transform(z,F,after,placement),color=[.97,.04,.03] if red else color))
        return dict(parts=parts,design=n.g.DESIGN)
    def render(scene,view,size):
        try:
            scene['_render_state']=r.make_renderer(scene,{'v':view},size,engine)
            scene['_render_state']['model'].geom_rgba[:,3]=1.
            return r.render_one(scene,[0.]*20,'v',{'v':view},size,engine)[0]
        finally:
            if '_render_state' in scene:
                scene['_render_state']['renderer'].close()
                del scene['_render_state']
    images=HERE/'images'
    images.mkdir(exist_ok=True)
    scenes=[assembly(previous),assembly(proposed,after=True),assembly(previous,flex),assembly(proposed,flex,True)]
    view=r.view_fit(scenes,[dict(angles_deg=[0.]*20)],np.array([.8,-.55,.4]),1.)
    labels=['이전 ARM055 미채택 /872전부','종단·조절부·죠19행 방향 변경 /872전부',
            '이전 손목 -35° /872전부','새 후보 손목 -35° /872전부']
    r.sheet([(s,render(z,view,850)) for s,z in zip(labels,scenes)],2,850,
        '고정 종단 묶음을 돌리고 실제 M3자리까지 일체 받침 연결',
        '모터16개 설치·명령각 추가변경0 / 기존862 설치 유지 / 미채택 / 긴 피복·줄 없음').save(images/'unit_before_after.png')
    actual=assembly(current,placement=False)
    fit=r.view_fit([actual,scenes[1]],[dict(angles_deg=[0.]*20)],np.array([.05,-.25,1.]),1.)
    r.sheet([('현재862,그대로',render(actual,fit,900)),('별도872후보,미채택',render(scenes[1],fit,900))],2,900,
        '현재 설치와 별도 종단방향 후보의 실제 전체 CAD',
        '전체 부품 불투명 / 외형 개선·통합구동 승인 아님').save(images/'current_proposal.png')
    packet_names=a.members()|{'thumb_ip_two_tail_spool','thumb_ip_stock_output_horn','thumb_ip_tail_clamp_1','thumb_ip_tail_clamp_2',a.NEW_FRAME}
    local=assembly([z for z in proposed if z['name'] in packet_names],after=True)
    cord_meta=n.p.read(ROOT/raw['cord_scene'])['parts']
    local['parts']+=assembly(cord_meta)['parts']
    fit=r.view_fit([local],[dict(angles_deg=[0.]*20)],np.array([.7,.55,.6]),1.)
    r.sheet([('종단19행 + 스풀/혼/꼬리나사4행 + 국부 줄12조각',render(local,fit,1100))],1,1100,
        '실제 R10홈의 새 접선 → 18mm 직선 → R5통로 → 조절부',
        '명령0 한자세 줄2개 / 다른 모터·팔 생략 / 감김30°·210° / 전체 긴줄·운동 승인 아님').save(images/'terminal_cords.png')
    witnesses=[]
    positive=[(k,z) for k,t in raw['native'].items() for z in t['overlaps'] if k!='local_cords_internal']
    if positive:
        label,record=max(positive,key=lambda item:item[1]['material_intersection_mm3'])
        lookup={z['name']:z for z in proposed+cord_meta}
        selected=[dict(lookup[record[k]]) for k in ['lead','part']]
        for z in selected:
            if 'step' not in z:
                rel='cad/modular_roots_v9/parts/'+z['name']+'.step'
                assert r.sha(ROOT/rel)==source[rel]
                z.update(step=rel,sha256_step=source[rel])
        F=neutral
        for state in raw['motion']:
            if label.startswith(state['id']+'__'):
                F=state['transforms']
        loaded=n.c.load(selected,source)
        posed=[n.g.original.transform(z['shape'],transform(z,F,True)) for z in loaded]
        common=posed[0].intersect(posed[1],tol=1e-6)
        path=n.a.a.prior.export([dict(name='arm056_actual_common',shape=common,body='forearm',evidence_only=True,color=[.97,.04,.03])],HERE/'generated/actual_common')
        red=assembly(n.p.read(ROOT/path)['parts'],red=True)
        joined=assembly(selected,F,True)
        joined['parts']+=red['parts']
        fit=r.view_fit([joined],[dict(angles_deg=[0.]*20)],np.array([.8,-.6,.5]),1.)
        zoom=r.view_fit([red],[dict(angles_deg=[0.]*20)],np.array([.8,-.6,.5]),1.)
        r.sheet([('실제 두부품 전체 /빨강 공통재료',render(joined,fit,850)),('같은 공통재료만 별도 확대',render(red,zoom,850))],2,850,
            '종단 묶음과 실제 지지 변경 뒤에도 남은 최대 겹침',record['lead']+' / '+record['part']+' / '+str(record['material_intersection_mm3'])+'mm³').save(images/'actual_block.png')
        witnesses.append(dict(group=label,record=record,scene=path,reconstructed_common_volume_mm3=common.Volume()))
    ports=[]
    for z in guide['design']:
        ports.append(dict(line=z['line'],before=z['free_port_world_mm'],after=(S@np.r_[z['free_port_world_mm'],1.])[:3],
            tangent_after=S[:3,:3]@z['free_tangent_world'],same_fixed_housing_path_length_mm=z['shoulder_to_free_port_length_mm']))
    n.a.save(out,dict(completed=True,source_sha256=source,whole_hardware_counts=[872,872,872,872,862,872],
        whole_hardware_opaque=True,actual_common=witnesses,free_ports=ports,
        output_sha256={f.relative_to(ROOT).as_posix():r.sha(f) for f in images.glob('*.png')},
        native_groups_reexecuted=0,common_pairs_constructed=len(witnesses),
        input_hash_guard=all(r.sha(ROOT/f)==h for f,h in source.items()),current862_installation_replaced=False,candidate_adopted=False))


if __name__ == '__main__':
    main()
