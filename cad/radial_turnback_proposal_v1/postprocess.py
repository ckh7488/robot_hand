"""Full actual hardware comparison, with no hidden old clamps or motors."""
import importlib.util
import sys
from pathlib import Path

sys.dont_write_bytecode = True
spec = importlib.util.spec_from_file_location('_arm055_post_model', Path(__file__).with_name('model.py'))
a = importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)
n, np, d, ROOT, HERE = a.n, a.np, a.d, a.ROOT, a.HERE
q = n.m.drive.module('_arm055_renderer_basis', ROOT/'cad/eight_bank_layout_v1/postprocess.py')
v, r, engine = q.v, q.r, q.engine


def main():
    out = HERE/'reviews/evidence.json'
    assert not out.exists()
    raw = n.p.read(HERE/'reviews/check.json')
    assert raw['completed']
    old = n.p.read(ROOT/'cad/eight_bank_layout_v1/reviews/check.json')
    source = dict(raw['source_sha256'])
    for f in [Path(__file__), HERE/'reviews/check.json']:
        source[f.relative_to(ROOT).as_posix()] = r.sha(f)
    _, base, _ = v.metadata()
    pack = n.p.read(ROOT/'cad/compact_turnback_pack_v1/reviews/check.json')
    current = [z for z in base if z['name'] not in pack['replaced_hardware']]+n.p.read(ROOT/pack['scene'])['parts']
    support = n.p.read(ROOT/old['scene'])['parts']
    previous = [z for z in current if z['name'] != 'Forearm_twenty_axis_module_rack']+support
    new = n.p.read(ROOT/raw['scene'])['parts']
    proposed = [z for z in previous if z['name'] not in a.OLD_NAMES]+new
    assert len(current) == 862 and len(previous) == len(proposed) == 872
    membership = {z['name']: frame for frame, rows in n.p.read(ROOT/'cad/arm_packaging_audit_v1/reviews/audit.json')['frame_members'].items() for z in rows}
    delta = {name: np.array(z['transform_from_current']) for z in old['placements'] for name in z['rows']}
    arm = n.p.read(ROOT/'cad/modular_roots_v9/design.json')['arm']
    neutral = d.transforms([0, 0, 0, 0], arm)
    flex = d.transforms([0, 0, -35, 0], arm)
    def assembly(rows, T, placement=True, red=False):
        parts = []
        for z in rows:
            frame = membership.get(z['name'], 'forearm')
            M = np.array(T[frame])@(delta.get(z['name'], np.eye(4)) if placement else np.eye(4))
            path = ROOT/z['mesh']
            assert r.sha(path) == z['sha256_stl']
            source[z['mesh']] = z['sha256_stl']
            parts.append(dict(name=z['name'], body='palm', mesh_path=path, points=r.read_stl_points(path),
                neutral_inverse=M, color=[.97, .04, .03] if red else ([.93,.48,.1] if z['name'].startswith('arm055_') else q.COLORS[frame])))
        return dict(parts=parts, design=n.g.DESIGN)
    def render(scene, view, size):
        try:
            scene['_render_state'] = r.make_renderer(scene, {'v':view}, size, engine)
            scene['_render_state']['model'].geom_rgba[:,3] = 1.
            return r.render_one(scene, [0.]*20, 'v', {'v':view}, size, engine)[0]
        finally:
            if '_render_state' in scene:
                scene['_render_state']['renderer'].close()
                del scene['_render_state']
    images = HERE/'images'
    images.mkdir(exist_ok=True)
    scenes = [assembly(previous,neutral),assembly(proposed,neutral),assembly(previous,flex),assembly(proposed,flex)]
    view = r.view_fit(scenes,[dict(angles_deg=[0.]*20)],np.array([.8,-.55,.4]),1.)
    labels = ['이전 미채택 ARM054 /872전부','두 관 굽힘면 변경 후보 /872전부',
              'ARM054 손목-35° /872전부','새 두 관 후보 손목-35° /872전부']
    r.sheet([(label,render(scene,view,850)) for label,scene in zip(labels,scenes)],2,850,
        '실제 죔쇠 자리는 유지하고 두 R40 굽힘면만 바깥으로 분리',
        '모든 부속 불투명 / 기존862설치 변경0 / 긴 피복·줄 없음 / 기존 빔·캡·다른 죔쇠 간섭은 그대로').save(images/'planes_before_after.png')
    actual = assembly(current,neutral,False)
    view2 = r.view_fit([actual,scenes[1]],[dict(angles_deg=[0.]*20)],np.array([.05,-.25,1.]),1.)
    r.sheet([('현재862설치, 그대로',render(actual,view2,900)),('별도872후보, 미채택',render(scenes[1],view2,900))],2,900,
        '현재 설치와 별도 후보의 같은 배율 전체 비교',
        '모터16/손/전선·고정부 유지. 두 관의 방향 변경을 전체 구동·외형 개선으로 세지 않음.').save(images/'current_proposal.png')
    local_names = {z['name'] for z in new}
    for i in [0,1]:
        local_names.add('motor_housing_clamp_'+str(i)+'_jaw_1')
        local_names |= {'motor_housing_clamp_'+str(i)+'_'+kind+'_'+str(j) for kind in ['bolt','nut'] for j in [0,1]}
    local = assembly([z for z in proposed if z['name'] in local_names],neutral)
    assert len(local['parts']) == 12
    fit = r.view_fit([local],[dict(angles_deg=[0.]*20)],np.array([.7,.6,.5]),1.)
    r.sheet([('두 교체죠/관 + 기존 반대죠2/M2볼트4·너트4만',render(local,fit,1000))],1,1000,
        '실제 부착 죔쇠와 바깥으로 갈라진 동일 R40 관, 12행 부분 상세',
        '다른 모터·전체 팔 생략 / 0거리나 유효SOLID는 실제 체결력·마찰·조립 승인이 아님').save(images/'clamp_detail.png')
    actual_common = []
    candidates = [(label,z) for label,test in raw['native'].items() for z in test['overlaps']]
    if candidates:
        label,record = max(candidates,key=lambda z:z[1]['material_intersection_mm3'])
        lookup = {z['name']:z for z in proposed}
        selected = [dict(lookup[record[k]]) for k in ['lead','part']]
        for z in selected:
            if 'step' not in z:
                rel = 'cad/modular_roots_v9/parts/'+z['name']+'.step'
                assert r.sha(ROOT/rel) == source[rel]
                z.update(step=rel,sha256_step=source[rel])
        loaded = n.c.load(selected,source)
        posed = neutral
        if '__new2__' in label and label != 'new2__retained870_neutral':
            posed = next(z['transforms'] for z in raw['motion'] if label.startswith(z['id']+'__'))
        shapes = [n.g.original.transform(z['shape'],np.array(posed[membership.get(z['name'],'forearm')])@delta.get(z['name'],np.eye(4))) for z in loaded]
        common = shapes[0].intersect(shapes[1],tol=1e-6)
        path = n.a.a.prior.export([dict(name='arm055_actual_common',shape=common,body='forearm',evidence_only=True,color=[.97,.04,.03])],HERE/'generated/actual_common')
        red = assembly(n.p.read(ROOT/path)['parts'],neutral,red=True)
        joined = assembly(selected,posed)
        joined['parts'] += red['parts']
        fit = r.view_fit([joined],[dict(angles_deg=[0.]*20)],np.array([.8,-.6,.5]),1.)
        zoom = r.view_fit([red],[dict(angles_deg=[0.]*20)],np.array([.8,-.6,.5]),1.)
        r.sheet([('실제 두 부품 전체 /빨강 공통재료',render(joined,fit,850)),('같은 공통재료만 확대 /별도 배율',render(red,zoom,850))],2,850,
            '두 변경 관의 새 검사에서 남은 실제 최대 겹침',
            record['lead']+' / '+record['part']+' / '+str(record['material_intersection_mm3'])+'mm³').save(images/'actual_block.png')
        actual_common.append(dict(group=label,record=record,scene=path,reconstructed_STEP_common_volume_mm3=common.Volume()))
    n.a.save(out,dict(completed=True,source_sha256=source,
        whole_hardware_counts=[872,872,872,872,862,872],all_whole_hardware_opaque=True,
        actual_common=actual_common,new_full_native_groups=0,
        new_common_pairs_constructed=len(actual_common),
        outputs={f.relative_to(ROOT).as_posix():r.sha(f) for f in images.glob('*.png')},
        input_hash_guard=all(r.sha(ROOT/f)==h for f,h in source.items()),
        current862_installation_replaced=False,proposal_adopted=False))


if __name__ == '__main__':
    main()
