"""Actual full hardware views and independent command-zero/quarter-turn details."""
import importlib.util
import sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm057_post_model',Path(__file__).with_name('model.py'))
a=importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)
n,np,d,ROOT,HERE=a.n,a.np,a.d,a.ROOT,a.HERE
old=n.m.drive.module('_arm057_old_render',ROOT/'cad/terminal_unit_orientation_v1/postprocess.py')
q,v,r,engine=old.q,old.v,old.r,old.engine


def metadata(raw):
    raw56=n.p.read(ROOT/'cad/terminal_unit_orientation_v1/reviews/check.json')
    current,_,previous,membership,delta,_=old.metadata(raw56)
    new=n.p.read(ROOT/raw['scene'])['parts']
    excluded={a.p.NEW_FRAME,'arm055_split_return_0','arm055_split_return_1'}
    proposed=[z for z in previous if z['name'] not in excluded]+new
    assert len(current)==862 and len(previous)==len(proposed)==872
    before_S=np.array(raw56['design']['transform_from_ARM055'])
    before_names=set(raw56['design']['moved_hardware_names'])
    reloc={name:np.array(z['transform_from_ARM055']) for z in raw['design']['ports'] for name in z['moved_names']}
    clock=np.array(raw['design']['clock_transform_proposal'])
    return current,previous,proposed,membership,delta,before_S,before_names,reloc,clock


def main():
    out=HERE/'reviews/evidence.json'
    assert not out.exists()
    raw=n.p.read(HERE/'reviews/check.json')
    assert raw['completed']
    current,previous,proposed,membership,delta,S,before_names,reloc,clock=metadata(raw)
    source=dict(raw['source_sha256'])
    for f in [Path(__file__),HERE/'reviews/check.json']:
        source[f.relative_to(ROOT).as_posix()]=r.sha(f)
    arm=n.p.read(ROOT/'cad/modular_roots_v9/design.json')['arm']
    neutral,flex=[d.transforms(q,arm) for q in [[0,0,0,0],[0,0,-35,0]]]
    stock={'thumb_ip_stock_output_horn','thumb_ip_stock_central_retainer'}
    def matrix(z,F,after=False,placement=True,command=None):
        name=z['name']
        M=delta.get(name,np.eye(4)) if placement else np.eye(4)
        if placement:
            if after:
                if name in reloc: M=reloc[name]@M
                if name in a.CLOCK_NAMES: M=clock@M
                if command is not None and name in a.CLOCK_NAMES|stock: M=command@M
            elif name in before_names: M=S@M
        return np.array(F[membership.get(name,'forearm')])@M
    def assembly(rows,F=neutral,after=False,placement=True,command=None,red=False):
        parts=[]
        for z in rows:
            path=ROOT/z['mesh']
            assert r.sha(path)==z['sha256_stl']
            source[z['mesh']]=z['sha256_stl']
            frame=membership.get(z['name'],'forearm')
            color=z.get('color',q.COLORS[frame]) if z['name'].startswith('arm057_cord') else q.COLORS[frame]
            if z['name']==a.FRAME or z['name'] in a.GUIDES: color=[.93,.48,.1]
            parts.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),
                neutral_inverse=matrix(z,F,after,placement,command),color=[.97,.04,.03] if red else color))
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
    fit=r.view_fit(scenes,[dict(angles_deg=[0.]*20)],np.array([.8,-.55,.4]),1.)
    labels=['이전 ARM056 미채택 /872전부','새 손방향 R5지지·직선관 /872전부','이전 손목-35° /872전부','새 후보 손목-35° /872전부']
    r.sheet([(s,render(z,fit,850)) for s,z in zip(labels,scenes)],2,850,
        '긴 종단틀·되돌림관을 실제 새 지지와 손방향 직선관으로 교체',
        '현재862 설치 유지 / 모터16 설치 동일 / 출력 스풀만 명시적 -90°체결위상 / 외형·통합구동 승인 아님').save(images/'support_before_after.png')
    actual=assembly(current,placement=False)
    fit=r.view_fit([actual,scenes[1]],[dict(angles_deg=[0.]*20)],np.array([.05,-.25,1.]),1.)
    r.sheet([('현재862,그대로',render(actual,fit,900)),('별도872후보,미채택',render(scenes[1],fit,900))],2,900,
        '실제 전체 CAD,같은 배율 비교','전체 부속 불투명 / 긴 피복·손까지의 줄 없음 / 기존 다른 스풀·빔·손목·캡 간섭 미해결').save(images/'current_proposal.png')
    local_names={a.FRAME,*a.GUIDES,*reloc,*a.CLOCK_NAMES,*stock,'thumb_ip_actuator_adapter'}
    local_meta=[z for z in proposed if z['name'] in local_names]
    assert len(local_meta)==29
    locals_=[]
    local=assembly(local_meta,after=True)
    local['parts']+=assembly(n.p.read(ROOT/raw['cord_scene'])['parts'])['parts']
    locals_.append(('명령0° /두줄감김120°·120°',local))
    for state in raw['motor_command_states']:
        local=assembly(local_meta,after=True,command=np.array(state['transform']))
        local['parts']+=assembly(n.p.read(ROOT/state['cord_scene'])['parts'])['parts']
        locals_.append(('명령'+str(state['command_deg'])+'° /독립 국부기하',local))
    fit=r.view_fit([z[1] for z in locals_],[dict(angles_deg=[0.]*20)],np.array([.7,.5,.55]),1.)
    r.sheet([(s,render(z,fit,900)) for s,z in locals_],3,900,
        '실제 R10홈 → 새 접선/R5 → 조절부·죠/직선관,원래 M3 지지',
        '각29하드웨어+두국부줄12조각만 / 모터·팔 나머지 생략 / 세 독립기하이며 같은긴줄 운동·손관절구동 아님').save(images/'terminal_commands.png')
    witness=[]
    # Coupling thread envelopes and same-cord seams remain in raw results.
    positive=[(k,z) for k,t in raw['native'].items() for z in t['overlaps'] if k=='changed26__unchanged846_neutral' or '__new26_and_local12__' in k]
    if positive:
        label,record=max(positive,key=lambda item:item[1]['material_intersection_mm3'])
        lookup={z['name']:z for z in proposed+n.p.read(ROOT/raw['cord_scene'])['parts']}
        selected=[dict(lookup[record[k]]) for k in ['lead','part']]
        for z in selected:
            if 'step' not in z:
                rel='cad/modular_roots_v9/parts/'+z['name']+'.step'
                assert r.sha(ROOT/rel)==source[rel]
                z.update(step=rel,sha256_step=source[rel])
        F=neutral
        for state in raw['arm_motion']:
            if label.startswith(state['id']+'__'): F=state['transforms']
        loaded=n.c.load(selected,source)
        sh=[n.g.original.transform(z['shape'],matrix(z,F,True)) for z in loaded]
        common=sh[0].intersect(sh[1],tol=1e-6)
        path=n.a.a.prior.export([dict(name='arm057_actual_common',shape=common,body='forearm',evidence_only=True,color=[.97,.04,.03])],HERE/'generated/actual_common')
        red=assembly(n.p.read(ROOT/path)['parts'],red=True)
        joined=assembly(selected,F,True)
        joined['parts']+=red['parts']
        fit=r.view_fit([joined],[dict(angles_deg=[0.]*20)],np.array([.8,-.6,.5]),1.)
        zoom=r.view_fit([red],[dict(angles_deg=[0.]*20)],np.array([.8,-.6,.5]),1.)
        r.sheet([('실제 두부품 /빨강 공통재료',render(joined,fit,850)),('같은 공통재료만 별도 확대',render(red,zoom,850))],2,850,
            '새 변경부 검사에 남은 실제 최대 원시 겹침',record['lead']+' / '+record['part']+' / '+str(record['material_intersection_mm3'])+'mm³').save(images/'actual_common.png')
        witness.append(dict(group=label,record=record,scene=path,reconstructed_common_volume_mm3=common.Volume()))
    n.a.save(out,dict(completed=True,source_sha256=source,whole_hardware_counts=[872,872,872,872,862,872],
        whole_hardware_opaque=True,local_subset_hardware_rows=29,local_cord_segments_per_state=12,
        actual_common=witness,native_groups_reexecuted=0,common_pairs_constructed=len(witness),
        output_sha256={f.relative_to(ROOT).as_posix():r.sha(f) for f in images.glob('*.png')},
        input_hash_guard=all(r.sha(ROOT/f)==h for f,h in source.items()),current862_installation_replaced=False,candidate_adopted=False))


if __name__=='__main__': main()
