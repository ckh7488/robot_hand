"""Actual full hardware before/after and changed-unit evidence, without native replay."""
import importlib.util
import sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm059_post_model',Path(__file__).with_name('model.py'))
a=importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)
n,np,d,ROOT,HERE=a.n,a.np,a.d,a.ROOT,a.HERE
r,engine=a.p.r,a.p.engine


def metadata(raw):
    prior=n.p.read(ROOT/'cad/forward_terminal_support_v1/reviews/check.json')
    current,_,before,membership,delta,_,_,reloc,clock=a.p.metadata(prior)
    new=n.p.read(ROOT/raw['scene'])['parts']
    proposed=[z for z in before if z['name'] not in a.OLD]+new
    assert len(before)==872 and len(proposed)==871
    return current,before,proposed,membership,delta,reloc,clock,np.array(raw['design']['mechanical_translation_from_ARM057'])


def main():
    out=HERE/'reviews/evidence.json'
    assert not out.exists()
    raw=n.p.read(HERE/'reviews/check.json')
    assert raw['completed']
    current,before,proposed,membership,delta,reloc,clock,shift=metadata(raw)
    source={}
    moved=set(raw['design']['moved_names'])
    arm=n.p.read(ROOT/'cad/modular_roots_v9/design.json')['arm']
    neutral,flex=[d.transforms(v,arm) for v in [[0,0,0,0],[0,0,-35,0]]]
    def matrix(z,F,after=False,placement=True,command=None):
        name=z['name']
        M=np.array(delta.get(name,np.eye(4))) if placement else np.eye(4)
        if placement:
            if name in reloc: M=reloc[name]@M
            if name in a.a.CLOCK_NAMES: M=clock@M
            if after and name in moved: M=shift@M
            if command is not None and name in a.a.CLOCK_NAMES|{'thumb_ip_stock_output_horn','thumb_ip_stock_central_retainer'}: M=command@M
        return np.array(F[membership.get(name,'forearm')])@M
    def scene(rows,F=neutral,after=False,placement=True,command=None,red=False):
        parts=[]
        for z in rows:
            path=ROOT/z['mesh']
            assert r.sha(path)==z['sha256_stl']
            source[z['mesh']]=z['sha256_stl']
            color=z.get('color',[.3,.5,.75])
            if z['name'] in {a.FRAME,*a.JAWS}: color=[.93,.48,.1]
            parts.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),
                neutral_inverse=matrix(z,F,after,placement,command),color=[.98,.04,.03] if red else color))
        return dict(parts=parts,design=n.g.DESIGN)
    def render(z,view,size):
        try:
            z['_render_state']=r.make_renderer(z,{'v':view},size,engine)
            z['_render_state']['model'].geom_rgba[:,3]=1.
            return r.render_one(z,[0.]*20,'v',{'v':view},size,engine)[0]
        finally:
            if '_render_state' in z:
                z['_render_state']['renderer'].close()
                del z['_render_state']
    def fit(scenes,direction): return r.view_fit(scenes,[dict(angles_deg=[0.]*20)],np.array(direction),1.)
    images=HERE/'images'
    images.mkdir(exist_ok=True)
    full=[scene(before),scene(proposed,after=True),scene(before,flex),scene(proposed,flex,True)]
    view=fit(full,[.8,-.55,.4])
    r.sheet([(label,render(z,view,850)) for label,z in zip(['이전ARM057 /872전부','새통합지지·우회받침 /871전부','이전 손목-35° /872','새 손목-35° /871'],full)],2,850,
        '실제 CAD 전후,모터 설치·전기선 유지 /새 지지대 후보',
        '불투명 전체부속 /현재862설치교체0 /2지지기능의1체화로872→871 /팔전체·연속운동·외형승인 아님').save(images/'before_after.png')
    old_unit=[z for z in before if membership.get(z['name'],'forearm')=='forearm' and z['name'].startswith(('thumb_ip_','motor_housing_clamp_','arm057_'))]
    new_unit=[z for z in proposed if membership.get(z['name'],'forearm')=='forearm' and (z['name'].startswith(('thumb_ip_','motor_housing_clamp_')) or z['name'] in {a.FRAME,*a.JAWS})]
    assert len(old_unit)==49 and len(new_unit)==48
    left,right=scene(old_unit),scene(new_unit,after=True)
    cord=scene(n.p.read(ROOT/raw['cord_scene'])['parts'],placement=False)
    right['parts']+=cord['parts']
    view=fit([left,right],[.75,.3,.65])
    r.sheet([('이전 모터끝49행',render(left,view,1000)),('새 모터끝48행+명령0두국부줄12조각',render(right,view,1000))],2,1000,
        '실제 전기홈/덮개·리드와 구동 종단의 일체 지지',
        '전기/모터·출력 설치 동일 /실제죠·조절부16은명시적바깥이동 /새긴피복·손까지연결·장력승인 아님').save(images/'motor_end.png')
    carrier_names={'arm054_eight_bank_carrier',a.CARRIER,'thumb_ip_two_tail_spool','thumb_ip_tail_clamp_1',
        'middle_dip_actuator_adapter','middle_dip_actuator_rack_screw_1','middle_dip_actuator_rack_screw_2'}
    left,right=scene([z for z in before if z['name'] in carrier_names]),scene([z for z in proposed if z['name'] in carrier_names],after=True)
    view=fit([left,right],[.55,-.85,.65])
    r.sheet([('이전 실제carrier',render(left,view,1000)),('새 원래M3패드/허브를잇는우회carrier',render(right,view,1000))],2,1000,
        'middle_dip 45°/−13받침, 단순 삭제 대신 실제 우회 재료 연결',
        '두carrier각1solid /원래M3축·바깥패드유지 /새경로 강도·조립·전체구동검증 아님').save(images/'carrier_before_after.png')
    records=[(k,z) for k,t in raw['native'].items() for z in t['overlaps'] if 'internal' not in k or not k.startswith('new_cords')]
    witnesses=[]
    if records:
        label,record=max(records,key=lambda item:item[1]['material_intersection_mm3'])
        lookup={z['name']:z for z in proposed+n.p.read(ROOT/raw['cord_scene'])['parts']}
        selected=[dict(lookup[record[k]]) for k in ['lead','part']]
        for z in selected:
            if 'step' not in z:
                rel='cad/modular_roots_v9/parts/'+z['name']+'.step'
                z.update(step=rel,sha256_step=raw['source_sha256'][rel])
        F=neutral
        for state in raw['arm_motion']:
            if label.startswith(state['id']+'__'): F=state['transforms']
        command=None
        for state in raw['output_command_checks']:
            if label.startswith('command_'+str(int(state['command_deg']))+'__'): command=np.array(state['transform'])
        loaded=n.c.load(selected,source)
        shapes=[n.g.original.transform(z['shape'],matrix(z,F,True,True,command)) for z in loaded]
        common=shapes[0].intersect(shapes[1],tol=1e-6)
        path=n.a.a.prior.export([dict(name='arm059_actual_common',shape=common,body='forearm',evidence_only=True,color=[.98,.04,.03])],HERE/'generated/actual_common')
        red=scene(n.p.read(ROOT/path)['parts'],placement=False,red=True)
        joined=scene(selected,F,True,command=command)
        joined['parts']+=red['parts']
        r.sheet([('실제 두 부품과 빨강 공통',render(joined,fit([joined],[.8,-.6,.5]),850)),
            ('같은 공통재료만 확대',render(red,fit([red],[.8,-.6,.5]),850))],2,850,
            '새 후보에 남은 최대 실제 간섭',record['lead']+' / '+record['part']+' / '+str(record['material_intersection_mm3'])+'mm³').save(images/'actual_common.png')
        witnesses.append(dict(group=label,record=record,scene=path,reconstructed_common_volume_mm3=common.Volume()))
    for path in [Path(__file__),HERE/'reviews/check.json']: source[path.relative_to(ROOT).as_posix()]=r.sha(path)
    n.a.save(out,dict(completed=True,source_sha256=source,output_sha256={f.relative_to(ROOT).as_posix():r.sha(f) for f in images.glob('*.png')},
        opaque=True,whole_hardware_counts=[872,871,872,871],local_hardware_counts=[49,48],neutral_cord_segments=12,
        actual_common=witnesses,native_groups_reexecuted=0,extra_actual_common_pairs=len(witnesses),
        current862_installation_replaced=False,proposal_adopted=False,
        input_hash_guard=all(r.sha(ROOT/f)==h for f,h in source.items())))


if __name__=='__main__': main()
