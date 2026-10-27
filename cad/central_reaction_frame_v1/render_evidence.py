"""Opaque complete candidate and explicit structural/interface subsets,without CAD replay."""
import importlib.util
import sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm061_render',Path(__file__).with_name('model.py'))
a=importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)
n,np,d,ROOT,HERE=a.n,a.np,a.d,a.ROOT,a.HERE
r,engine=a.p.p.r,a.p.p.engine


def main():
    out=HERE/'reviews/evidence.json';assert not out.exists()
    raw=n.p.read(HERE/'reviews/check.json');follow=n.p.read(HERE/'reviews/followup.json')
    prior=n.p.read(ROOT/'cad/integrated_motor_end_v1/reviews/check.json')
    _,_,base,membership,delta,reloc,clock,shift=a.p.p.metadata(prior)
    saved060=n.p.read(ROOT/'cad/rotation_reserved_support_v1/reviews/check.json')
    before=[z for z in base if z['name'] not in a.p.OLD]+n.p.read(ROOT/saved060['scene'])['parts']
    fresh=n.p.read(ROOT/raw['scene'])['parts'];after=[z for z in before if z['name']!=a.OLD]+fresh
    assert len(before)==len(after)==871
    membership[a.p.GUIDE]='hand';membership[a.NAME]='forearm'
    moved=set(prior['design']['moved_names']);source={}
    arm=n.p.read(ROOT/'cad/modular_roots_v9/design.json')['arm']
    neutral=d.transforms([0,0,0,0],arm);compound=d.transforms([55,40,-20,15],arm)
    def scene(rows,F=neutral,world=False):
        parts=[]
        for z in rows:
            name=z['name'];M=np.array(delta.get(name,np.eye(4)))
            if name in reloc: M=reloc[name]@M
            if name in a.p.a.a.CLOCK_NAMES: M=clock@M
            if name in moved: M=shift@M
            if world: M=np.eye(4)
            else: M=np.array(F[membership.get(name,'forearm')])@M
            path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl']
            parts.append(dict(name=name,body='palm',mesh_path=path,points=r.read_stl_points(path),neutral_inverse=M,color=z.get('color',[.3,.5,.75])))
        return dict(parts=parts,design=n.g.DESIGN)
    def render(s,view,size):
        try:
            s['_render_state']=r.make_renderer(s,{'v':view},size,engine);s['_render_state']['model'].geom_rgba[:,3]=1.
            return r.render_one(s,[0.]*20,'v',{'v':view},size,engine)[0]
        finally:
            if '_render_state' in s: s['_render_state']['renderer'].close();del s['_render_state']
    def fit(ss,vec): return r.view_fit(ss,[dict(angles_deg=[0.]*20)],np.array(vec),1.)
    images=HERE/'images';images.mkdir(exist_ok=True)
    full=[scene(before),scene(after),scene(before,compound),scene(after,compound)]
    view=fit(full,[.8,-.55,.4])
    r.sheet([(label,render(s,view,850)) for label,s in zip(['이전 ARM060 /871전부','새 중앙닫힌프레임 /871전부','이전 복합55/40/-20/15 /871','새 같은복합 /871'],full)],2,850,
        '실제 CAD 전후 /손과16원격모터 설치 유지',
        '새carrier1만교체 /현재862설치변경0 /전회전보수공간·나사간섭남아미채택 /전체구동·힘승인 아님').save(images/'before_after.png')
    names={a.OLD,a.NAME,'Forearm_box_beam','Roll_output_flange','AK_forearm_roll'}
    names|={z['name'] for z in before if z['name'].startswith(('arm054_beam_cap_','arm054_M3'))}
    small=[scene([z for z in before if z['name'] in names]),scene([z for z in after if z['name'] in names])]
    view=fit(small,[.55,-.85,.65])
    r.sheet([(label,render(s,view,1000)) for label,s in zip(['이전 원환·엄지축방향살 /구조만별도확대','새 사각닫힌프레임·연결살 /같은구조범위'],small)],2,1000,
        '실제 모터패드에서 빔 클램프까지 이어지는 중앙 지지',
        '원래M3패드32/아래클램프2재료보존검사 /4x3단면은강도동등성증명아님 /모터삭제 아님').save(images/'support_path.png')
    interfaces=scene([z for z in after if z['name'] in {'Forearm_box_beam','Roll_output_flange','AK_forearm_roll'}])
    view=fit([interfaces],[1.,.15,.25])
    distance=raw['reaction_interfaces'][1]['distance_mm']
    r.sheet([('실제포장형상3개,출력허브·체결은미모델링',render(interfaces,view,1100))],1,1100,
        '전완 회전부 반력 경로의 남은 모델 공백',
        f'빔/플랜지거리0 /플랜지/명목모터외곽거리{distance:.6f}mm /실제모터출력축의연결불가능을뜻하지않음').save(images/'roll_interface.png')
    common=n.p.read(ROOT/follow['scene'])['parts']
    panels=[]
    for i,label in [(0,'보수적회전원통/받침의공통,실제출력공통아님'),(1,'실제M3나사/새연결살공통')]:
        s=scene([common[i]],world=True)
        panels.append((label,render(s,fit([s],[.55,-.85,.65]),900)))
    r.sheet(panels,2,900,'남은 두 종류의 공통재료,각기 다른 확대배율',
        '원통공통과실제부품간섭을구별 /증거2는설치물아님 /분류로rawfalse덮지않음').save(images/'remaining_common.png')
    protection=n.p.read(ROOT/raw['protection_input_scene'])['parts']
    s=scene(fresh+protection,world=True)
    view=fit([s],[.55,-.85,.65])
    r.sheet([('새캐리어+기존9회전공간/너트접근참고',render(s,view,1100))],1,1100,
        '현재 실제 보호공간을 새 지지와 대조',
        '보수적원통은정확회전합집합보다클수있음 /기존보호10재생성0 /모든순간실제출력검사 아님').save(images/'protected_space.png')
    for f in [Path(__file__),HERE/'model.py',HERE/'reviews/check.json',HERE/'reviews/followup.json']:
        source[f.relative_to(ROOT).as_posix()]=r.sha(f)
    n.a.save(out,dict(completed=True,source_sha256=source,output_sha256={f.relative_to(ROOT).as_posix():r.sha(f) for f in images.glob('*.png')},
        opaque=True,whole_hardware_counts=[871]*4,exact_before_after_metadata_names_preserved=True,
        native_or_geometry_rebuilt=0,protected_or_common_shapes_are_installed=False,
        pixels_matrices_armvisualunits_numerically_replayed=False,input_hash_guard=all(r.sha(ROOT/f)==h for f,h in source.items())))


if __name__=='__main__': main()
