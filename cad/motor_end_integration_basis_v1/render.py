"""Opaque actual CAD and explicitly rejected diagnostic cuts, not a new layout."""
import importlib.util
import sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm058_render_survey',Path(__file__).with_name('survey.py'))
s=importlib.util.module_from_spec(spec)
spec.loader.exec_module(s)
n,np,ROOT,HERE,p=s.n,s.np,s.ROOT,s.HERE,s.p
r,engine=p.r,p.engine


def main():
    out=HERE/'reviews/evidence.json'
    assert not out.exists()
    survey=n.p.read(HERE/'reviews/survey.json')
    follow=n.p.read(HERE/'reviews/carrier_followup.json')
    raw=n.p.read(ROOT/'cad/forward_terminal_support_v1/reviews/check.json')
    _,_,proposed,membership,delta,_,_,reloc,clock=p.metadata(raw)
    source={}
    def scene(rows,placement=True,red=False):
        parts=[]
        for z in rows:
            path=ROOT/z['mesh']
            assert r.sha(path)==z['sha256_stl']
            source[z['mesh']]=z['sha256_stl']
            M=np.array(delta.get(z['name'],np.eye(4))) if placement else np.eye(4)
            if placement:
                if z['name'] in reloc: M=reloc[z['name']]@M
                if z['name'] in s.a.CLOCK_NAMES: M=clock@M
            color=z.get('color',[.3,.5,.75])
            if z['name'].endswith(('lead1','lead2','lead3')): color=[.1,.65,.85]
            parts.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),
                neutral_inverse=M,color=[.98,.04,.03] if red else color))
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
    def fit(scenes,direction):
        return r.view_fit(scenes,[dict(angles_deg=[0.]*20)],np.array(direction),1.)
    def select(names): return [z for z in proposed if z['name'] in names]
    saved=n.p.read(ROOT/survey['scene'])['parts']
    later=n.p.read(ROOT/follow['scene'])['parts']
    images=HERE/'images'
    images.mkdir(exist_ok=True)
    unit=select({z['name'] for z in proposed if membership.get(z['name'],'forearm')=='forearm' and z['name'].startswith(('thumb_ip_','motor_housing_clamp_','arm057_'))})
    whole=scene(unit)
    focused=scene(select({'arm057_straight_jaw_0',*[f'thumb_ip_actuator_lead{i}' for i in range(1,4)],
        'thumb_ip_actuator_wire_guide_lid',*[f'thumb_ip_actuator_lid_screw_{i}' for i in [1,2]]}))
    focused['parts']+=scene([z for z in saved if z['name']=='arm058_jaw_lead2_common'],False,True)['parts']
    r.sheet([('ARM057 실제 모터끝 전체'+str(len(unit))+'행',render(whole,fit([whole],[.6,.35,.7]),950)),
        ('실제 전선3·뚜껑/나사·아래죠만 /빨강 lead2공통',render(focused,fit([focused],[.8,.35,.55]),950))],2,950,
        '전기 통로와 구동 종단을 함께 보존해야 하는 변경범위',
        '실제 CAD 불투명 / 국부 부품만 명시 표시 / 설치변경0 / 통합지지 새형상·조립검사 아님').save(images/'integration_unit.png')
    carrier_names={'arm054_eight_bank_carrier','thumb_ip_two_tail_spool','thumb_ip_tail_clamp_1',
        'middle_dip_actuator_adapter','middle_dip_actuator_rack_screw_1','middle_dip_actuator_rack_screw_2'}
    actual=scene(select(carrier_names))
    unchanged=select(carrier_names-{'arm054_eight_bank_carrier'})
    wrong=scene(unchanged)
    wrong['parts']+=scene([z for z in saved if z['name'].startswith('arm058_counterfactual_carrier_solid_')],False)['parts']
    correct=scene(unchanged)
    correct['parts']+=scene([z for z in later if z['name'].startswith('arm058_counterfactual_middle_dip_solid_')],False)['parts']
    view=fit([actual,wrong,correct],[.55,-.85,.65])
    r.sheet([(label,render(z,view,850)) for label,z in [('전: 실제 carrier1솔리드',actual),
        ('진단후: 엄지기둥제거→3솔리드·실제충돌남음',wrong),('진단후: 이웃받침자유구간제거→2솔리드',correct)]],3,850,
        '실제 캐리어 전후 CAD, 두 절삭은 수리안이 아닌 연결성 진단',
        '스풀/꼬리보스·middle_dip adapter/M3 포함 / 두 진단 모두 미채택 / 받침을 지우면 하중 경로가 끊김').save(images/'carrier_dependency.png')
    common=scene([z for z in saved+later if z['name'] in {'arm058_spool_carrier_common','arm058_tail_carrier_common'}],False,True)
    view=fit([common],[.6,-.8,.7])
    bands=scene([z for z in saved if z['name'].startswith('arm058_adjuster_')],False,True)
    r.sheet([('새 spool/꼬리보스 공통만 확대 /둘다 이웃45°받침',render(common,view,900)),
        ('기존 조절부/base 공통 재사용·두 높이영역 분할',render(bands,fit([bands],[.8,-.6,.5]),900))],2,900,
        '빨강은 부품 전체가 아닌 실제 공통재료',
        '좌 spool18.0578526203 /tail.7857790902mm³ / 우foot44.5619152273+post46.2806024600 /절삭 허가·강도 승인 아님').save(images/'actual_obstructions.png')
    for path in [Path(__file__),HERE/'reviews/survey.json',HERE/'reviews/carrier_followup.json']:
        source[path.relative_to(ROOT).as_posix()]=r.sha(path)
    n.a.save(out,dict(completed=True,source_sha256=source,output_sha256={f.relative_to(ROOT).as_posix():r.sha(f) for f in images.glob('*.png')},
        opaque=True,whole_motor_end_rows=len(unit),unit_detail_subset_rows=len(focused['parts']),
        carrier_before_after_diagnostics_only=True,new_native_queries=0,new_installed_geometry_rows=0,
        input_hash_guard=all(r.sha(ROOT/f)==h for f,h in source.items())))


if __name__=='__main__': main()
