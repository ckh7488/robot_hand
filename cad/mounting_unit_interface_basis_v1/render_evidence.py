"""Actual CAD reference comparison,whole mounting unit and source output pattern."""
import importlib.util
import sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm062_render',Path(__file__).with_name('model.py'))
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
n,np,d,ROOT,HERE=a.n,a.np,a.d,a.ROOT,a.HERE
r,engine=a.p.p.p.r,a.p.p.p.engine


def main():
    out=HERE/'reviews/evidence.json';assert not out.exists()
    raw=n.p.read(HERE/'reviews/check.json');follow=n.p.read(HERE/'reviews/interface_followup.json')
    base=n.p.read(ROOT/'cad/integrated_motor_end_v1/reviews/check.json')
    _,_,meta,membership,delta,reloc,clock,shift=a.p.p.p.metadata(base)
    stage060=n.p.read(ROOT/'cad/rotation_reserved_support_v1/reviews/check.json')
    stage061=n.p.read(ROOT/'cad/central_reaction_frame_v1/reviews/check.json')
    before=[z for z in meta if z['name'] not in a.p.p.OLD]+n.p.read(ROOT/stage060['scene'])['parts']
    before=[z for z in before if z['name']!=a.p.OLD]+n.p.read(ROOT/stage061['scene'])['parts']
    assert len(before)==871
    vendor=n.p.read(ROOT/raw['vendor_reference_scene'])['parts']
    entry=n.p.read(ROOT/raw['entry_reference_scene'])['parts']
    unit=n.p.read(ROOT/raw['mounting_reference_scene'])['parts']
    axes=n.p.read(ROOT/follow['scene'])['parts']
    source={};moved=set(base['design']['moved_names'])
    def scene(items,world=False):
        parts=[]
        for z in items:
            name=z['name'];M=np.array(delta.get(name,np.eye(4)))
            if name in reloc:M=reloc[name]@M
            if name in a.p.p.a.a.CLOCK_NAMES:M=clock@M
            if name in moved:M=shift@M
            if world:M=np.eye(4)
            path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl']
            parts.append(dict(name=name,body='palm',mesh_path=path,points=r.read_stl_points(path),neutral_inverse=M,color=z.get('color',[.3,.5,.75])))
        return dict(parts=parts,design=n.g.DESIGN)
    def render(s,view,size):
        try:
            s['_render_state']=r.make_renderer(s,{'v':view},size,engine);s['_render_state']['model'].geom_rgba[:,3]=1.
            return r.render_one(s,[0.]*20,'v',{'v':view},size,engine)[0]
        finally:
            if '_render_state' in s:s['_render_state']['renderer'].close();del s['_render_state']
    def fit(ss,vec):return r.view_fit(ss,[dict(angles_deg=[0.]*20)],np.array(vec),1.)
    images=HERE/'images';images.mkdir(exist_ok=True)
    # This is a registered vendor surface comparison,not an adopted replacement.
    after=[z for z in before if z['name']!='AK_forearm_roll']+vendor
    full=[scene(before),scene(after)]
    view=fit(full,[.8,-.55,.4])
    r.sheet([(label,render(s,view,1050)) for label,s in zip(['기존871 /명목AK실린더','참고871 /공식표면을출력기준에등록'],full)],2,1050,
        '실제 CAD 비교 /손과16원격모터·지지형상 변경0',
        '오른쪽은설치후보채택아님 /원본표면은닫힌solid아님 /현재862유지 /긴재료·전체팔운동검사0').save(images/'reference_before_after.png')
    beam=[z for z in before if z['name']=='Forearm_box_beam']
    structural=[z for z in before if z['name'] in set(a.UNIT_NAMES)|{a.p.NAME,'Forearm_box_beam'}]
    small=[scene(unit+beam,world=True),scene(structural+entry)]
    view=fit(small,[.5,-.7,.6])
    r.sheet([(label,render(s,view,1050)) for label,s in zip(['실제front체결6기능영역+빔','연결된전체지지+양쪽축진입참고4'],small)],2,1050,
        '나사 끝 한 점 대신 전체 체결과 연결살의 관계',
        'lower영역1은기존carrier일부 /cap·나사·너트5유지 /진입20mm·실제머리/너트외경연장가정 /공급공구선정아님').save(images/'whole_mounting_unit.png')
    ss=[scene(vendor,world=True),scene(axes,world=True)]
    panels=[]
    for label,s in zip(['실제공식출력쪽표면 /닫힌재료아님','실제탭축6의판두께4mm연장 /설치물아님'],ss):
        panels.append((label,render(s,fit([s],[.12,1.,.12]),1000)))
    r.sheet(panels,2,1000,'공식 출력면과 6개 M3 탭의 축',
        '도면M3x6/PCD20 /STEP탭Ø2.5x6·핀Ø3x2분리 /기존판에는축구멍없음 /실제출력체결·토크승인 아님').save(images/'actual_output_pattern.png')
    # Save one complete reference-space/common region,not a tiny bolt-corner fix.
    want=max([q for q in raw['native']['new_axial_entry4__all_existing871']['overlaps'] if q['part']==a.p.NAME],key=lambda q:q['material_intersection_mm3'])
    E=n.c.load([z for z in entry if z['name']==want['lead']],source)[0]
    C=n.c.load(n.p.read(ROOT/stage061['scene'])['parts'],source)[0]
    common=E['shape'].intersect(C['shape'],tol=1e-6)
    assert common.isValid() and len(common.Solids())==1
    row=dict(name='arm062_entry_reference_carrier_common',shape=common,body='forearm',color=[.85,.1,.1],role='reference_space_common_not_actual_tool_material')
    common_scene=n.a.a.prior.export([row],HERE/'generated/common_reference')
    s=scene(n.p.read(ROOT/common_scene)['parts'],world=True)
    r.sheet([('새연결살/너트쪽진입참고의공통만별도확대',render(s,fit([s],[.6,-.8,.6]),1000))],1,1000,
        '전체 진입공간 공통의 실제 CAD',
        '선정공구의실제충돌확정아님 /증거1설치0 /raw기록삭제·검사제외0').save(images/'entry_common.png')
    for f in [Path(__file__),HERE/'model.py',HERE/'reviews/check.json',HERE/'reviews/interface_followup.json']:
        source[f.relative_to(ROOT).as_posix()]=r.sha(f)
    n.a.save(out,dict(completed=True,source_sha256=source,output_sha256={f.relative_to(ROOT).as_posix():r.sha(f) for f in images.glob('*.png')},
        common_scene=common_scene,common_source_pair=want,common_volume_mm3=common.Volume(),common_source_queries=1,
        native_groups_repeated=0,opaque=True,registered_reference_display_rows=[871,871],installed_hardware_changes=0,
        source_open_surface_reference_is_not_adoption=True,pixels_matrices_armvisualunits_numerically_replayed=False,
        input_hash_guard=all(r.sha(ROOT/f)==h for f,h in source.items())))


if __name__=='__main__':main()
