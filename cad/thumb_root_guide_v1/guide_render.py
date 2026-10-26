import sys,types
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import guide_candidate3 as g
import mujoco
from PIL import ImageDraw
np,p,cq,ROOT=g.np,g.p,g.cq,g.ROOT
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'cad/gesture_hand_v3'))
import poses
r=g.g.drive.module('_guide020_renderer',ROOT/'cad/gesture_hand_v3/render_gestures.py')
class SerialModel:
    @staticmethod
    def from_xml_string(xml):
        node=r.ET.fromstring(xml);node.find('compiler').set('usethread','false')
        return mujoco.MjModel.from_xml_string(r.ET.tostring(node,encoding='unicode'))
engine=types.SimpleNamespace(MjModel=SerialModel,MjData=mujoco.MjData,Renderer=mujoco.Renderer,mj_forward=mujoco.mj_forward)

def main():
    out=HERE/'images';out.mkdir(exist_ok=False);report=p.read(HERE/'selected2/reviews/check.json');assert report['completed'] and report['all_nominal_snapshots_clear']
    old=p.read(HERE/'reviews/check.json');hand=p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')['hand']
    source={};F=poses.forward_kinematics(g.DESIGN,np.zeros(20))['bodies'];evidence=HERE/'generated/visual_evidence_scope2';evidence.mkdir(parents=True,exist_ok=False)
    oldmeta=p.read(ROOT/old['scene'])['parts'];liner=next(z for z in oldmeta if z['name']=='flanged_liner_1');skin=next(z for z in hand if z['name']=='thumb_segment2_dark_faceplate')
    shapes=g.c.load([liner,skin],source);common=shapes[0]['shape'].intersect(shapes[1]['shape'],tol=1e-6)
    st=evidence/'initial_liner_cover_common.step';me=st.with_suffix('.stl');cq.exporters.export(common,str(st));cq.exporters.export(common,str(me),tolerance=.02,angularTolerance=.1)
    hit=dict(name='initial_actual_liner_cover_intersection',body='palm',mesh=me.relative_to(ROOT).as_posix(),sha256_stl=r.sha(me),color=[.95,.02,.05],step=st.relative_to(ROOT).as_posix(),sha256_step=r.sha(st),installed_part=False,actual_common_mm3=common.Volume())
    service=p.read(HERE/'selected2/reviews/service_scopes.json');assert service['completed']
    path=next(z for z in service['paths'] if z['id']=='flanged_liner_1_axial_insert');sample=path['samples'][3]
    newmeta=p.read(ROOT/report['scene'])['parts'];liner_meta=next(z for z in newmeta if z['name']=='flanged_liner_1')
    shape=g.c.load([liner_meta],source)[0]['shape'];T=np.asarray(sample['transform_local']);moved=g.original.transform(shape,g.original.WORLD@T@g.original.INV)
    tip_meta=next(z for z in hand if z['name']=='thumb_segment3_tip_core');tip=g.c.load([tip_meta],source)[0]['shape'];frames=g.g.v.frames(g.DESIGN,path['hand_angles_deg']);neutral=g.g.v.frames(g.DESIGN,np.zeros(20));tip=g.g.v.relocate(tip,frames['thumb_ip'],neutral['thumb_ip'])
    assembly_common=moved.intersect(tip,tol=1e-6);assert assembly_common.Volume()>.001
    ast=evidence/'assembly_liner_common.step';ame=ast.with_suffix('.stl');cq.exporters.export(assembly_common,str(ast));cq.exporters.export(assembly_common,str(ame),tolerance=.02,angularTolerance=.1)
    mst=evidence/'assembly_liner_shifted.step';mme=mst.with_suffix('.stl');cq.exporters.export(moved,str(mst));cq.exporters.export(moved,str(mme),tolerance=.035,angularTolerance=.1)
    ahit=dict(hit,name='assembly_actual_liner_tip_intersection',mesh=ame.relative_to(ROOT).as_posix(),step=ast.relative_to(ROOT).as_posix(),sha256_stl=r.sha(ame),sha256_step=r.sha(ast),actual_common_mm3=assembly_common.Volume())
    moved_meta=dict(liner_meta,mesh=mme.relative_to(ROOT).as_posix(),step=mst.relative_to(ROOT).as_posix(),sha256_stl=r.sha(mme),sha256_step=r.sha(mst),installed_part=False)
    p.write(evidence/'scene.json',dict(parts=[hit,ahit,moved_meta],evidence_only=True))
    def assembly(case,newmeta,whole=False,before=False,extra=None,show_cords=True):
        parts=[]
        def add(z,body,alpha,color=None):
            path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl']
            parts.append(dict(name=z['name'],body=body,mesh_path=path,points=r.read_stl_points(path),color=color or z.get('color',[.6,.66,.72]),alpha=alpha,neutral_inverse=np.linalg.inv(F[body]) if body!='palm' else np.eye(4)))
        for z in hand:
            if z['name'] in report['design']['replaced_hand_parts']:continue
            if not whole and not z['body'].startswith('thumb'):continue
            add(z,z['body'],.23,[.64,.69,.74])
        for z in newmeta:add(z,z['body'],.45 if z['name']=='paired_liner_carrier' else .98)
        if show_cords:
            for z in p.read(ROOT/case['cord_scene'])['parts']:add(z,'palm',1.)
        if before:add(hit,'palm',1.)
        if extra:add(extra,'palm',1.)
        return dict(parts=parts,design=g.DESIGN)
    opening=next(z for z in report['cases'] if z['id']=='open');closing=next(z for z in report['cases'] if z['id']=='parent_full');prior=next(z for z in old['cases'] if z['id']=='open');minus=next(z for z in report['cases'] if z['id']=='ip_minus15')
    scenes=[assembly(prior,oldmeta,before=True),assembly(opening,newmeta),assembly(closing,newmeta),assembly(opening,newmeta,whole=True)]
    cases=[prior,opening,closing,opening]
    installed=set(path['already_installed_new_parts']+path['moving']);limited=[z for z in newmeta if z['name'] in installed]
    scenes.extend([assembly(minus,[moved_meta if z['name']=='flanged_liner_1' else z for z in limited],extra=ahit,show_cords=False),assembly(minus,limited,show_cords=False)]);cases.extend([minus,minus])
    close=r.view_fit([scenes[i] for i in [0,1,2,4,5]],[dict(angles_deg=cases[i]['angles_deg']) for i in [0,1,2,4,5]],np.array([.4,-.7,1.]),1.)
    whole=r.view_fit([scenes[3]],[dict(angles_deg=opening['angles_deg'])],np.array([.3,-.25,1.]),1.)
    images=[]
    try:
        for index,(a,case) in enumerate(zip(scenes,cases)):
            camera=whole if index==3 else close;a['_render_state']=r.make_renderer(a,{'view':camera},900,engine)
            for i,z in enumerate(a['parts']):a['_render_state']['model'].geom_rgba[i,3]=z['alpha']
            image,_=r.render_one(a,case['angles_deg'],'view',{'view':camera},900,engine)
            if index in [0,4]:
                cloud=r.read_stl_points(me if index==0 else ame);center=cloud.mean(axis=0)-camera['focus'];factor=900/(2*camera['parallel_scale_mm']);x=450+np.dot(center,camera['right'])*factor;y=450-np.dot(center,camera['up'])*factor
                draw=ImageDraw.Draw(image);draw.ellipse((x-26,y-26,x+26,y+26),outline=(215,15,30),width=4)
                draw.text((max(8,x-100),y+30),'실제 덮개 겹침' if index==0 else '끼우는 중에만 걸림',font=r.font(23),fill=(185,20,30))
            images.append(image)
        r.sheet([('첫 가이드: 기존 덮개를 가로지름',images[0]),('교정: 손은 그대로, 가이드 출구 이동',images[1]),('엄지를 함께 굽힌 최종 목표각',images[2])],3,900,
            '기존 나사 두 곳에 붙는 가이드 후보 · 손/모터 배치 유지',
            '청록: 받침/분리 덮개 · 노랑: 안내관 · 주황/파랑: 구동줄 · 얇은 관의 공급품/강도 및 모터~입구 연결 미완').save(out/'guide_details.png')
        r.sheet([('전체 손에서 본 새 가이드 위치',images[3]),('손가락쪽 고정 가이드와 두 줄',images[1])],2,900,
            '손가락쪽 가이드 후보를 추가 · 전체 구동계는 아직 미완',
            '현재 영상은 저장 CAD의 정적 자세 · 관은 명목 OD1.2/ID0.8mm,R5 가정 · 팔꿈치 배선 확장 없음').save(out/'guide_hand.png')
        r.sheet([('안내관을 5mm 더 밀어 넣는 조립 중간',images[4]),('같은 단계에서 안내관이 앉은 위치는 겹침0',images[5])],2,900,
            '남은 문제는 안내관 삽입 경로 · 완성 자세와 구별',
            '아직 줄을 끼우지 않은 조립 단계 · 빨강은 실제 안내관/끝마디 공통 재료 · 순서/진입 자세부터 교정,손 절삭 근거 아님').save(out/'assembly_block.png')
    finally:
        for a in scenes:
            if '_render_state' in a:a['_render_state']['renderer'].close()
    assert all(r.sha(ROOT/z)==h for z,h in source.items())
    p.write(out/'report.json',dict(completed=True,source_sha256=source,outputs={z.name:r.sha(z) for z in out.iterdir() if z.is_file()},simulation_steps=0,asset_compile_threads=False,
        actual_saved_meshes=True,scope='New nominal distal guide/hand poses and actual first liner/cover intersection.Motor-to-terminal/root crossing and flexible conduit/tension absent;not motor-driven hand motion.'))
if __name__=='__main__':main()
