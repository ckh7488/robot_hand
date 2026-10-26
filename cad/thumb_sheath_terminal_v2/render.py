"""Actual saved candidate meshes and explicit upper-keeper service stage."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import selected_model as m
g,c,p,np,cq,ROOT,HERE=m.g,m.c,m.p,m.np,m.cq,m.ROOT,m.HERE
helper=m.drive.module('_offset024_render_helpers',ROOT/'cad/thumb_root_guide_v1/guide_render.py');r,poses,engine=helper.r,helper.poses,helper.engine
from PIL import ImageDraw
def main():
    out=Path(__file__).resolve().parent/'images';out.mkdir(exist_ok=False);report=p.read(HERE/'reviews/static.json');entry=p.read(HERE/'reviews/keeper_entry.json');assert report['completed'] and report['static_candidate_clear'] and entry['completed']
    source={};new=p.read(ROOT/report['scene'])['parts'];housing=p.read(ROOT/report['housing_scene'])['parts'];cord=p.read(ROOT/report['cord_scene'])['parts'];prior=p.read(ROOT/'cad/thumb_guide_assembly_v2/generated/parts/scene.json')['parts'];hand=p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')['hand'];retained=[z for z in prior if z['name'] not in ['liner_flange_keeper_0','liner_flange_keeper_1']]
    oldreport=p.read(ROOT/'cad/thumb_sheath_terminal_v1/reviews/static.json');oldnew=p.read(ROOT/oldreport['scene'])['parts'];oldhousing=p.read(ROOT/oldreport['housing_scene'])['parts'];hit=p.read(ROOT/'cad/thumb_sheath_terminal_v1/generated/visual_evidence/scene.json')['parts'][0]
    cap=next(z for z in new if z['name']=='distal_housing_keeper_1');shape=c.load([cap])[0]['shape'];sample=next(z for z in entry['samples'] if z['distance_mm']==1.);T=np.asarray(sample['transform_local']);shape=g.original.transform(shape,g.original.WORLD@T@g.original.INV)
    ev=Path(__file__).resolve().parent/'generated/visual_evidence';ev.mkdir(parents=True,exist_ok=False);st=ev/'upper_keeper_entry_1mm.step';me=st.with_suffix('.stl');cq.exporters.export(shape,str(st));cq.exporters.export(shape,str(me),tolerance=.025,angularTolerance=.1);moved=dict(cap,step=st.relative_to(ROOT).as_posix(),mesh=me.relative_to(ROOT).as_posix(),sha256_step=r.sha(st),sha256_stl=r.sha(me),installed=False);p.write(ev/'scene.json',dict(parts=[moved],evidence_only=True))
    F=poses.forward_kinematics(g.DESIGN,np.zeros(20))['bodies']
    def scene(parts,whole=False):
        rows=[]
        for z in hand+parts:
            is_hand=z in hand
            if is_hand and (z['name'] in ['thumb_segment2_dark_faceplate_clamp1_bolt','thumb_segment2_dark_faceplate_clamp2_bolt'] or (not whole and not z['body'].startswith('thumb'))):continue
            file=ROOT/z['mesh'];assert r.sha(file)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl']
            rows.append(dict(name=z['name'],body=z['body'],mesh_path=file,points=r.read_stl_points(file),neutral_inverse=np.linalg.inv(F[z['body']]),color=[.65,.7,.75] if is_hand else z.get('color',[.5,.6,.7]),alpha=.18 if is_hand else 1. if z['name']==hit['name'] else .38 if z['name'] in ['paired_liner_carrier','distal_housing_shoe_1'] else .9))
        return dict(parts=rows,design=g.DESIGN)
    scenes=[scene(retained+oldnew+oldhousing+[hit]),scene(retained+new+housing+cord),scene([z for z in prior if z['name'] in entry['installed_original_guide']]+[moved]),scene(retained+new+housing+cord,True)]
    view=r.view_fit(scenes[:3],[dict(angles_deg=[0.]*20)]*3,np.array([.25,-.55,1.]),1.);wide=r.view_fit([scenes[3]],[dict(angles_deg=[0.]*20)],np.array([.3,-.25,1.]),1.);imgs=[]
    try:
        for j,s in enumerate(scenes):
            cam=wide if j==3 else view;s['_render_state']=r.make_renderer(s,{'view':cam},1000,engine)
            for k,z in enumerate(s['parts']):s['_render_state']['model'].geom_rgba[k,3]=z['alpha']
            im,_=r.render_one(s,[0.]*20,'view',{'view':cam},1000,engine);imgs.append(im)
        common=c.load([hit])[0]['shape'];pt=np.asarray(common.Center().toTuple())-view['focus'];factor=1000/(2*view['parallel_scale_mm']);x=500+np.dot(pt,view['right'])*factor;y=500-np.dot(pt,view['up'])*factor;ImageDraw.Draw(imgs[0]).ellipse((x-40,y-40,x+40,y+40),outline=(200,15,25),width=4)
        r.sheet([('이전: 기존 고정부와 실제 겹침',imgs[0]),('교정: 옆 입구·곡선 통로·지지 기둥/리브',imgs[1])],2,1000,'손 본체·모터 유지, 새 피복 받침의 연결 구조 교정','빨강: 이전 실제 공통 재료 · 보라: 짧은 피복 참고 · 상류 두 곡선은 같은 한 줄의 연장 · 긴 피복/전체 구동·장력 미완').save(out/'offset_before_after.png')
        r.sheet([('위 덮개 진입 1mm, 다른 새 부품 아직 없음',imgs[2]),('새 종단을 포함한 후보 외형',imgs[3])],2,1000,'부분 조립 경로와 손 전체에서 본 후보','기존 가는 안내관/끝마디/핀은 그대로 · 전체 종단 조립/공구·강도/외형 수용 승인 아님 · 모터 구동 영상 아님').save(out/'service_context.png')
    finally:
        for s in scenes:
            if '_render_state' in s:s['_render_state']['renderer'].close()
    p.write(out/'report.json',dict(completed=True,source_sha256=source,outputs={z.name:r.sha(z) for z in out.iterdir() if z.is_file()},simulation_steps=0,upper_keeper_stage_only=True,whole_assembly_approved=False,actual_meshes=True))
if __name__=='__main__':main()
