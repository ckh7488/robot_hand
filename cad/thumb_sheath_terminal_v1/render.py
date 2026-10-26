"""Show the prior local guide and the rejected abutment with actual common material."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import distal_model as m
g,c,p,np,cq,ROOT,HERE=m.g,m.c,m.p,m.np,m.cq,m.ROOT,m.HERE
helper=m.drive.module('_distal023_render_helpers',ROOT/'cad/thumb_root_guide_v1/guide_render.py')
r,poses,engine=helper.r,helper.poses,helper.engine
from PIL import ImageDraw
def main():
    out=HERE/'images';out.mkdir(exist_ok=False);report=p.read(HERE/'reviews/static.json');assert report['completed'] and not report['static_candidate_clear']
    source={};new=p.read(ROOT/report['scene'])['parts'];housing=p.read(ROOT/report['housing_scene'])['parts'];prior=p.read(ROOT/'cad/thumb_guide_assembly_v2/generated/parts/scene.json')['parts'];hand=p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')['hand']
    by={z['name']:z for z in c.load(new+housing+prior)}
    a=by['distal_housing_shoe_1'];b=by['paired_liner_carrier'];common=a['shape'].intersect(b['shape'],tol=1e-6)
    want=next(z['material_intersection_mm3'] for z in report['cases'][0]['native']['hand']['overlaps'] if z['lead']==a['name'] and z['part']==b['name']);assert abs(common.Volume()-want)<1e-5
    ev=HERE/'generated/visual_evidence';ev.mkdir(parents=True,exist_ok=False);st=ev/'upper_shoe_carrier_common.step';me=st.with_suffix('.stl');cq.exporters.export(common,str(st));cq.exporters.export(common,str(me),tolerance=.025,angularTolerance=.1)
    hit=dict(name='actual_upper_shoe_carrier_common',step=st.relative_to(ROOT).as_posix(),mesh=me.relative_to(ROOT).as_posix(),sha256_step=r.sha(st),sha256_stl=r.sha(me),body='thumb_mcp',color=[.97,.02,.04],actual_common_mm3=common.Volume(),installed=False)
    p.write(ev/'scene.json',dict(parts=[hit],evidence_only=True))
    F=poses.forward_kinematics(g.DESIGN,np.zeros(20))['bodies']
    def scene(parts,whole=False):
        rows=[]
        for z in hand+parts:
            if z in hand and (z['name'] in ['thumb_segment2_dark_faceplate_clamp1_bolt','thumb_segment2_dark_faceplate_clamp2_bolt'] or (not whole and not z['body'].startswith('thumb'))):continue
            f=ROOT/z['mesh'];assert r.sha(f)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl'];is_hand=z in hand
            rows.append(dict(name=z['name'],body=z['body'],mesh_path=f,points=r.read_stl_points(f),neutral_inverse=np.linalg.inv(F[z['body']]),color=[.65,.7,.75] if is_hand else z.get('color',[.5,.6,.7]),alpha=.18 if is_hand else 1. if z['name']==hit['name'] else .36 if z['name'] in ['distal_housing_shoe_1','paired_liner_carrier','distal_housing_keeper_1'] else .85))
        return dict(parts=rows,design=g.DESIGN)
    retained=[z for z in prior if z['name'] not in report['design']['replaced_guide_parts']]
    scenes=[scene(prior),scene(retained+new+housing+[hit]),scene(retained+new+housing,True)]
    view=r.view_fit(scenes[:2],[dict(angles_deg=[0.]*20)]*2,np.array([.25,-.55,1.]),1.)
    wide=r.view_fit([scenes[2]],[dict(angles_deg=[0.]*20)],np.array([.3,-.25,1.]),1.);imgs=[]
    try:
        for j,s in enumerate(scenes):
            cam=wide if j==2 else view;s['_render_state']=r.make_renderer(s,{'view':cam},1000,engine)
            for k,z in enumerate(s['parts']):s['_render_state']['model'].geom_rgba[k,3]=z['alpha']
            im,_=r.render_one(s,[0.]*20,'view',{'view':cam},1000,engine);imgs.append(im)
        d=ImageDraw.Draw(imgs[1]);point=np.asarray(common.Center().toTuple())-view['focus'];factor=1000/(2*view['parallel_scale_mm']);x=500+np.dot(point,view['right'])*factor;y=500-np.dot(point,view['up'])*factor
        d.ellipse((x-42,y-42,x+42,y+42),outline=(200,15,25),width=4)
        r.sheet([('기존: 얇은 줄 안내관, 피복 받침 없음',imgs[0]),('미채택: 새 피복 받침이 기존 고정부와 겹침',imgs[1])],2,1000,'손 쪽 피복 종단 검토, 손 본체·모터 위치 유지','빨강: 위쪽 새 받침과 기존 가이드 지지대의 실제 공통 재료 · 손 전체 절삭 대상 아님 · 조립/긴 피복·장력 미검증').save(out/'distal_before_after.png')
        r.sheet([('손 전체에서 본 미채택 부착물',imgs[2])],1,1000,'새 피복 종단의 후보 외형','초록/청록: 새 받침 · 금색: 맞춤 피복 끝구 · 보라: 짧은 피복 참고 · 외형 수용/전체 구동 승인 아님').save(out/'candidate_context.png')
    finally:
        for s in scenes:
            if '_render_state' in s:s['_render_state']['renderer'].close()
    p.write(out/'report.json',dict(completed=True,source_sha256=source,outputs={z.name:r.sha(z) for z in out.iterdir() if z.is_file()},actual_common_mm3=common.Volume(),simulation_steps=0,candidate_adopted=False))
if __name__=='__main__':main()
