"""Current actual solids and the first real housing/arm common material."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import route_solver as m
base=m.drive.module('_housing027_render_inputs',m.HERE/'build_check.py')
g,c,p,np,cq,ROOT,HERE=m.g,m.c,m.p,m.np,m.cq,m.ROOT,m.HERE
helper=m.drive.module('_housing027_render_helper',ROOT/'cad/thumb_root_guide_v1/guide_render.py');r,engine=helper.r,helper.engine
from PIL import ImageDraw
def main():
    out=HERE/'images';out.mkdir(exist_ok=False);raw=p.read(HERE/'reviews/check.json');new=p.read(HERE/'selected/reviews/check.json');assert new['completed'];first=raw['cases'][0];last=next((z for z in reversed(new['cases']) if 'scene' in z),None);source={}
    current=p.read(ROOT/'cad/thumb_sheath_terminal_v2/selected/generated/candidate_scene.json');ret=p.read(ROOT/'cad/sheath_retention_v1/selected/generated/parts/scene.json')['parts'];guide=current['retained_guide_parts']+[z for z in current['parts'] if z['name'] not in ['distal_flanged_ferrule_0','distal_flanged_ferrule_1']]+[z for z in ret if z['body']=='thumb_mcp'];hand=p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')['hand'];pack=p.read(ROOT/'cad/motor_tendon_terminal_v1/selected/generated/assembly_scene.json');pack=pack['new_parts']+pack['retained_parts']+[z for z in ret if z['body']=='forearm']
    def scene(case):
        F=g.g.v.frames(g.DESIGN,case['angles_deg']);N=g.g.v.frames(g.DESIGN,np.zeros(20));parts=[]
        def add(z,alpha,transform=None):
            path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl'];pts=r.read_stl_points(path)
            if transform is not None:pts=pts@transform[:3,:3].T+transform[:3,3]
            parts.append(dict(name=z['name'],body='palm',mesh_path=path,points=pts,color=z.get('color',[.6,.65,.7]),alpha=alpha,neutral_inverse=np.eye(4),mesh_transform=transform))
        # The renderer loads meshes by path; use body FK for moving hand pieces.
        for z in hand+guide:
            if z['name'] in ['thumb_segment2_dark_faceplate_clamp1_bolt','thumb_segment2_dark_faceplate_clamp2_bolt']:continue
            body=z['body'];add(z,.28);parts[-1]['body']=body;O,R=N[body];T=np.eye(4);T[:3,:3]=R;T[:3,3]=O;parts[-1]['neutral_inverse']=np.linalg.inv(T)
        for z in pack:add(z,.25)
        for z in p.read(ROOT/case['scene'])['parts']:
            if z['kind']=='housing':add(z,.95)
        return dict(parts=parts,design=g.DESIGN)
    scenes=[scene(first)]+([scene(last)] if last else []);cases=[first]+([last] if last else []);view=r.view_fit(scenes,[dict(angles_deg=z['angles_deg']) for z in cases],np.array([.65,-.4,.65]),1.);imgs=[]
    try:
        for a,case in zip(scenes,cases):
            a['_render_state']=r.make_renderer(a,{'v':view},1100,engine)
            for j,z in enumerate(a['parts']):a['_render_state']['model'].geom_rgba[j,3]=z['alpha']
            im,_=r.render_one(a,case['angles_deg'],'v',{'v':view},1100,engine);imgs.append(im)
        labels=['첫 고정길이 곡선: 모터/손목 받침과 실제 걸림']+(['두번째 후보: 모터 구간 회피, 손목 걸림은 미해결'] if last else [])
        r.sheet(list(zip(labels,imgs)),len(imgs),1100,'대표 엄지 IP의 긴 피복 두 개 연결 후보','보라: 실제 CAD 피복 · 두 종단만 지지 · 곡선 계수는 설치 가이드가 아님 · 길이/형상 후보이며 장력/마찰/실물 미검증').save(out/'housing_before_after.png')
    finally:
        for a in scenes:
            if '_render_state' in a:a['_render_state']['renderer'].close()
    evidence_case=last if last and 'native' in last else first;hits=[z for key in ['packaging','arm'] for z in evidence_case['native'][key]['overlaps']];hit=max(hits,key=lambda z:z['material_intersection_mm3']);movingmeta=next(z for z in p.read(ROOT/evidence_case['scene'])['parts'] if z['name']==hit['lead']);a=c.load([movingmeta])[0];armmeta=next(z for z in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference'] if z['name']==hit['part']);armstep=ROOT/'cad/modular_roots_v9/parts'/(armmeta['asset']+'.step');source[armstep.relative_to(ROOT).as_posix()]=r.sha(armstep);b=dict(name=hit['part'],shape=cq.importers.importStep(str(armstep)).val());common=a['shape'].intersect(b['shape'],tol=1e-6);assert abs(common.Volume()-hit['material_intersection_mm3'])<1e-5;folder=HERE/'generated/visual_evidence';folder.mkdir(exist_ok=False);meta=[]
    for name,sh,color in [('first_housing_blocked_segment',a['shape'],[.6,.27,.78]),('first_blocking_wrist_part',b['shape'],[.65,.7,.76]),('actual_housing_wrist_common',common,[.96,.01,.03])]:
        st=folder/(name+'.step');me=st.with_suffix('.stl');cq.exporters.export(sh,str(st));cq.exporters.export(sh,str(me),tolerance=.2,angularTolerance=.35);meta.append(dict(name=name,mesh=me.relative_to(ROOT).as_posix(),step=st.relative_to(ROOT).as_posix(),sha256_step=r.sha(st),sha256_stl=r.sha(me),color=color,installed_part=False))
    p.write(folder/'scene.json',dict(parts=meta,actual_common_mm3=common.Volume(),evidence_only=True))
    parts=[]
    for z in meta:
        path=ROOT/z['mesh'];parts.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),color=z['color'],alpha=.3 if 'blocking' in z['name'] else .65 if 'segment' in z['name'] else 1.,neutral_inverse=np.eye(4)))
    a=dict(parts=parts,design=g.DESIGN);view=r.view_fit([a],[dict(angles_deg=[0.]*20)],np.array([.5,-.5,1.]),1.)
    try:
        a['_render_state']=r.make_renderer(a,{'v':view},1100,engine)
        for j,z in enumerate(a['parts']):a['_render_state']['model'].geom_rgba[j,3]=z['alpha']
        im,_=r.render_one(a,[0.]*20,'v',{'v':view},1100,engine);P=np.asarray(common.Center().toTuple())-view['focus'];scale=1100/(2*view['parallel_scale_mm']);x=550+P@view['right']*scale;y=550-P@view['up']*scale;ImageDraw.Draw(im).ellipse((x-40,y-40,x+40,y+40),outline=(215,10,30),width=4);r.sheet([('현재 후보와 손목 받침의 실제 공통 재료',im)],1,1100,'고정 길이만으로 통과를 주장하지 않음',format(common.Volume(),'.9f')+'mm³ / 빨강은 실제 공통 체적 / 손이나 받침을 깎으라는 뜻이 아님').save(out/'actual_block.png')
    finally:a['_render_state']['renderer'].close()
    p.write(out/'report.json',dict(completed=True,source_sha256=source,outputs={z.name:r.sha(z) for z in out.iterdir() if z.is_file()},actual_common_mm3=common.Volume(),actual_saved_meshes=True,whole_hand_drive_approved=False))
if __name__=='__main__':main()
