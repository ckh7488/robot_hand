"""Render saved wrist material, failed mounting and actual local guide."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import final_edge_model as a
from wrist_survey import export
m,g,c,p,np,cq,ROOT,HERE=a.m,a.g,a.c,a.p,a.np,a.cq,a.ROOT,a.HERE
helper=m.drive.module('_wrist028_render_helper',ROOT/'cad/thumb_root_guide_v1/guide_render.py');r,engine=helper.r,helper.engine

def main():
    out=HERE/'images';out.mkdir(exist_ok=False);source={};meta=p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')['hand'];old=p.read(HERE/'reviews/check.json');selected=p.read(HERE/'selected2/reviews/check.json')
    first=p.read(ROOT/old['scene'])['parts'];last=p.read(ROOT/selected['scene'])['parts'];survey=p.read(HERE/'reviews/survey.json');wrist=p.read(ROOT/survey['wrist_scene'])['parts'];straight=p.read(ROOT/survey['probe_scene'])['parts']
    context=[z for z in meta if z['name'] in ['Palm_tapered_chassis','thumb_root_fixed_housing','thumb_root_spread_carrier']]
    hit=old['cases'][0]['native']['hand']['overlaps'][0];part=c.load([next(z for z in first if z['name']==hit['lead'])])[0];ob=c.load([next(z for z in meta if z['name']==hit['part'])])[0];common=part['shape'].intersect(ob['shape'],tol=1e-6)
    assert abs(common.Volume()-hit['material_intersection_mm3'])<1e-5
    evidence=export([dict(name='actual_first_cap_thumb_common',shape=common,color=[.96,.02,.03],body='palm',installed_part=False)],HERE/'generated/actual_common');cm=p.read(ROOT/evidence)['parts']
    def make(rows,commons=()):
        parts=[]
        for z,alpha in [(z,.27) for z in context]+[(z,.38) for z in wrist]+[(z,.95) for z in rows]+[(z,1.) for z in commons]:
            path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl'];parts.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),color=z.get('color',[.5,.6,.7]),alpha=alpha,neutral_inverse=np.eye(4)))
        return dict(parts=parts,design=g.DESIGN)
    blocked=selected['cases'][-1];q=blocked['angles_deg'];N=g.g.v.frames(g.DESIGN,np.zeros(20));F=g.g.v.frames(g.DESIGN,q);carrier=next(z for z in meta if z['name']=='thumb_root_spread_carrier');moving=c.load([carrier])[0];moving=dict(moving,shape=g.g.v.relocate(moving['shape'],F[carrier['body']],N[carrier['body']]))
    housing=c.load([next(z for z in last if z['name']=='wrist_local_housing_0')])[0];block=housing['shape'].intersect(moving['shape'],tol=1e-6);expected=next(z['material_intersection_mm3'] for z in blocked['native']['hand']['overlaps'] if z['lead']=='wrist_local_housing_0');assert abs(block.Volume()-expected)<1e-5
    ev=export([dict(name='quarter_posed_thumb_carrier',shape=moving['shape'],body='palm',color=[.38,.58,.63],installed_part=False),dict(name='actual_parent_quarter_housing_common',shape=block,body='palm',color=[.96,.02,.03],installed_part=False)],HERE/'generated/parent_block');evmeta=p.read(ROOT/ev)['parts']
    quarter=make(last,evmeta);quarter['parts']=[z for z in quarter['parts'] if z['name']!='thumb_root_spread_carrier']
    scenes=[make(straight),make(last),make(first,cm),quarter];fit=make(last);fit['parts']=[z for z in fit['parts'] if z['name']!='Palm_tapered_chassis'];view=r.view_fit([fit],[dict(angles_deg=[0.]*20)],np.array([1.,-1.8,.6]),1.);ims=[]
    try:
        for scene in scenes:
            scene['_render_state']=r.make_renderer(scene,{'v':view},1100,engine)
            for j,z in enumerate(scene['parts']):scene['_render_state']['model'].geom_rgba[j,3]=z['alpha']
            im,_=r.render_one(scene,[0.]*20,'v',{'v':view},1100,engine);ims.append(im)
        r.sheet([('기존 창 정면 통과: 앞쪽 손바닥/엄지 뿌리에 걸림',ims[0]),('같은 창 45도 통과 + 아래쪽 손목 벽의 분리형 안내대',ims[1])],2,1100,'손을 깎지 않고 실제 손목 창과 지지 방향 확인','보라: 국부 피복 2개 · 청록: 실제 안내대 후보 · 회색: 기존 손목 · 긴 전체 연결/왕복운동·장력은 아직 미검증').save(out/'window_path.png')
        r.sheet([('첫 죔쇠: 앞 덮개와 엄지 뿌리의 실제 공통 재료',ims[2]),('위 안내관은 유지, 죔쇠만 아래 손목 벽으로 이동',ims[1])],2,1100,'추가 고정부가 만든 실제 걸림과 설치 위치 교정','빨강은 첫 덮개/엄지 뿌리의 실제 공통 체적 '+format(common.Volume(),'.9f')+'mm³ · 손 본체/관절/모터/전기선은 유지 · 외형·강도 수용은 미정').save(out/'mount_correction.png')
        r.sheet([('편 손: 국부 직선 피복과 손 간섭0',ims[1]),('부모25%: 직선 피복이 움직인 엄지 뿌리에 걸림',ims[3])],2,1100,'안내대 설치 가능과 피복 운동 성공은 다름','빨강은 실제 피복/부모25% 공통 체적 '+format(block.Volume(),'.9f')+'mm³ · 두 피복 각각 같은 걸림 · 경로 미채택/긴 연결·장력 미완').save(out/'parent_block.png')
    finally:
        for scene in scenes:
            if '_render_state' in scene:scene['_render_state']['renderer'].close()
    p.write(out/'report.json',dict(completed=True,source_sha256=source,outputs={z.name:r.sha(z) for z in out.iterdir() if z.is_file()},actual_common_mm3=common.Volume(),parent_quarter_actual_common_mm3=block.Volume(),local_housing_route_adopted=False,actual_saved_CAD_meshes=True,physical_guide_candidate_drawn=True,long_housing_connected=False,wrist_motion_tested=False))
if __name__=='__main__':main()
