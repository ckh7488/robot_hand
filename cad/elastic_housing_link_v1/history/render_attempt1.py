"""Actual saved CAD connection and common material, with correct body frames."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import check_elastic as b
a,m,g,p,np,ROOT,HERE=b.a,b.m,b.g,b.p,b.np,b.ROOT,b.HERE
helper=m.drive.module('_elastic033_render',ROOT/'cad/thumb_root_guide_v1/guide_render.py');r,engine=helper.r,helper.engine
from PIL import ImageDraw

def main():
    raw=p.read(HERE/'reviews/check.json');details=p.read(HERE/'reviews/details.json');case=next(z for z in raw['native_cases'] if z['id']==details['displayed_case_id']);fresh=p.read(ROOT/case['scene'])['parts'];common=p.read(ROOT/details['common_scene'])['parts'] if details['common_scene'] else [];ret=p.read(ROOT/'cad/sheath_retention_v1/selected/generated/parts/scene.json')['parts'];source={}
    hand=p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')['hand'];hand=[z for z in hand if z['name'] not in ['thumb_segment2_dark_faceplate_clamp1_bolt','thumb_segment2_dark_faceplate_clamp2_bolt']];dist=p.read(ROOT/'cad/thumb_sheath_terminal_v2/selected/generated/candidate_scene.json');hand+=dist['retained_guide_parts']+[z for z in dist['parts'] if z['name'] not in ['distal_flanged_ferrule_0','distal_flanged_ferrule_1']]+[z for z in ret if z['body']=='thumb_mcp'];assert len(hand)==397
    pack=p.read(ROOT/'cad/motor_tendon_terminal_v1/selected/generated/assembly_scene.json');pack=pack['new_parts']+pack['retained_parts']+[z for z in ret if z['body']=='forearm'];assert len(pack)==385
    wrist=p.read(ROOT/'cad/wrist_external_feed_v1/generated/assembly_scene.json');hardware=[z for z in wrist['new_parts']+wrist['retained_parts'] if z['installed_part_candidate']];local=[z for z in wrist['new_parts'] if not z['installed_part_candidate']];hous=p.read(ROOT/'cad/thumb_sheath_terminal_v2/selected/generated/housing_reference/scene.json')['parts']+p.read(ROOT/'cad/motor_tendon_terminal_v1/selected/generated/housing_reference/scene.json')['parts'];elect=p.read(ROOT/'cad/remote_motor_lead_v2/generated/scene.json')['parts'];arm=[]
    for z in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference']:
        path=ROOT/'cad/remote_bus_v1/generated/arm_visual_meshes'/(z['name']+'.stl');arm.append(dict(name=z['name'],mesh=path.relative_to(ROOT).as_posix(),sha256_stl=m.drive.sha(path),color=[.5,.6,.7]))
    P,H=a.a.wrist_frames(case['wrist_flex_deg'],case['wrist_deviation_deg']);F=g.g.v.frames(g.DESIGN,case['angles_deg']);N=g.g.v.frames(g.DESIGN,np.zeros(20))
    def handT(body):
        O,R=F[body];O0,R0=N[body];T=np.eye(4);T[:3,:3]=R@R0.T;T[:3,3]=O-T[:3,:3]@O0;return H@T
    context=[(z,handT(z['body'])) for z in hand]+[(z,H if z.get('body')=='palm' else np.eye(4)) for z in pack]+[(z,H) for z in hardware]+[(z,np.eye(4)) for z in elect]+[(z,H if z['name']=='Palm_wrist_saddle' else P if z['name'] in ['Wrist_box_carrier','AK_wrist_deviation'] else np.eye(4)) for z in arm]
    def scene(extra,ctx):
        parts=[]
        for z,T,alpha in [(z,T,.17) for z,T in ctx]+[(z,T,1. if z['name'].startswith('actual_') else .86) for z,T in extra]:
            path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl'];parts.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),color=z.get('color',[.57,.63,.69]),alpha=alpha,neutral_inverse=T))
        return dict(parts=parts,design=g.DESIGN)
    before=scene([(z,H) for z in local]+[(z,handT('thumb_mcp') if z.get('body')=='thumb_mcp' else np.eye(4)) for z in hous],context);after=scene([(z,np.eye(4)) for z in fresh+common],context)
    hits={z['lead'] for z in details['actual_pairs']};targets={z['part'] for z in details['actual_pairs']};closeextra=[z for z in fresh if z['name'] in hits or z['name'].replace('_cord_','_housing_') in hits]+common;closecontext=[(z,T) for z,T in context if z['name'] in targets or z['name'].startswith('thumb_') and z.get('body')=='thumb_mcp'];inset=scene([(z,np.eye(4)) for z in closeextra or fresh],closecontext);scenes=[before,after,inset]
    whole=r.view_fit(scenes[:2],[dict(angles_deg=[0.]*20)],np.array([1.,-.65,.45]),1.);close=r.view_fit([inset],[dict(angles_deg=[0.]*20)],np.array([.6,-1.,.7]),1.);ims=[]
    try:
        for j,sc in enumerate(scenes):
            view=close if j==2 else whole;sc['_render_state']=r.make_renderer(sc,{'v':view},1000,engine)
            for k,z in enumerate(sc['parts']):sc['_render_state']['model'].geom_rgba[k,3]=z['alpha']
            im,_=r.render_one(sc,[0.]*20,'v',{'v':view},1000,engine)
            if j==2 and common:
                center=r.read_stl_points(ROOT/common[0]['mesh']).mean(axis=0)-view['focus'];scale=1000/(2*view['parallel_scale_mm']);x=500+center@view['right']*scale;y=500-center@view['up']*scale;ImageDraw.Draw(im).ellipse((x-30,y-30,x+30,y+30),outline=(215,10,30),width=4)
            ims.append(im)
        folder=HERE/'images';folder.mkdir(exist_ok=False)
        r.sheet([('이전: 실제 종단/안내대와 짧은 참고만 있음',ims[0]),('이번: 원호 막대 모델로 이은 전체 기하 후보',ims[1])],2,1000,'모터 → 바깥쪽 손목 안내대 → 엄지 종단','보라: 각650mm가정 피복2 · 주황: 중심줄2 · 빨강: 실제 공통 재료 · 이상화된 굽힘 모델,힘/장력·실물 승인이 아님').save(folder/'link_before_after.png')
        r.sheet([('새 전체 후보의 실제 부품 대조',ims[2])],1,1000,'끝점/일정 길이와 실제 재료 간섭을 구별','원호 접속점은 설치 가이드가 아님 · 실제 공통 재료만 빨강 · 손 전체를 깎을 근거가 아님').save(folder/'actual_block.png')
    finally:
        for sc in scenes:
            if '_render_state' in sc:sc['_render_state']['renderer'].close()
    p.write(HERE/'images/report.json',dict(completed=True,source_sha256=source,outputs={z.name:r.sha(z) for z in folder.iterdir() if z.is_file()},displayed_case_id=case['id'],actual_saved_CAD=True,new_native_groups=0,whole_hand_drive_approved=False))
if __name__=='__main__':main()
