"""Actual rear passage and new supported sleeves, with bounded wrist poses."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import check_feed as b
a,m,g,c,p,np,cq,ROOT,HERE=b.a,b.m,b.g,b.c,b.p,b.np,b.cq,b.ROOT,b.HERE
helper=m.drive.module('_feed031_render',ROOT/'cad/thumb_root_guide_v1/guide_render.py');r,engine=helper.r,helper.engine

def main():
    static=p.read(HERE/'reviews/final.json');service=p.read(HERE/'reviews/service.json');assert static['sampled_candidate_clear'] and service['required_sampled_hardware_paths_clear'] and service['straight_end_continuous_axial_stroke_clear'];source={}
    hand=p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')['hand'];hand=[z for z in hand if z['name'] not in ['thumb_segment2_dark_faceplate_clamp1_bolt','thumb_segment2_dark_faceplate_clamp2_bolt']]
    dist=p.read(ROOT/'cad/thumb_sheath_terminal_v2/selected/generated/candidate_scene.json');ret=p.read(ROOT/'cad/sheath_retention_v1/selected/generated/parts/scene.json')['parts'];hand+=dist['retained_guide_parts']+[z for z in dist['parts'] if z['name'] not in ['distal_flanged_ferrule_0','distal_flanged_ferrule_1']]+[z for z in ret if z['body']=='thumb_mcp'];assert len(hand)==397
    pack=p.read(ROOT/'cad/motor_tendon_terminal_v1/selected/generated/assembly_scene.json');pack=pack['new_parts']+pack['retained_parts']+[z for z in ret if z['body']=='forearm'];assert len(pack)==385
    elec=p.read(ROOT/'cad/remote_motor_lead_v2/generated/scene.json')['parts'];arm=[]
    for z in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference']:
        if z['name'] not in ['AK_wrist_flex','AK_wrist_deviation','Wrist_box_carrier','Palm_wrist_saddle']:continue
        path=ROOT/'cad/remote_bus_v1/generated/arm_visual_meshes'/(z['name']+'.stl');arm.append(dict(name=z['name'],mesh=path.relative_to(ROOT).as_posix(),sha256_stl=m.drive.sha(path),color=[.5,.6,.7]))
    old=p.read(ROOT/'cad/wrist_outlet_route_v1/inboard/generated/assembly_scene.json');old=old['new_parts']+old['retained_parts'];fresh=p.read(HERE/'generated/assembly_scene.json');fresh=fresh['new_parts']+fresh['retained_parts']
    failed=p.read(ROOT/'cad/guided_housing_link_v1/generated/pose_0/scene.json')['parts'];failed=[z for z in failed if z['segment']==3 and z['kind']=='housing'];common=p.read(ROOT/'cad/guided_housing_link_v1/generated/actual_common/scene.json')['parts'];common=[z for z in common if z['name']=='actual_arm_common']
    def scene(hardware,flex=0,yaw=0,before=False,whole=False):
        P,H=a.wrist_frames(flex,yaw);parts=[]
        def add(z,alpha,T):
            path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl'];parts.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),color=z.get('color',[.58,.64,.7]),alpha=alpha,neutral_inverse=T))
        for z in hand:
            if whole or z['body']=='palm' or z['body'].startswith('thumb'):add(z,.18,H)
        for z in arm:add(z,.2,H if z['name']=='Palm_wrist_saddle' else P if z['name'] in ['Wrist_box_carrier','AK_wrist_deviation'] else np.eye(4))
        if whole:
            for z in pack:add(z,.3,H if z.get('body')=='palm' else np.eye(4))
            for z in elec:add(z,.5,np.eye(4))
        for z in hardware:add(z,.95,H)
        if before:
            for z in failed:add(z,.9,np.eye(4))
            for z in common:add(z,1.,np.eye(4))
        return dict(parts=parts,design=g.DESIGN)
    scenes=[scene(old,before=True),scene(fresh),scene(fresh,20,15),scene(fresh,whole=True)];detail=r.view_fit(scenes[:3],[dict(angles_deg=[0.]*20)],np.array([.7,-1.,.5]),1.);whole=r.view_fit([scenes[3]],[dict(angles_deg=[0.]*20)],np.array([.7,-.5,.5]),1.);ims=[]
    try:
        for j,s in enumerate(scenes):
            view=whole if j==3 else detail;s['_render_state']=r.make_renderer(s,{'v':view},1000,engine)
            for k,z in enumerate(s['parts']):s['_render_state']['model'].geom_rgba[k,3]=z['alpha']
            im,_=r.render_one(s,[0.]*20,'v',{'v':view},1000,engine);ims.append(im)
        folder=HERE/'images';folder.mkdir(exist_ok=False)
        r.sheet([('이전 실패: 뒤쪽 구동부를 관통한 긴 곡선',ims[0]),('이번: 실제 받침과 바깥쪽 긴 안내관',ims[1]),('참고 손목 굽힘20° / 편위15°의 새 구성',ims[2])],3,1000,
                '손목 구동부 바깥의 실제 피복 지지 후보',
                '청록: 새 일체 안내대 · 보라:118mm 직선 피복 끝참고 · 기존 손/모터 유지 · 모터~엄지 전체 긴 피복·장력은 미완').save(folder/'feed_before_after.png')
        r.sheet([('새 안내대가 추가된 손/원격모터 배치',ims[3]),('손목의 실제 부착과 두 안내관',ims[1])],2,1000,
                '손목 뒤 진입을 위한 명목 하드웨어 · 전체 구동 완성품이 아님',
                '손바닥 외곽보다 오른쪽12.5mm/뒤로96mm의 안내관 후보 · 외형 수용/강도·마찰 미검증 · 긴 피복은 아직 양끝 미연결').save(folder/'feed_hand.png')
    finally:
        for s in scenes:
            if '_render_state' in s:s['_render_state']['renderer'].close()
    p.write(HERE/'images/report.json',dict(completed=True,source_sha256=source,outputs={z.name:r.sha(z) for z in (HERE/'images').iterdir() if z.is_file()},actual_saved_CAD=True,wrist_display_flex_yaw_deg=[20,15],current_palm39_packaging_moves_with_hand=True,no_motor_or_tendon_drive_simulated=True,whole_hand_drive_approved=False))
if __name__=='__main__':main()
