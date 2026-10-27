"""Actual old crossing and new connected rear channels, plus exterior context."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import check_pair as b
a,m,g,c,p,np,cq,ROOT,HERE=b.a,b.m,b.g,b.c,b.p,b.np,b.cq,b.ROOT,b.HERE
helper=m.drive.module('_pair034_render',ROOT/'cad/thumb_root_guide_v1/guide_render.py');r,engine=helper.r,helper.engine

def main():
    static=p.read(HERE/'reviews/static.json');service=p.read(HERE/'reviews/service.json');assert static['sampled_candidate_clear'] and service['required_hardware_paths_clear'] and service['prescribed_feed_union_clear'];source={}
    hand=p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')['hand'];hand=[z for z in hand if z['name'] not in ['thumb_segment2_dark_faceplate_clamp1_bolt','thumb_segment2_dark_faceplate_clamp2_bolt']]
    dist=p.read(ROOT/'cad/thumb_sheath_terminal_v2/selected/generated/candidate_scene.json');ret=p.read(ROOT/'cad/sheath_retention_v1/selected/generated/parts/scene.json')['parts'];hand+=dist['retained_guide_parts']+[z for z in dist['parts'] if z['name'] not in ['distal_flanged_ferrule_0','distal_flanged_ferrule_1']]+[z for z in ret if z['body']=='thumb_mcp'];assert len(hand)==397
    pack=p.read(ROOT/'cad/motor_tendon_terminal_v1/selected/generated/assembly_scene.json');pack=pack['new_parts']+pack['retained_parts']+[z for z in ret if z['body']=='forearm'];assert len(pack)==385
    elec=p.read(ROOT/'cad/remote_motor_lead_v2/generated/scene.json')['parts'];arm=[]
    for z in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference']:
        if z['name'] not in ['AK_wrist_flex','AK_wrist_deviation','Wrist_box_carrier','Palm_wrist_saddle']:continue
        path=ROOT/'cad/remote_bus_v1/generated/arm_visual_meshes'/(z['name']+'.stl');arm.append(dict(name=z['name'],mesh=path.relative_to(ROOT).as_posix(),sha256_stl=m.drive.sha(path),color=[.5,.6,.7]))
    old=p.read(ROOT/'cad/wrist_external_feed_v1/generated/assembly_scene.json');old=old['new_parts']+old['retained_parts'];fresh=p.read(HERE/'generated/assembly_scene.json');fresh=fresh['new_parts']+fresh['retained_parts']
    failed=p.read(ROOT/'cad/elastic_housing_link_v1/generated/pose_0/scene.json')['parts'];failed=[z for z in failed if z.get('kind')=='housing' and z.get('segment') in [9,10,11,12]]
    common=p.read(ROOT/'cad/elastic_housing_link_v1/generated/actual_common/scene.json')['parts'];common=[z for z in common if z['name']=='actual_internal_common']
    def scene(hardware,before=False,whole=False):
        parts=[]
        def add(z,alpha):
            path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl'];parts.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),color=z.get('color',[.58,.64,.7]),alpha=alpha,neutral_inverse=np.eye(4)))
        if whole:
            for z in hand:add(z,.28)
            for z in arm:add(z,.22)
            for z in pack:add(z,.3)
            for z in elec:add(z,.45)
        for z in hardware:add(z,.85 if z.get('installed_part_candidate') else .95)
        if before:
            for z in failed:add(z,.85)
            for z in common:add(z,1.)
        return dict(parts=parts,design=g.DESIGN)
    scenes=[scene(old,True),scene(fresh),scene(fresh,whole=True)];detail=r.view_fit(scenes[:2],[dict(angles_deg=[0.]*20)],np.array([.7,-.7,1.2]),1.);whole=r.view_fit([scenes[2]],[dict(angles_deg=[0.]*20)],np.array([.7,-.5,.7]),1.);ims=[]
    try:
        for j,s in enumerate(scenes):
            view=whole if j==2 else detail;s['_render_state']=r.make_renderer(s,{'v':view},1100,engine)
            for k,z in enumerate(s['parts']):s['_render_state']['model'].geom_rgba[k,3]=z['alpha']
            im,_=r.render_one(s,[0.]*20,'v',{'v':view},1100,engine);ims.append(im)
        folder=HERE/'images';folder.mkdir(exist_ok=False)
        r.sheet([('이전: 독립 피복 두 선의 뒤쪽 실제 교차',ims[0]),('이번: 두 S통로가 몸체에 이어진 짝안내대',ims[1])],2,1100,'손목 뒤 피복을 따로 받는 실제 지지 후보','청록: 안내대 · 보라: 피복 · 두 새 뒤입구 X90/110 · 기존 통로 X100 유지 · 전체 긴 피복/엄지 외피 문제는 아직 미해결').save(folder/'pair_before_after.png')
        r.sheet([('손·모터를 유지한 새 후보의 실제 돌출',ims[2])],1,1100,'손목 받침에 붙은 긴 짝안내대의 외형 후보','손바닥 최대 X90 대비 안내관 최대 X112.5 · 뒤입구 Y−178.730 · 외형 수용/강도 미정 · 전체 구동 완성품이 아님').save(folder/'pair_hand.png')
    finally:
        for s in scenes:
            if '_render_state' in s:s['_render_state']['renderer'].close()
    source['cad/paired_rear_guide_v1/render_pair.py']=m.drive.sha(Path(__file__));p.write(HERE/'images/report.json',dict(completed=True,source_sha256=source,outputs={z.name:r.sha(z) for z in (HERE/'images').iterdir() if z.is_file()},actual_saved_CAD=True,old_rejected_curves_visual_only=True,whole_link_connected=False,no_motor_tendon_force_simulated=True))
if __name__=='__main__':main()
