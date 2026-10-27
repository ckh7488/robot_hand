"""Render stored results without repeating the completed native groups."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import details_render as d
a,m,g,c,p,np,cq,ROOT,HERE=d.a,d.m,d.g,d.c,d.p,d.np,d.cq,d.ROOT,d.HERE
r,engine=d.r,d.engine
from PIL import ImageDraw

def main():
    raw=p.read(HERE/'reviews/check.json');case=raw['cases'][0];details=p.read(HERE/'reviews/details.json');assert details['completed']
    fresh=p.read(ROOT/case['scene'])['parts'];common=p.read(ROOT/details['common_scene'])['parts'];ret=p.read(ROOT/'cad/sheath_retention_v1/selected/generated/parts/scene.json')['parts']
    hand=p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')['hand'];hand=[z for z in hand if z['name'] not in ['thumb_segment2_dark_faceplate_clamp1_bolt','thumb_segment2_dark_faceplate_clamp2_bolt']]
    dist=p.read(ROOT/'cad/thumb_sheath_terminal_v2/selected/generated/candidate_scene.json');hand+=dist['retained_guide_parts']+[z for z in dist['parts'] if z['name'] not in ['distal_flanged_ferrule_0','distal_flanged_ferrule_1']]+[z for z in ret if z['body']=='thumb_mcp'];assert len(hand)==397
    pack=p.read(ROOT/'cad/motor_tendon_terminal_v1/selected/generated/assembly_scene.json');pack=pack['new_parts']+pack['retained_parts']+[z for z in ret if z['body']=='forearm'];assert len(pack)==385
    wrist=p.read(ROOT/'cad/wrist_outlet_route_v1/inboard/generated/assembly_scene.json');hardware=[z for z in wrist['new_parts']+wrist['retained_parts'] if z['installed_part_candidate']]
    wires=p.read(ROOT/'cad/remote_motor_lead_v2/generated/scene.json');elect=wires['parts'];assert len(elect)==64
    arm=[]
    for z in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference']:
        path=ROOT/'cad/remote_bus_v1/generated/arm_visual_meshes'/(z['name']+'.stl');assert path.exists();arm.append(dict(name=z['name'],mesh=path.relative_to(ROOT).as_posix(),sha256_stl=m.drive.sha(path),color=[.5,.6,.7]))
    context=hand+pack+hardware+elect+arm;source={}
    def scene(extra,ctx):
        parts=[]
        for z,alpha in [(z,.18) for z in ctx]+[(z,1. if z['name'].startswith('actual_') else .86) for z in extra]:
            path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl'];parts.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),color=z.get('color',[.57,.63,.69]),alpha=alpha,neutral_inverse=np.eye(4)))
        return dict(parts=parts,design=g.DESIGN)
    before=scene([z for z in wrist['new_parts'] if not z['installed_part_candidate']],context);after=scene(fresh+common,context)
    arms=scene([z for z in fresh if z['segment'] in [3,4]]+[z for z in common if z['name']=='actual_arm_common'],[z for z in arm if z['name'] in ['AK_wrist_deviation','Wrist_box_carrier','Palm_wrist_saddle']]+hardware)
    distal=scene([z for z in fresh if z['segment']>=4]+[z for z in common if z['name']=='actual_hand_common'],[z for z in hand if z['body']=='palm' or z['body'].startswith('thumb')]+hardware)
    scenes=[before,after,arms,distal];view=r.view_fit([before,after],[dict(angles_deg=[0.]*20)]*2,np.array([1.,-.65,.45]),1.)
    views=[view,view]+[r.view_fit([s],[dict(angles_deg=[0.]*20)],np.array([.6,-1.,.7]),1.) for s in [arms,distal]];ims=[]
    try:
        for s,v in zip(scenes,views):
            s['_render_state']=r.make_renderer(s,{'v':v},1000,engine)
            for j,z in enumerate(s['parts']):s['_render_state']['model'].geom_rgba[j,3]=z['alpha']
            im,_=r.render_one(s,[0.]*20,'v',{'v':v},1000,engine)
            if len(ims)>=2:
                hit=next(z for z in common if z['name']==('actual_arm_common' if len(ims)==2 else 'actual_hand_common'))
                center=r.read_stl_points(ROOT/hit['mesh']).mean(axis=0)-v['focus'];scale=1000/(2*v['parallel_scale_mm']);x=500+center@v['right']*scale;y=500-center@v['up']*scale
                ImageDraw.Draw(im).ellipse((x-30,y-30,x+30,y+30),outline=(215,10,30),width=4)
            ims.append(im)
        r.sheet([('이전: 양끝과 손목 국부 참고만 있음',ims[0]),('이번: 전체 연결 후보,실제 관통으로 미채택',ims[1])],2,1000,
                '모터 → 실제 손목 안내대 → 엄지 종단 · 전체 연결 연구',
                '보라: 피복2개 · 주황: 줄2개 · 빨강: 실제 공통 재료 · 일정 길이 수식만으로 구동 성공을 주장하지 않음').save(HERE/'images/link_before_after.png')
        r.sheet([('안내대 뒤: 손목 구동부를 관통',ims[2]),('안내대 앞: 엄지 뿌리를 관통',ims[3])],2,1000,
                '같은 손/모터에서 남은 실제 전체 경로 걸림',
                '단일 편 손 후보 · 급한 굽힘/두 피복 간 겹침도 있어 미채택 · 손을 깎거나 각도 제한할 근거가 아님').save(HERE/'images/actual_block.png')
    finally:
        for s in scenes:
            if '_render_state' in s:s['_render_state']['renderer'].close()
    p.write(HERE/'images/report.json',dict(completed=True,source_sha256=source,outputs={z.name:r.sha(z) for z in (HERE/'images').iterdir() if z.is_file()},new_native_groups=0,original_render_KeyError_preserved=True,actual_saved_CAD=True,whole_hand_drive_approved=False))
if __name__=='__main__':main()
