"""Actual thenar penetration before and attached terminal elbow after."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import check_entry as b
a,m,g,c,p,np,cq,ROOT,HERE=b.a,b.m,b.g,b.c,b.p,b.np,b.cq,b.ROOT,b.HERE
helper=m.drive.module('_entry035_render',ROOT/'cad/thumb_root_guide_v1/guide_render.py');r,engine=helper.r,helper.engine
def main():
    static=p.read(HERE/'reviews/static.json');service=p.read(HERE/'reviews/service.json');assert static['sampled_candidate_clear'] and service['hand_clear'] and service['feeding_clear'];source={}
    hand=p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')['hand'];hand=[z for z in hand if z['name'] not in ['thumb_segment2_dark_faceplate_clamp1_bolt','thumb_segment2_dark_faceplate_clamp2_bolt']];dist=p.read(ROOT/'cad/thumb_sheath_terminal_v2/selected/generated/candidate_scene.json');ret=p.read(ROOT/'cad/sheath_retention_v1/selected/generated/parts/scene.json')['parts'];hand+=dist['retained_guide_parts']+[z for z in dist['parts'] if z['name'] not in ['distal_flanged_ferrule_0','distal_flanged_ferrule_1']]+[z for z in ret if z['body']=='thumb_mcp'];assert len(hand)==397
    pack=p.read(ROOT/'cad/motor_tendon_terminal_v1/selected/generated/assembly_scene.json');pack=pack['new_parts']+pack['retained_parts']+[z for z in ret if z['body']=='forearm'];elec=p.read(ROOT/'cad/remote_motor_lead_v2/generated/scene.json')['parts'];w=p.read(ROOT/'cad/paired_rear_guide_v1/generated/assembly_scene.json');wrist=[z for z in w['new_parts']+w['retained_parts'] if z['installed_part_candidate']];fresh=p.read(HERE/'generated/parts/scene.json')['parts'];failed=p.read(ROOT/'cad/elastic_housing_link_v1/generated/pose_0/scene.json')['parts'];failed=[z for z in failed if z.get('kind')=='housing' and z.get('physical_line')==1 and z.get('segment') in [17,18,19,20,21,22]];common=[z for z in p.read(ROOT/'cad/elastic_housing_link_v1/generated/actual_common/scene.json')['parts'] if z['name']=='actual_hand_common']
    def scene(before=False,whole=False):
        parts=[]
        def add(z,alpha):
            path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl'];parts.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),color=z.get('color',[.58,.64,.7]),alpha=alpha,neutral_inverse=np.eye(4)))
        for z in hand:
            if not whole and not (z['body']=='palm' or z['body'].startswith('thumb')):continue
            if not before and z['name']==a.REPLACED:continue
            add(z,.22 if z['body']=='palm' else .28)
        if whole:
            for z in pack:add(z,.26)
            for z in elec:add(z,.4)
            for z in wrist:add(z,.85)
        if before:
            for z in failed:add(z,.9)
            for z in common:add(z,1.)
        else:
            for z in fresh:add(z,.82 if z['installed_part_candidate'] else .95)
        return dict(parts=parts,design=g.DESIGN)
    scenes=[scene(True),scene(),scene(whole=True)];detail=r.view_fit(scenes[:2],[dict(angles_deg=[0.]*20)],np.array([.8,-1.,.7]),1.);whole=r.view_fit([scenes[2]],[dict(angles_deg=[0.]*20)],np.array([.8,-.5,.7]),1.);ims=[]
    try:
        for j,s in enumerate(scenes):
            view=whole if j==2 else detail;s['_render_state']=r.make_renderer(s,{'v':view},1100,engine)
            for k,z in enumerate(s['parts']):s['_render_state']['model'].geom_rgba[k,3]=z['alpha']
            im,_=r.render_one(s,[0.]*20,'v',{'v':view},1100,engine);ims.append(im)
        folder=HERE/'images';folder.mkdir(exist_ok=False)
        r.sheet([('이전 실패: 자유 피복과 엄지 외피의 실제 관통',ims[0]),('이번: 위쪽 죔쇠에 붙인 R20 굽힘 안내관',ims[1])],2,1100,'엄지 끝구에 들어가기 전 방향을 잡는 실제 지지 후보','청록: 새 안내관/기존 죔쇠 한쪽의 일체 형상 · 보라: 국부 피복 · 손/축/끝구 위치 유지 · 전체 긴피복은 아직 미연결').save(folder/'entry_before_after.png')
        r.sheet([('새 뒤 짝안내대와 엄지 종단 굽힘 지지의 외형 후보',ims[2])],1,1100,'모터~손목~엄지 사이의 전체 유연 연결은 아직 없음','손/모터/전기선 유지 · 국부 피복/줄 참고만 표시 · 추가 외형/제조·강도·마찰/장력 미검증').save(folder/'entry_hand.png')
    finally:
        for s in scenes:
            if '_render_state' in s:s['_render_state']['renderer'].close()
    source['cad/thumb_entry_bend_v1/render_entry.py']=m.drive.sha(Path(__file__));p.write(HERE/'images/report.json',dict(completed=True,source_sha256=source,outputs={z.name:r.sha(z) for z in (HERE/'images').iterdir() if z.is_file()},actual_saved_CAD=True,old_failed_curve_visual_only=True,no_motor_or_force_simulation=True,whole_link_connected=False))
if __name__=='__main__':main()
