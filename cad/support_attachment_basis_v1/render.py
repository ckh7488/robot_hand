"""Actual saved before/after CAD; long failed routes are not installed."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import bend_model as b
a,m,g,c,p,np,cq,ROOT,HERE=b.a,b.m,b.g,b.c,b.p,b.np,b.cq,b.ROOT,b.HERE
helper=m.drive.module('_support038_render',ROOT/'cad/thumb_root_guide_v1/guide_render.py');r,engine=helper.r,helper.engine

def main():
    raw=p.read(HERE/'reviews/check.json');source=dict(raw['source_sha256']);ret=p.read(ROOT/'cad/sheath_retention_v1/selected/generated/parts/scene.json')['parts'];ps=p.read(ROOT/'cad/motor_tendon_terminal_v1/selected/generated/assembly_scene.json');pack=ps['new_parts']+ps['retained_parts']+[z for z in ret if z['body']=='forearm'];rows=p.read(ROOT/raw['scene'])['parts'];removed=[z['replaced'] for z in raw['design']];after=[z for z in pack if z['name'] not in removed]+rows
    def parts(meta,local=False):
        result=[]
        for z in meta:
            path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl'];pts=r.read_stl_points(path)
            if local:
                box=np.array([np.min(pts,axis=0),np.max(pts,axis=0)]);bounds=np.array([[-20,-260,-240],[40,-190,-110]])
                if np.any(box[1]<bounds[0]) or np.any(box[0]>bounds[1]):continue
            result.append(dict(name=z['name'],body='palm',mesh_path=path,points=pts,color=z.get('color',[.56,.64,.7]),alpha=.9 if z.get('installed_part_candidate') else .65,neutral_inverse=np.eye(4)))
        return result
    before_scene=dict(parts=parts(pack,True),design=g.DESIGN);after_scene=dict(parts=parts(after,True),design=g.DESIGN);view=r.view_fit([before_scene,after_scene],[dict(angles_deg=[0.]*20)],np.array([.8,-.9,.5]),1.)
    def render(scene,view,size=850):
        try:
            scene['_render_state']=r.make_renderer(scene,{'v':view},size,engine)
            for j,z in enumerate(scene['parts']):scene['_render_state']['model'].geom_rgba[j,3]=z['alpha']
            return r.render_one(scene,[0.]*20,'v',{'v':view},size,engine)[0]
        finally:
            if '_render_state' in scene:scene['_render_state']['renderer'].close()
    out=HERE/'images';out.mkdir(exist_ok=False)
    r.sheet([('이전: 모터 피복 종단과 기존 두 죠',render(before_scene,view)),('새 후보: 아래 죠2개에 고정 곡선 보어 일체화',render(after_scene,view))],2,850,'실제 모터 고정부에서 두 피복의 출발 방향을 안내','R40/90° + 직선12mm · 추가 체결품0 · 보라/주황은 국부 피복/중심줄 참고 · 전체 긴선 연결/교차 해결 승인 아님').save(out/'motor_before_after.png')
    hand=p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')['hand'];hand=[z for z in hand if z['name'] not in ['thumb_segment2_dark_faceplate_clamp1_bolt','thumb_segment2_dark_faceplate_clamp2_bolt']];offset=p.read(ROOT/'cad/thumb_sheath_terminal_v2/selected/generated/candidate_scene.json');hand+=offset['retained_guide_parts']+[z for z in offset['parts'] if z['name'] not in ['distal_flanged_ferrule_0','distal_flanged_ferrule_1']]+[z for z in ret if z['body']=='thumb_mcp'];hand=[z for z in hand if z['name']!=a.a.REPLACED]+[z for z in p.read(ROOT/'cad/thumb_entry_bend_v1/generated/parts/scene.json')['parts'] if z['installed_part_candidate']];assert len(hand)==397
    elect=p.read(ROOT/'cad/remote_motor_lead_v2/generated/scene.json')['parts'];ws=p.read(ROOT/'cad/paired_rear_guide_v1/generated/assembly_scene.json');wrist=[z for z in ws['new_parts']+ws['retained_parts'] if z['installed_part_candidate']];arm=[]
    for z in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference']:
        rel='cad/modular_roots_v9/meshes/'+z['asset']+'.stl';arm.append(dict(name=z['name'],mesh=rel,sha256_stl=r.sha(ROOT/rel),color=[.6,.65,.7]))
    whole=dict(parts=parts(hand+after+elect+arm+wrist),design=g.DESIGN);wv=r.view_fit([whole],[dict(angles_deg=[0.]*20)],np.array([.8,-.9,.5]),1.)
    r.sheet([('손397/포장385/전기64/팔10/PAIR6 · 실제 후보 외형',render(whole,wv,1000))],1,1000,'기존 손과 모터 배치를 유지한 새 모터 굽힘 안내관','이전 긴 실패 피복/증거 공통재료 미설치 · 전체 구동/물성·강도·외형 수용은 미검증').save(out/'hand_candidate.png')
    source['cad/support_attachment_basis_v1/render.py']=m.drive.sha(Path(__file__));a.save(out/'report.json',dict(completed=True,source_sha256=source,outputs={z.name:r.sha(z) for z in out.iterdir() if z.is_file()},actual_saved_CAD_before_after=True,old_long_failed_routes_not_installed=True,native_recomputed=0,whole_hand_drive_approved=False))
if __name__=='__main__':main()
