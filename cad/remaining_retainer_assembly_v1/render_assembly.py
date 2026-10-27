"""Display the actual moving and then-installed parts of two local paths."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import remaining_service as a
m,g,p,np,ROOT,HERE=a.m,a.g,a.p,a.np,a.ROOT,a.HERE
helper=m.drive.module('_assembly032_render',ROOT/'cad/thumb_root_guide_v1/guide_render.py');r,engine=helper.r,helper.engine

def main():
    report=p.read(HERE/'reviews/selected_service.json');raw=p.read(HERE/'reviews/service.json');source={};hand=p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')['hand'];hand=[z for z in hand if z['name'] not in ['thumb_segment2_dark_faceplate_clamp1_bolt','thumb_segment2_dark_faceplate_clamp2_bolt']]
    dist=p.read(ROOT/'cad/thumb_sheath_terminal_v2/selected/generated/candidate_scene.json');ret=p.read(ROOT/'cad/sheath_retention_v1/selected/generated/parts/scene.json')['parts'];hand+=dist['retained_guide_parts']+[z for z in dist['parts'] if z['name'] not in ['distal_flanged_ferrule_0','distal_flanged_ferrule_1']]+[z for z in ret if z['body']=='thumb_mcp']
    pack=p.read(ROOT/'cad/motor_tendon_terminal_v1/selected/generated/assembly_scene.json');pack=pack['new_parts']+pack['retained_parts']+[z for z in ret if z['body']=='forearm'];housing=p.read(ROOT/'cad/thumb_sheath_terminal_v2/selected/generated/housing_reference/scene.json')['parts']+p.read(ROOT/'cad/motor_tendon_terminal_v1/selected/generated/housing_reference/scene.json')['parts']
    original=p.read(ROOT/'cad/thumb_guide_assembly_v2/generated/parts/scene.json')['parts'];lookup={z['name']:z for z in hand+pack+housing+original};scenes=[];labels=[];selection=[]
    common=p.read(ROOT/report['actual_failed_common_scene'])['parts']
    specifications=[(raw,'lower_captured_ferrule_and_clamp','jaw_1',[0]),(report,'lower_preassembled_kit','captured_ferrule_clamp_kit_axial',[0,4])]
    for recorded,studyid,pathid,indices in specifications:
        st=next(z for z in recorded['studies'] if z['id']==studyid);path=next(z for z in st['paths'] if z['id']==pathid)
        for j in indices:
            parts=[];installed=set(path['obstacle_names']);moving=set(path['moving']);sample=path['samples'][j]
            for n in sorted((installed|moving)&set(lookup)):
                z=lookup[n]
                visible=z.get('body','').startswith('thumb')
                if not visible and n not in moving:continue
                T=np.eye(4)
                if n in moving:T[:3,3]=sample['translation_world_mm']
                mesh=ROOT/z['mesh'];assert r.sha(mesh)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl']
                parts.append(dict(name=n,body='palm',mesh_path=mesh,points=r.read_stl_points(mesh),color=[.96,.52,.13] if n in moving else z.get('color',[.55,.62,.7]),alpha=1. if n in moving else .26,neutral_inverse=T))
            if pathid=='jaw_1':
                for z in common:
                    mesh=ROOT/z['mesh'];assert r.sha(mesh)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl'];parts.append(dict(name=z['name'],body='palm',mesh_path=mesh,points=r.read_stl_points(mesh),color=[1.,.03,.02],alpha=1.,neutral_inverse=np.eye(4)))
            assert parts;scenes.append(dict(parts=parts,design=g.DESIGN));selection.append(dict(study=studyid,path=pathid,sample=j,translation_world_mm=sample['translation_world_mm'],displayed_names=[z['name'] for z in parts],actual_obstacle_names=path['obstacle_names']))
            labels.append('첫 실패: 죔쇠 후삽입이 손가락을 관통' if pathid=='jaw_1' else '교정: 손 밖에서 맞춘 8행 묶음 진입' if j==0 else '같은 묶음: 축 방향 착좌 위치')
    view=r.view_fit(scenes,[dict(angles_deg=[0.]*20)],np.array([1.,-.8,.5]),1.);ims=[]
    try:
        for j,sc in enumerate(scenes):
            sc['_render_state']=r.make_renderer(sc,{'v':view},1000,engine)
            for k,z in enumerate(sc['parts']):sc['_render_state']['model'].geom_rgba[k,3]=z['alpha']
            im,_=r.render_one(sc,[0.]*20,'v',{'v':view},1000,engine);ims.append(im)
        folder=HERE/'images';folder.mkdir(exist_ok=False)
        r.sheet(list(zip(labels,ims)),3,1000,'아래 죔쇠: 부품을 깎지 않고 조립 순서를 교정','빨강: 실제 공통 재료 · 주황: 움직이는 부품/묶음 · 단계별 국부 부품만 표시 · 현재 전체 부품 간섭 대조 · 긴 피복/줄·장력은 미완').save(folder/'assembly_paths.png')
    finally:
        for sc in scenes:
            if '_render_state' in sc:sc['_render_state']['renderer'].close()
    p.write(HERE/'images/report.json',dict(completed=True,source_sha256=source,outputs={z.name:r.sha(z) for z in (HERE/'images').iterdir() if z.is_file()},selections=selection,actual_saved_CAD=True,local_context_only=True,no_geometry_change_or_motion_drive_simulated=True))
if __name__=='__main__':main()
