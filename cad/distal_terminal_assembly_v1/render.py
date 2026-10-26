"""Actual failed material and explicit two-piece preassembly/entry stage."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from service import *
helper=m.drive.module('_assembly025_render_helpers',ROOT/'cad/thumb_root_guide_v1/guide_render.py');r,poses,engine=helper.r,helper.poses,helper.engine
from PIL import ImageDraw
def main():
    out=HERE/'images';out.mkdir(exist_ok=False);first=p.read(HERE/'reviews/service.json');selection=p.read(HERE/'reviews/selection.json');chosen=p.read(HERE/selection['selected_report']);candidate=p.read(ROOT/'cad/thumb_sheath_terminal_v2/selected/generated/candidate_scene.json');original=p.read(ROOT/'cad/thumb_guide_assembly_v2/generated/parts/scene.json')['parts'];new=candidate['parts'];housing=p.read(ROOT/candidate['housing_reference'])['parts'];hand=p.read(ROOT/candidate['base_hand_catalog'])['hand'];by={z['name']:z for z in original+new+housing};source={}
    path=next(z for z in first['paths'] if not z['clear']);index=max(range(len(path['samples'])),key=lambda i:sum(z['material_intersection_mm3'] for z in path['samples'][i]['native']['overlaps']));sample=path['samples'][index];name=path['moving'][0];base=c.load([by[name]])[0]['shape'];W=g.original.WORLD@np.asarray(sample['transform_local'])@g.original.INV;bad=g.original.transform(base,W);rootmeta=next(z for z in hand if z['name']=='thumb_segment1_thenar_shell');root=c.load([rootmeta])[0]['shape'];common=bad.intersect(root,tol=1e-6);want=next(z['material_intersection_mm3'] for z in sample['native']['overlaps'] if z['part']==rootmeta['name']);assert abs(common.Volume()-want)<1e-5
    ev=HERE/'generated/visual_evidence';ev.mkdir(parents=True,exist_ok=False)
    def evidence(label,shape,meta,color=None):
        st=ev/(label+'.step');me=st.with_suffix('.stl');cq.exporters.export(shape,str(st));cq.exporters.export(shape,str(me),tolerance=.025,angularTolerance=.1);return dict(meta,name=label,step=st.relative_to(ROOT).as_posix(),mesh=me.relative_to(ROOT).as_posix(),sha256_step=r.sha(st),sha256_stl=r.sha(me),color=color or meta.get('color',[.5,.6,.7]),installed=False)
    moved=evidence('failed_upper_shoe_stage',bad,by[name]);hit=evidence('actual_upper_shoe_root_common',common,dict(body='thumb_mcp'),[.97,.02,.04]);p.write(ev/'scene.json',dict(parts=[moved,hit],evidence_only=True,actual_common_mm3=common.Volume()))
    kit=next(z for z in chosen['paths'] if z['id']=='upper_kit_lateral');kitindex=next((i for i,z in enumerate(kit['samples']) if abs(np.asarray(z['transform_local'])[:3,3]).sum()>1e-5),0);T=np.asarray(kit['samples'][kitindex]['transform_local']);W=g.original.WORLD@T@g.original.INV;shifted=[evidence('kit_entry_'+n,g.original.transform(c.load([by[n]])[0]['shape'],W),by[n]) for n in kit['moving']]
    F=poses.forward_kinematics(g.DESIGN,np.zeros(20))['bodies']
    def scene(parts,whole=False):
        rows=[]
        for z in hand+parts:
            is_hand=z in hand
            if is_hand and (z['name'] in ['thumb_segment2_dark_faceplate_clamp1_bolt','thumb_segment2_dark_faceplate_clamp2_bolt'] or (not whole and not z['body'].startswith('thumb'))):continue
            f=ROOT/z['mesh'];assert r.sha(f)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl'];rows.append(dict(name=z['name'],body=z['body'],mesh_path=f,points=r.read_stl_points(f),neutral_inverse=np.linalg.inv(F[z['body']]),color=[.65,.7,.75] if is_hand else z.get('color',[.5,.6,.7]),alpha=.2 if is_hand else 1. if z['name']==hit['name'] else .65))
        return dict(parts=rows,design=g.DESIGN)
    scenes=[scene([by[n] for n in path['installed_before']]+[moved,hit]),scene([by[n] for n in kit['installed_before']]+shifted),scene([by[n] for n in chosen.get('final_installed',chosen['paths'][-1]['installed_before'])],True)]
    view=r.view_fit(scenes[:2],[dict(angles_deg=[0.]*20)]*2,np.array([.3,-.55,1.]),1.);wide=r.view_fit([scenes[2]],[dict(angles_deg=[0.]*20)],np.array([.3,-.25,1.]),1.);imgs=[]
    try:
        for j,s in enumerate(scenes):
            cam=wide if j==2 else view;s['_render_state']=r.make_renderer(s,{'view':cam},1000,engine)
            for k,z in enumerate(s['parts']):s['_render_state']['model'].geom_rgba[k,3]=z['alpha']
            im,_=r.render_one(s,[0.]*20,'view',{'view':cam},1000,engine);imgs.append(im)
        pt=np.asarray(common.Center().toTuple())-view['focus'];factor=1000/(2*view['parallel_scale_mm']);x=500+np.dot(pt,view['right'])*factor;y=500-np.dot(pt,view['up'])*factor;ImageDraw.Draw(imgs[0]).ellipse((x-40,y-40,x+40,y+40),outline=(200,15,25),width=4)
        r.sheet([('이전: 덮개 뒤 안쪽 직선 접근이 막힘',imgs[0]),('교정: 끝구·받침 먼저, 옆으로 진입',imgs[1])],2,1000,'손/부품 형상 유지, 조립 순서와 접근 방향 교정','빨강: 실제 받침/엄지 뿌리 공통 재료 · 오른쪽: 새 덮개·볼트 아직 없음 · 임시 고정 지그/공구/강도 미검증').save(out/'assembly_before_after.png')
        r.sheet([('선택 순서의 마지막 검사 단계',imgs[2])],1,1000,'손쪽 종단 조립 후보, 실제 검사한 단계만 표시','짧은 피복 끝 참고는 긴 케이블 설치가 아님 · 전체 손 구동/줄 끼우기·장력/물리 제작 승인이 아님').save(out/'final_stage.png')
    finally:
        for s in scenes:
            if '_render_state' in s:s['_render_state']['renderer'].close()
    for rel in ['reviews/service.json','reviews/selection.json',selection['selected_report']]:source['cad/distal_terminal_assembly_v1/'+rel]=r.sha(HERE/rel)
    p.write(out/'report.json',dict(completed=True,source_sha256=source,outputs={z.name:r.sha(z) for z in out.iterdir() if z.is_file()},actual_common_mm3=common.Volume(),raw_failed_sample=index,simulation_steps=0,whole_hand_drive_approved=False))
if __name__=='__main__':main()
