"""Actual saved meshes and exact stage omissions; no simulation steps."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from probe import *
import types
helper=g.g.drive.module('_guide021_render_helpers',ROOT/'cad/thumb_root_guide_v1/guide_render.py')
r,poses,engine=helper.r,helper.poses,helper.engine
from PIL import ImageDraw
def main():
    out=HERE/'images';out.mkdir(exist_ok=False)
    static=p.read(HERE/'reviews/static.json');service=p.read(HERE/'reviews/selected_service.json');assert static['completed'] and service['completed']
    hand=p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')['hand'];meta=p.read(ROOT/static['scene'])['parts'];by={z['name']:z for z in meta};source={}
    oldmeta=p.read(ROOT/'cad/thumb_root_guide_v1/selected2/generated/parts/scene.json')['parts'];oldby={z['name']:z for z in oldmeta}
    evidence=p.read(ROOT/'cad/thumb_root_guide_v1/generated/visual_evidence_scope2/scene.json')['parts'];hit=next(z for z in evidence if z['name']=='assembly_actual_liner_tip_intersection');shifted=next(z for z in evidence if z['name']=='flanged_liner_1')
    ev=HERE/'generated/visual_evidence';ev.mkdir(parents=True,exist_ok=False)
    path=next(z for z in service['paths'] if z['id']=='flanged_liner_1_radial');sample=path['samples'][-2];T=np.asarray(sample['transform_local'])
    s=c.load([by['flanged_liner_1']])[0]['shape'];s=g.original.transform(s,g.original.WORLD@T@g.original.INV)
    st=ev/'upper_liner_radial_stage.step';me=st.with_suffix('.stl');cq.exporters.export(s,str(st));cq.exporters.export(s,str(me),tolerance=.025,angularTolerance=.1)
    moved=dict(by['flanged_liner_1'],step=st.relative_to(ROOT).as_posix(),mesh=me.relative_to(ROOT).as_posix(),sha256_step=r.sha(st),sha256_stl=r.sha(me))
    p.write(ev/'scene.json',dict(parts=[moved],evidence_only=True))
    F=poses.forward_kinematics(g.DESIGN,np.zeros(20))['bodies']
    def scene(new,absent,extra=None,whole=False):
        rows=[]
        def add(z,body,alpha,color=None):
            file=ROOT/z['mesh'];assert r.sha(file)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl']
            rows.append(dict(name=z['name'],body=body,mesh_path=file,points=r.read_stl_points(file),color=color or z.get('color',[.65,.7,.75]),alpha=alpha,neutral_inverse=np.linalg.inv(F[body]) if body!='palm' else np.eye(4)))
        for z in hand:
            if z['name'] in absent+static['design']['replaced_hand_parts'] or (not whole and not z['body'].startswith('thumb')):continue
            add(z,z['body'],.25,[.65,.7,.75])
        for z in new:add(z,z['body'],.5 if z['name']=='paired_liner_carrier' else 1.)
        if extra:add(extra,'palm',1.)
        return dict(parts=rows,design=g.DESIGN)
    q0=np.zeros(20);qm=q0.copy();qm[19]=-15.
    firstnames=['paired_liner_carrier','guide_cover_M2x12_1','guide_cover_M2x12_2','flanged_liner_1']
    scenes=[scene([shifted if n=='flanged_liner_1' else oldby[n] for n in firstnames],[],extra=hit),
        scene([by[n] for n in path['installed_new']]+[moved],path['missing_hand_parts']),scene(meta,[]),scene(meta,[],whole=True)]
    qs=[qm,q0,q0,q0]
    camera=r.view_fit(scenes[:3],[dict(angles_deg=z.tolist()) for z in qs[:3]],np.array([.45,-.55,1.]),1.)
    wide=r.view_fit([scenes[3]],[dict(angles_deg=q0.tolist())],np.array([.3,-.25,1.]),1.)
    imgs=[]
    try:
        for i,(a,q) in enumerate(zip(scenes,qs)):
            cam=wide if i==3 else camera;a['_render_state']=r.make_renderer(a,{'view':cam},900,engine)
            for j,z in enumerate(a['parts']):a['_render_state']['model'].geom_rgba[j,3]=z['alpha']
            image,_=r.render_one(a,q,'view',{'view':cam},900,engine);imgs.append(image)
        r.sheet([('이전: 끼우는 중 끝마디에 걸림',imgs[0]),('교정: 끝마디 전에 위에서 끼움',imgs[1]),('끝마디·핀을 다시 놓은 후보',imgs[2])],3,900,
            '손 본체 유지 · 새 안내관 고정부를 분리형으로 교정',
            '빨강: 이전 실제 공통 재료 · 가운데: 아직 없는 끝마디/핀을 숨김 없이 제외한 조립 단계 · 실제 경로 결과는 보고서 범위').save(out/'assembly_sequence.png')
        r.sheet([('분리 고정부와 방향 고정 턱',imgs[2]),('손 전체에서 본 후보 돌출',imgs[3])],2,900,
            '안내관의 두 줄 경로는 유지 · 새 고정부만 교정',
            '맞춤 관/턱·공급품·강도·공구/지그 미검증 · 모터~가이드 입구/전체 장력 미연결 · 모터 구동 영상 아님').save(out/'guide_candidate.png')
    finally:
        for a in scenes:
            if '_render_state' in a:a['_render_state']['renderer'].close()
    assert all(r.sha(ROOT/z)==h for z,h in source.items())
    p.write(out/'report.json',dict(completed=True,source_sha256=source,outputs={z.name:r.sha(z) for z in out.iterdir() if z.is_file()},simulation_steps=0,asset_compile_threads=False,actual_saved_meshes=True,scope='Old true interference and current explicit assembly/complete nominal candidate;not force driven motion or human-form acceptance.'))
if __name__=='__main__':main()
