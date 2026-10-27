"""Saved real housing/loose-guide assembly scenes, with failure material if any."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import evidence as e
s,t,a,m,g,c,p,np,ROOT,HERE=e.s,e.t,e.a,e.m,e.g,e.c,e.p,e.np,e.ROOT,e.HERE
old=m.drive.module('_seat041_render_meta',ROOT/'cad/whole_line_threading_v1/render.py');r,engine=old.r,old.engine


def main():
    raw=p.read(HERE/'reviews/check.json');proof=p.read(HERE/'reviews/overlap_evidence.json');assert raw['completed'];source=dict(raw['source_sha256']);arm=[];folder=HERE/'generated/render_arm_mm';folder.mkdir(exist_ok=False)
    for z in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference']:
        st=ROOT/'cad/modular_roots_v9/parts'/(z['asset']+'.step');source[st.relative_to(ROOT).as_posix()]=r.sha(st);sh=s.cq.importers.importStep(str(st)).val();mesh=folder/(z['name']+'.stl');s.cq.exporters.export(sh,str(mesh),tolerance=.15,angularTolerance=.25);arm.append(dict(name=z['name'],mesh=mesh.relative_to(ROOT).as_posix(),sha256_stl=r.sha(mesh),mesh_units='mm',visual_reference_only=True,color=[.6,.65,.7]))
    allbase=[z for z in old.current_metadata() if z['name'] not in [v['name'] for v in arm]]+arm;base=[z for z in allbase if z['name'] not in raw['initial_missing_clamp_names']];first=raw['cases'][0];before_parts=p.read(ROOT/first['scene'])['parts'];bad,installed=e.failure(raw)
    if raw['conditional_sequence_clear']:after_base=allbase;after_parts=p.read(ROOT/raw['final']['scene'])['parts'];caption='두 피복 CAD 안착·죔쇠 복구 / 줄은 아직 없음'
    elif bad and bad['id']=='restoration':
        after_base=base+[z for z in allbase if z['name'] in installed and '_housing_clamp_' in z['name']];after_parts=p.read(ROOT/raw['cases'][-1]['scene'])['parts']+p.read(ROOT/bad['scene'])['parts'];caption='복구 경로 실패 / 빨강은 실제 겹친 재료'
    else:after_base=base;after_parts=p.read(ROOT/(bad or raw['cases'][-1])['scene'])['parts'];caption='안내관 운반 경로 실패 / 빨강은 실제 겹친 재료'
    common=p.read(ROOT/proof['evidence_scene'])['parts'] if proof['evidence_scene'] else []
    def scene(meta,new):
        parts=[]
        for z in meta:
            path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl'];parts.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),color=z.get('color',[.56,.64,.7]),alpha=1 if z in common else .85 if z in new else .23,neutral_inverse=np.eye(4)))
        return dict(parts=parts,design=g.DESIGN)
    before=scene(base+before_parts,before_parts);after=scene(after_base+after_parts+common,after_parts+common);view=r.view_fit([before,after],[dict(angles_deg=[0.]*20)],np.array([.8,-.9,.5]),1.)
    def render(ss,view):
        try:
            ss['_render_state']=r.make_renderer(ss,{'v':view},850,engine)
            for j,z in enumerate(ss['parts']):ss['_render_state']['model'].geom_rgba[j,3]=z['alpha']
            return r.render_one(ss,[0.]*20,'v',{'v':view},850,engine)[0]
        finally:
            if '_render_state' in ss:ss['_render_state']['renderer'].close()
    out=HERE/'images';out.mkdir(exist_ok=True);r.sheet([('시작: 죔쇠 분리, 긴 피복 뒤끝에 실제 안내관 진입',render(before,view)),(caption,render(after,view))],2,850,'모터 안내관을 먼저 끼우고 가져오는 조건부 조립 · 실제 CAD','손·모터·랙·전기 설치 위치 유지 / 같은 피복 길이 / 수동 유지 기하 후보 / 실제 힘·마찰·줄끼우기·왕복운동·외형 승인은 없음').save(out/'seat_before_after.png')
    if common:
        by={z['name']:z for z in after_base+after_parts};chosen=max(common,key=lambda z:z['pair']['material_intersection_mm3']);pair=chosen['pair'];local=scene([by[pair['lead']],by[pair['part']],chosen],[by[pair['lead']],by[pair['part']],chosen])
        for z in local['parts']:
            if z['name']!=chosen['name']:z['alpha']=.22
        v=r.view_fit([local],[dict(angles_deg=[0.]*20)],np.array([.8,-.9,.5]),1.);r.sheet([('실제 겹친 두 CAD와 공통재료',render(local,v))],1,850,'조건부 조립 경로에서 발견한 실제 간섭',pair['lead']+' / '+pair['part']+' / 기본체적 '+str(round(pair['material_intersection_mm3'],6))+' mm³').save(out/'actual_block.png')
    source['cad/motor_guide_prethread_v1/render.py']=m.drive.sha(Path(__file__));a.save(out/'report.json',dict(completed=True,source_sha256=source,outputs={z.name:r.sha(z) for z in out.iterdir() if z.is_file()},native_recomputed=0,saved_actual_CAD=True,arm_visual_references=arm,arm_mm_meshes_from_exact_native_STEP=True,inherited_m_unit_arm_meshes_not_used_as_mm=True,first_render_and_original_code_preserved='history/images_attempt1 and history/render_attempt1.py.txt',all_old_long_refs_and_cords_absent=True))
if __name__=='__main__':main()
