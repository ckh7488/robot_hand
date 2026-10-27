"""Actual saved-material CAD with explicit parent/wrist hardware transforms."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import continuation as n
k=n.m.drive.module('_continue046_render_native',n.HERE/'check.py')
r,engine,old=n.s.e.r,n.s.e.engine,n.s.e.old


def transforms(meta,case,rotor):
    H=n.np.array(case['H']);P=n.np.array(case['P']);F=n.g.g.v.frames(n.g.DESIGN,n.np.array(case['q']));F0=n.g.g.v.frames(n.g.DESIGN,n.np.zeros(20));M=n.s.frames(case['q'][19])[1]
    result={}
    for j,z in enumerate(meta):
        name=z['name'];T=n.np.eye(4)
        if j<397:
            body=z['body'];A,B=F[body];C,D=F0[body];T=H@n.s.transform(A,B)@n.np.linalg.inv(n.s.transform(C,D))
        elif name in rotor:T=M
        elif j<397+385 and z.get('body')=='palm':T=H
        elif name=='Palm_wrist_saddle':T=H
        elif name in ['Wrist_box_carrier','AK_wrist_deviation']:T=P
        elif j>=397+385+64+10:T=H
        result[name]=T
    return result


def main():
    raw=n.p.read(n.HERE/'reviews/check.json');analysis=n.p.read(n.HERE/'reviews/analysis.json');assert raw['completed'] and raw['states']
    out=n.HERE/'reviews/evidence.json';assert not out.exists();last=raw['states'][-1];rec=analysis['steps'][last['step']]
    source=dict(raw['source_sha256']);ctx=k.context();groups=k.pose(ctx,last['case']);weights=[[n.np.array(z) for z in row] for row in analysis['arc_length_weights']];budgets=n.np.array(analysis['free_contour_budgets_mm'])
    commonrows=[];pairs=[]
    if last['blocking_records']:
        chosen=max(last['blocking_records'],key=lambda z:z['pair']['material_intersection_mm3']);pair=chosen['pair']
        fresh=k.build(n.np.array(rec['parameters']),rec['case'],weights,budgets);lookup={z['name']:z for z in fresh+[v for _,zz in groups for v in zz]}
        common=lookup[pair['lead']]['shape'].intersect(lookup[pair['part']]['shape'],tol=1e-6)
        pairs.append(dict(step=last['step'],pair=pair,group=chosen['group'],adaptive_common_values=n.s.e.measure.metrics(common)['global_values'],common_bbox_mm=n.c.m.pack._bbox(common)))
        commonrows.append(dict(name='continued_actual_common',shape=common,body='palm',color=[.95,.08,.07],installed_part_candidate=False,pair=pair))
    scene=k.export(commonrows,'actual_common') if commonrows else None
    source['cad/coupled_material_continuation_v1/evidence_render.py']=n.m.drive.sha(Path(__file__))
    source['cad/coupled_material_continuation_v1/reviews/check.json']=n.m.drive.sha(n.HERE/'reviews/check.json')
    n.a.save(out,dict(completed=True,source_sha256=source,new_common_pairs=pairs,scene=scene,native_groups_recomputed=0,same_cord_seam_precision_pairs_recomputed=0,
        selection='Largest recorded new blocking pair at first stopped native state; old adjacent motor seams are preserved and not chased.'))
    arm=n.p.read(n.ROOT/'cad/motor_guide_prethread_v1/images/report.json')['arm_visual_references']
    # Keep ordering: hand397/pack385/elect64/arm10/PAIR6.
    original=old.current_metadata();byarm={z['name']:z for z in arm};base=[byarm.get(z['name'],z) for z in original];assert len(base)==862
    before_case=n.case_at(n.np.zeros(20));before_T=transforms(base,before_case,ctx[-1]);after_T=transforms(base,last['case'],ctx[-1])
    form=n.p.read(n.ROOT/'cad/progressive_tail_assembly_v1/reviews/restoration.json');houses=n.p.read(n.ROOT/form['final']['scene'])['parts']
    alt=n.p.read(n.ROOT/'cad/motor_full_cord_thread_v1/reviews/anchor_check.json');cords=[]
    for i in [0,1]:
        z=next(z for z in reversed(alt['cases']) if z['line']==i);cords+=n.p.read(n.ROOT/z['scene'])['parts']
    now=n.p.read(n.ROOT/last['scene'])['parts']
    def make(meta,T,accent):
        parts=[]
        for z in meta:
            path=n.ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl']
            parts.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),neutral_inverse=T.get(z['name'],n.np.eye(4)),
                color=z.get('color',[.56,.64,.7]),alpha=.95 if z['name'] in accent else .24))
        return dict(parts=parts,design=n.g.DESIGN)
    before=make(base+houses+cords,before_T,{z['name'] for z in houses+cords});after=make(base+now,after_T,{z['name'] for z in now})
    view=r.view_fit([before,after],[dict(angles_deg=[0.]*20)],n.np.array([.8,-.9,.5]),1.)
    def render(ss,v):
        try:
            ss['_render_state']=r.make_renderer(ss,{'v':v},850,engine)
            for j,z in enumerate(ss['parts']):ss['_render_state']['model'].geom_rgba[j,3]=z['alpha']
            return r.render_one(ss,[0.]*20,'v',{'v':v},850,engine)[0]
        finally:
            if '_render_state' in ss:ss['_render_state']['renderer'].close()
    images=n.HERE/'images';images.mkdir(exist_ok=False)
    status='새 실제 간섭에서 중단' if last['blocking_records'] else '유한 표본, 힘·연속 운동 승인 없음'
    r.sheet([('중립: 기존 전체 피복·줄',render(before,view)),('부모 접근 '+str(round(last['parent_fraction']*100))+'% / '+status,render(after,view))],2,850,
        '같은 재료 길이로 이어간 부모 접근 후보',
        '실제 모터·손 FK와 피복 변형 / 새 지지 없음 / 미끄럼 없음은 조건부 / 전체 부모·손목·실물은 미완').save(images/'continuation_before_after.png')
    if commonrows:
        commonmeta=n.p.read(n.ROOT/scene)['parts'];lookup={z['name']:z for z in base+now};pair=pairs[0]['pair'];selected=[lookup[pair['lead']],lookup[pair['part']]]+commonmeta
        local=make(selected,after_T,{commonmeta[0]['name']});v=r.view_fit([local],[dict(angles_deg=[0.]*20)],n.np.array([.8,-.9,.5]),1.)
        r.sheet([('실제 두 부품 전체와 빨간 공통재료',render(local,v))],1,850,'부모 접근 후보의 실제 막힘',
            str(round(pair['material_intersection_mm3'],9))+' mm³ / 한 변형 후보의 실패, 손 절삭·모든 운동 불가능의 근거 아님').save(images/'actual_block.png')
    n.a.save(images/'report.json',dict(completed=True,source_sha256=source,native_groups_recomputed=0,actual_saved_material_CAD=True,
        hardware_mesh_rigid_transforms=after_T,hardware_transform_count=len(after_T),arm_visuals='SEAT041 exact native STEP-derived mm',
        native_hardware_pose_function='check.pose: accepted hand FK+H, palm39/PAIR H, arm P/H, forearm/electrical fixed, rotor9 actual motor phase',
        outputs={z.name:r.sha(z) for z in images.iterdir() if z.is_file()}))


if __name__=='__main__':main()
