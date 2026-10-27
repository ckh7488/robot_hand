"""Preserve both feed candidates, original obstruction, and same-cord seams."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import check_anchor as k
n,first,t,w,l,a,m,g,c,p,np,cq,ROOT,HERE=k.n,k.first,k.t,k.w,k.l,k.a,k.m,k.g,k.c,k.p,k.np,k.cq,k.ROOT,k.HERE
measure=t.prior.measure
old=m.drive.module('_cord043_render',ROOT/'cad/whole_line_threading_v1/render.py');r,engine=old.r,old.engine


def inputs():
    case=t.neutral();data,groups=t.prior.inputs(case);form=p.read(ROOT/'cad/progressive_tail_assembly_v1/reviews/restoration.json')
    housing=c.load(p.read(ROOT/form['final']['scene'])['parts'],data[0])
    return case,data,groups,housing


def moving(case,record,alternative):
    return n.build(case,record['line'],record['tip_coordinate_mm'])[0] if alternative else first.n.build(case,record['line'],record['tip_coordinate_mm'],record['tail_fraction'])[0]


def retained(case,record):
    if not record['retained_cord_names']:return []
    rows=n.build(case,0,n.samples(case,0)[-1])[0]
    assert [z['name'] for z in rows]==record['retained_cord_names']
    return rows


def main():
    out=HERE/'reviews/evidence.json';assert not out.exists();raw=p.read(HERE/'reviews/check.json');alt=p.read(HERE/'reviews/anchor_check.json')
    assert raw['completed'] and alt['completed'];case,data,groups,housing=inputs();full=[z for _,zz in groups for z in zz]+housing
    choices=[];bad=raw['cases'][-1]
    pairs=[(key,z) for key,zz in bad['native'].items() for z in zz['overlaps']]
    if pairs:
        key,pair=max(pairs,key=lambda z:z[1]['material_intersection_mm3']);choices.append(('original',bad,key,pair))
    for i in [0,1]:
        records=[z for z in alt['cases'] if z['line']==i]
        if not records:continue
        rec=records[-1]
        actual=rec['blocking_overlap_records'] or rec['known_adjacent_motor_joint_records']
        if actual:
            z=max(actual,key=lambda v:v['pair']['material_intersection_mm3']);choices.append(('anchor_first',rec,z['group'],z['pair']))
    results=[];common=[]
    for mode,rec,key,pair in choices:
        lookup={z['name']:z for z in full+retained(case,rec)+moving(case,rec,mode=='anchor_first')}
        sh=lookup[pair['lead']]['shape'].intersect(lookup[pair['part']]['shape'],tol=1e-6)
        result=dict(mode=mode,state=rec['id'],group=key,pair=pair,adaptive_source_common_values=measure.metrics(sh)['global_values'],common_bbox_mm=c.m.pack._bbox(sh))
        if mode=='anchor_first' and key=='self':
            left,right=lookup[pair['lead']],lookup[pair['part']];join=np.asarray(left['endpoint_end_mm']);bbox=result['common_bbox_mm']
            corners=np.array([[x,y,z] for x in [bbox[0][0],bbox[1][0]] for y in [bbox[0][1],bbox[1][1]] for z in [bbox[0][2],bbox[1][2]]])
            result.update(same_physical_line=left['physical_line']==right['physical_line'],consecutive_path_indices=sorted([left['path_segment'],right['path_segment']])==[0,1],
                          maximum_common_bbox_corner_distance_from_join_mm=float(np.max(np.linalg.norm(corners-join,axis=1))),metadata_classification_not_overlap_exclusion=True)
        results.append(result);common.append(dict(name='cord_actual_common_'+str(len(common)),shape=sh,body='palm',installed_part_candidate=False,color=[.95,.08,.07],pair=pair,mode=mode))
    scene=first.export(common,'actual_common') if common else None
    source=dict(alt['source_sha256']);source['cad/motor_full_cord_thread_v1/evidence_render.py']=m.drive.sha(Path(__file__));source['cad/motor_full_cord_thread_v1/reviews/anchor_check.json']=m.drive.sha(HERE/'reviews/anchor_check.json')
    joins=[]
    for i in [0,1]:
        path,labels,length=n.specification(case,i)
        angle=float(np.degrees(np.arccos(np.clip(np.dot(np.array(path[0].tangentAt(1).toTuple()),np.array(path[1].tangentAt(0).toTuple())),-1,1))))
        joins.append(dict(line=i,anchor_bore_model_length_mm=path[0].Length(),motor_free_tangent_mm=path[2].Length(),endpoint_gap_mm=(path[0].endPoint()-path[1].startPoint()).Length,tangent_kink_deg=angle))
    a.save(out,dict(completed=True,source_sha256=source,pairs=results,evidence_scene=scene,native_group_replays=0,original_raw_reports_unchanged=True,motor_join_geometry=joins,
                    selection='Preplanned largest original obstruction and last executed anchor-first state per line: blocking pair if present, otherwise recorded adjacent motor model seam. No unreported pair searched.',adaptive_epsilon=1e-10,absolute_comparison_tolerance=1e-5))
    arm=p.read(ROOT/'cad/motor_guide_prethread_v1/images/report.json')['arm_visual_references'];base=[z for z in old.current_metadata() if z['name'] not in [v['name'] for v in arm]]+arm
    houses=p.read(ROOT/alt['housing_scene'])['parts'];before=p.read(ROOT/raw['cases'][-1]['scene'])['parts'];last=alt['cases'][-1]
    after=p.read(ROOT/last['scene'])['parts'];earlier=[]
    if last['retained_cord_names']:
        final0=next(z for z in reversed(alt['cases']) if z['line']==0);earlier=p.read(ROOT/final0['scene'])['parts']
    commons=p.read(ROOT/scene)['parts'] if scene else []
    def make(meta,new):
        parts=[]
        for z in meta:
            path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl']
            parts.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),color=z.get('color',[.56,.64,.7]),alpha=.9 if z in new else .22,neutral_inverse=np.eye(4)))
        return dict(parts=parts,design=g.DESIGN)
    bscene=make(base+houses+before,before);ascene=make(base+houses+earlier+after,earlier+after)
    view=r.view_fit([bscene,ascene],[dict(angles_deg=[0.]*20)],np.array([.8,-.9,.5]),1.)
    def render(ss,v):
        try:
            ss['_render_state']=r.make_renderer(ss,{'v':v},850,engine)
            for j,z in enumerate(ss['parts']):ss['_render_state']['model'].geom_rgba[j,3]=z['alpha']
            return r.render_one(ss,[0.]*20,'v',{'v':v},850,engine)[0]
        finally:
            if '_render_state' in ss:ss['_render_state']['renderer'].close()
    images=HERE/'images';images.mkdir(exist_ok=False)
    caption='고정부 보어 먼저, 두 전체줄의 선언 끝까지 배치' if alt['bounded_sequence_clear_of_other_overlaps'] else '고정부 보어 먼저, 새 실제 간섭에서 중단'
    r.sheet([('첫 후보: 감김홈 시작 긴 줄, 실제 스풀 간섭',render(bscene,view)),(caption,render(ascene,view))],2,850,
            '실제 모터 고정부·감김홈·R5 통로부터 전체 줄을 넣는 조건부 순서',
            '현재 하드웨어·두 전체피복 유지 / raw 접합 기록 보존 / 수동 유지 기하, 실제 고정력·장력·왕복·외형 승인 없음').save(images/'cord_before_after.png')
    if commons:
        original=next(z for z in commons if z['mode']=='original');by={z['name']:z for z in base};local=make([by[original['pair']['part']],original],[original])
        local['parts'][-1]['alpha']=1.;v=r.view_fit([local],[dict(angles_deg=[0.]*20)],np.array([.8,-.9,.5]),1.)
        r.sheet([('실제 스풀과 첫 후보 공통재료, 긴 줄은 전체그림 참조',render(local,v))],1,850,
                '첫 진입 후보의 실제 겹침 위치',str(round(original['pair']['material_intersection_mm3'],6))+' mm³ / 증거는 설치 부품이 아님').save(images/'actual_motor_block.png')
    a.save(images/'report.json',dict(completed=True,source_sha256=source,outputs={z.name:r.sha(z) for z in images.iterdir() if z.is_file()},native_recomputed=0,
                                    saved_actual_CAD=True,arm_visual_units='SEAT041 native STEP-derived mm references'))


if __name__=='__main__':main()
