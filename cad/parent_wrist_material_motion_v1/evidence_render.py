"""Preserve new contact evidence and render the actual continued assembly."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import run_motion as d
n,ROOT,HERE=d.n,d.ROOT,d.HERE
fr=n.m.drive.module('_motion047_frame_renderer',n.HERE/'evidence_render.py')
r,engine,old=fr.r,fr.engine,fr.old


def projection_plot(raw):
    if not raw['stop_reason'] or not raw['stop_reason'].startswith(('projection_','target_projection_')):return None
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams['font.family']='Malgun Gothic';plt.rcParams['axes.unicode_minus']=False
    rec=raw['increments'][-1];weights=[[n.np.array(z) for z in row] for row in raw['arc_length_weights']];budgets=n.np.array(raw['free_contour_budgets_mm'])
    curves=n.curves(n.np.array(rec['parameters']),rec['case'],weights,budgets)
    fig=plt.figure(figsize=(12,6));values=[]
    for i,row in enumerate(curves):
        side=max(['up','down'],key=lambda key:n.np.linalg.norm(row[key][-1]['end']-row['ports'][key+'_B']))
        ax=fig.add_subplot(1,2,i+1,projection='3d');points=[]
        for arc in row[side]:points.extend([arc['start'],arc['mid'],arc['end']])
        P=n.np.array(points);A=n.np.array(row[side][-1]['end']);B=n.np.array(row['ports'][side+'_B']);gap=float(n.np.linalg.norm(A-B))
        ax.plot(*P.T,color='#347f9f',label='미채택 수학중심선');ax.scatter(*A,c='#347f9f',s=35);ax.scatter(*B,c='#c84b37',s=50,label='실제 종단 목표')
        ax.plot(*n.np.array([A,B]).T,c='#c84b37');ax.set_title('선 '+str(i)+' / '+side+' / 끝점차 '+format(gap,'.6g')+' mm')
        ax.set_xlabel('X mm');ax.set_ylabel('Y mm');ax.set_zlabel('Z mm');ax.legend(fontsize=8);values.append(dict(line=i,span=side,endpoint_gap_mm=gap))
    fig.suptitle('증분 투영 실패 진단: '+rec['id']+' / 실제 재료 native 실패와 구별')
    fig.text(.04,.025,'원래 실패/한 차례 수치 이분 보존. 손·모터·길이·외경 변경 없음. 모든 자연스러운 경로 불가능의 증명 아님.',fontsize=10)
    fig.tight_layout(rect=[0,.06,1,.93]);path=HERE/'images/projection_diagnostic.png';fig.savefig(path,dpi=150);plt.close(fig)
    return dict(increment_id=rec['id'],endpoint_diagnostics=values,mathematical_centerlines_only=True,native_material_failure_proven=False)


def main():
    raw=n.p.read(HERE/'reviews/check.json');assert raw['completed'] and raw['states'];out=HERE/'reviews/evidence.json';assert not out.exists()
    ctx=d.k.context();hw_module=n.m.drive.module('_motion047_hardware_supplement',HERE/'moving_hardware.py');hw_module.main(ctx)
    hardware=n.p.read(HERE/'reviews/moving_hardware.json')
    witness=n.m.drive.module('_motion047_guide_witness',HERE/'guide_witnesses.py');witness.main()
    source=dict(raw['source_sha256']);source.update(hardware['source_sha256']);by_increment={z['id']:z for z in raw['increments']};last=raw['states'][-1]
    hardware_stop=next((z for z in hardware['states'] if not z['conditional_clear']),None)
    if hardware_stop:last=next(z for z in raw['states'] if z['id']==hardware_stop['id'])
    weights=[[n.np.array(z) for z in row] for row in raw['arc_length_weights']];budgets=n.np.array(raw['free_contour_budgets_mm']);commonrows=[];pairs=[]
    new_blocks=hardware_stop['blocking_records'] if hardware_stop else last['blocking_records']
    if new_blocks:
        chosen=max(new_blocks,key=lambda z:z['pair']['material_intersection_mm3']);pair=chosen['pair'];rec=by_increment[last['id']]
        lookup={v['name']:v for _,zz in d.k.pose(ctx,last['case']) for v in zz}
        if pair['lead'].startswith('continued_') or pair['part'].startswith('continued_'):
            fresh=d.k.build(n.np.array(rec['parameters']),rec['case'],weights,budgets);lookup.update({z['name']:z for z in fresh})
        sh=lookup[pair['lead']]['shape'].intersect(lookup[pair['part']]['shape'],tol=1e-6)
        pairs.append(dict(id=last['id'],pair=pair,group=chosen['group'],adaptive_common_values=n.s.e.measure.metrics(sh)['global_values'],common_bbox_mm=n.c.m.pack._bbox(sh)))
        commonrows.append(dict(name='motion_actual_common',shape=sh,body='palm',color=[.95,.08,.07],installed_part_candidate=False,pair=pair))
    common_scene=d.export(commonrows,'actual_common') if commonrows else None
    arm=n.p.read(ROOT/'cad/motor_guide_prethread_v1/images/report.json')['arm_visual_references'];byarm={z['name']:z for z in arm};base=[byarm.get(z['name'],z) for z in old.current_metadata()];assert len(base)==862
    images=HERE/'images';images.mkdir(exist_ok=False);mesh_cache={}
    def make(parts,case,accent):
        T=fr.transforms(base,case,ctx[-1]);result=[]
        for z in parts:
            path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl']
            if z['mesh'] not in mesh_cache:mesh_cache[z['mesh']]=r.read_stl_points(path)
            result.append(dict(name=z['name'],body='palm',mesh_path=path,points=mesh_cache[z['mesh']],neutral_inverse=T.get(z['name'],n.np.eye(4)),
                color=z.get('color',[.56,.64,.7]),alpha=.95 if z['name'] in accent else .24))
        return dict(parts=result,design=n.g.DESIGN)
    seed_parts=n.p.read(ROOT/raw['seed_scene'])['parts'];before=make(base+seed_parts,raw['seed_case'],{z['name'] for z in seed_parts})
    display_states=raw['states'][:raw['states'].index(last)+1] if hardware_stop else raw['states']
    peak=max(display_states,key=lambda z:z['parent_fraction']);chosen=[]
    for state in [peak,last]:
        if state['id'] not in [z['id'] for z in chosen]:chosen.append(state)
    scenes=[before];labels=['기존 저장 25%, 이번 재검사 표본에 합산하지 않음']
    for state in chosen:
        now=n.p.read(ROOT/state['scene'])['parts'];scenes.append(make(base+now,state['case'],{z['name'] for z in now}))
        labels.append(state['id']+' / '+('새 차단에서 중단' if state['blocking_records'] or (hardware_stop and state['id']==hardware_stop['id']) else '유한 기하 표본'))
    fit_scenes=list(scenes)
    if raw['all_planned_targets_reached'] and hardware['all_material_snapshot_hardware_checks_clear']:
        for state in raw['states']:
            if state['id'] in [z['id'] for z in chosen]:continue
            parts=n.p.read(ROOT/state['scene'])['parts'];fit_scenes.append(make(base+parts,state['case'],{z['name'] for z in parts}))
    view=r.view_fit(fit_scenes,[dict(angles_deg=[0.]*20)],n.np.array([.8,-.9,.5]),1.)
    def render(scene,camera):
        try:
            scene['_render_state']=r.make_renderer(scene,{'v':camera},850,engine)
            for j,z in enumerate(scene['parts']):scene['_render_state']['model'].geom_rgba[j,3]=z['alpha']
            return r.render_one(scene,[0.]*20,'v',{'v':camera},850,engine)[0]
        finally:
            if '_render_state' in scene:
                scene['_render_state']['renderer'].close();del scene['_render_state']
    tiles=[(label,render(scene,view)) for label,scene in zip(labels,scenes)]
    r.sheet(tiles,len(tiles),850,'저장 25%에서 이어간 같은 재료의 잡기·풀기/손목 후보',
        '같은 전체 길이 / 실제 FK·보어 재료 이동 / 미끄럼 없음은 조건부 / 힘·연속 운동·전체20모터 승인 없음').save(images/'motion_before_after.png')
    if commonrows:
        now=n.p.read(ROOT/last['scene'])['parts'];lookup={z['name']:z for z in base+now};pair=pairs[0]['pair'];cm=n.p.read(ROOT/common_scene)['parts']
        local=make([lookup[pair['lead']],lookup[pair['part']]]+cm,last['case'],{cm[0]['name']});v=r.view_fit([local],[dict(angles_deg=[0.]*20)],n.np.array([.8,-.9,.5]),1.)
        r.sheet([('실제 두 부품 전체와 빨간 공통재료',render(local,v))],1,850,'새 실제 간섭 위치',
            str(round(pair['material_intersection_mm3'],9))+' mm³ / 한 경로 후보의 실패, 손 절삭·모든 동작 불가능의 근거 아님').save(images/'actual_block.png')
    previews=[]
    if raw['all_planned_targets_reached'] and hardware['all_material_snapshot_hardware_checks_clear']:
        groups=[('approach',[None]+[z for z in raw['states'] if z['phase']=='approach']),
                ('release',[peak]+[z for z in raw['states'] if z['phase']=='release'])]
        release_end=next(z for z in reversed(raw['states']) if z['phase']=='release')
        groups.append(('wrist',[release_end]+[z for z in raw['states'] if z['phase']=='wrist']))
        for phase,states in groups:
            frames=[]
            for state in states:
                if state is None:scene=before;label='기존 25% 시작'
                else:
                    parts=n.p.read(ROOT/state['scene'])['parts'];scene=make(base+parts,state['case'],{z['name'] for z in parts});label=state['id']
                tile=render(scene,view)
                frames.append(r.sheet([(label,tile)],1,850,'대표 엄지 두 줄의 조건부 기하: '+phase,
                    '저장된 native 표본 재생 / 220ms는 설명속도 / 전체20모터·힘·장력·실물 구동 증거 아님'))
            filename='conditional_'+phase+'.gif';frames[0].save(images/filename,save_all=True,append_images=frames[1:],duration=220,loop=0)
            previews.append(dict(file=filename,frames=len(frames),frame_duration_ms=220,requested_whole_hand_gestures_completed=False))
    diagnostic=projection_plot(raw)
    source['cad/parent_wrist_material_motion_v1/evidence_render.py']=n.m.drive.sha(Path(__file__))
    source['cad/parent_wrist_material_motion_v1/reviews/check.json']=n.m.drive.sha(HERE/'reviews/check.json')
    source['cad/parent_wrist_material_motion_v1/reviews/guide_witnesses.json']=n.m.drive.sha(HERE/'reviews/guide_witnesses.json')
    source['cad/parent_wrist_material_motion_v1/reviews/moving_hardware.json']=n.m.drive.sha(HERE/'reviews/moving_hardware.json')
    source['cad/parent_wrist_material_motion_v1/moving_hardware.py']=n.m.drive.sha(HERE/'moving_hardware.py')
    n.a.save(out,dict(completed=True,source_sha256=source,new_common_pairs=pairs,scene=common_scene,projection_diagnostic=diagnostic,native_groups_recomputed=0,
        same_cord_seam_precision_pairs_recomputed=0,actual_hardware_supplement_report='cad/parent_wrist_material_motion_v1/reviews/moving_hardware.json',
        actual_hardware_supplement_is_additional_pairs=True,
        selection='Largest recorded new blocking pair at the first stopped native state. Numerical projection failure is separately illustrated and not classified as a native collision.'))
    n.a.save(images/'report.json',dict(completed=True,source_sha256=source,actual_saved_material_CAD=True,rendered_state_ids=['CONT046_seed_25']+[z['id'] for z in chosen],
        arm_visuals='SEAT041 exact native STEP-derived mm',hardware_pose_transform_function='CONT046 evidence_render.transforms',native_groups_recomputed=0,conditional_phase_previews=previews,
        outputs={z.name:r.sha(z) for z in images.iterdir() if z.is_file()}))


if __name__=='__main__':main()
