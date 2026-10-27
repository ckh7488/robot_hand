"""Preserve any new actual obstruction and show simultaneously posed CAD."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import slide_model as s
r,engine,old=s.e.r,s.e.engine,s.e.old


def main():
    raw=s.p.read(s.HERE/'reviews/verified_check.json');assert raw['completed'];out=s.HERE/'reviews/evidence.json';assert not out.exists()
    source=dict(raw['source_sha256']);evidence=[];results=[]
    if raw['states'][-1]['blocking_records']:
        _,groups,housing,rotor,child=s.context()
        rec=raw['states'][-1];chosen=max(rec['blocking_records'],key=lambda z:z['pair']['material_intersection_mm3'])
        posed,_=s.hardware(groups,rec['IP_deg'],rotor,child);cord=s.cords(rec['IP_deg']);lookup={z['name']:z for z in cord+housing+[v for _,zz in posed for v in zz]};pair=chosen['pair']
        common=lookup[pair['lead']]['shape'].intersect(lookup[pair['part']]['shape'],tol=1e-6)
        results.append(dict(IP_deg=rec['IP_deg'],selection='Largest actually recorded new blocking pair at stopped state; old adjacent same-cord seams are not recomputed as evidence.',
                            pair=pair,adaptive_common_values=s.e.measure.metrics(common)['global_values']))
        evidence.append(dict(name='sliding_actual_common',shape=common,body='palm',installed_part_candidate=False,color=[.95,.08,.07],pair=pair))
    scene=s.export(evidence,'actual_common') if evidence else None
    source['cad/whole_cord_sliding_v1/evidence_render.py']=s.m.drive.sha(Path(__file__));source['cad/whole_cord_sliding_v1/reviews/verified_check.json']=s.m.drive.sha(s.HERE/'reviews/verified_check.json')
    s.a.save(out,dict(completed=True,source_sha256=source,new_common_pairs=results,scene=scene,native_groups_recomputed=0,same_cord_seam_precision_pairs_recomputed=0))
    arm=s.p.read(s.ROOT/'cad/motor_guide_prethread_v1/images/report.json')['arm_visual_references'];base=[z for z in old.current_metadata() if z['name'] not in {v['name'] for v in arm}]+arm
    form=s.p.read(s.ROOT/'cad/progressive_tail_assembly_v1/reviews/restoration.json');houses=s.p.read(s.ROOT/form['final']['scene'])['parts']
    alt=s.p.read(s.ROOT/'cad/motor_full_cord_thread_v1/reviews/anchor_check.json');before=[]
    for i in [0,1]:
        rec=next(z for z in reversed(alt['cases']) if z['line']==i);before+=s.p.read(s.ROOT/rec['scene'])['parts']
    def make(metadata,accent):
        parts=[]
        for z in metadata:
            path=s.ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl']
            parts.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),color=z.get('color',[.56,.64,.7]),
                alpha=.95 if z['name'] in accent else .23,neutral_inverse=s.np.eye(4)))
        return dict(parts=parts,design=s.g.DESIGN)
    allscenes=[make(base+houses+before,{z['name'] for z in before})];labels=['편손 중립, 기존 전체줄 경계']
    for rec in raw['states']:
        now=s.p.read(s.ROOT/rec['scene'])['parts'];names={z['name'] for z in now}
        allscenes.append(make([z for z in base if z['name'] not in names]+houses+now,names));labels.append('IP '+str(rec['IP_deg'])+'° / 모터 '+str(rec['motor_output_deg'])+'°')
    view=r.view_fit(allscenes,[dict(angles_deg=[0.]*20)],s.np.array([.8,-.9,.5]),1.)
    def render(ss,v):
        try:
            ss['_render_state']=r.make_renderer(ss,{'v':v},850,engine)
            for j,z in enumerate(ss['parts']):ss['_render_state']['model'].geom_rgba[j,3]=z['alpha']
            return r.render_one(ss,[0.]*20,'v',{'v':v},850,engine)[0]
        finally:
            if '_render_state' in ss:ss['_render_state']['renderer'].close()
    images=s.HERE/'images';images.mkdir(exist_ok=False);frames=[render(z,view) for z in allscenes]
    caption='조건부 유한 IP 경로, 부모·손목은 중립' if raw['conditional_IP_path_clear'] else '새 실제 간섭에서 중단한 후보'
    r.sheet([(labels[0],frames[0]),(labels[-1]+' / '+caption,frames[-1])],2,850,
        '같은 두 전체줄 · 실제 모터와 엄지 IP의 동시 회전',
        '줄·피복 길이 유지 / 실제 축슬라이딩 기하 / 고정·예압·힘·실물·부모손목 왕복 승인 없음').save(images/'sliding_before_after.png')
    if raw['conditional_IP_path_clear']:
        sequence=[frames[j] for j in [0,1,2,3,2,1,0]]
        sequence[0].save(images/'conditional_IP_sliding.gif',save_all=True,append_images=sequence[1:],duration=220,loop=0)
    if evidence:
        pair=results[0]['pair'];last=s.p.read(s.ROOT/raw['states'][-1]['scene'])['parts'];lookup={z['name']:z for z in base+houses+last}
        common=s.p.read(s.ROOT/scene)['parts'][0];local=make([lookup[pair['lead']],lookup[pair['part']],common],{common['name']});v=r.view_fit([local],[dict(angles_deg=[0.]*20)],s.np.array([.8,-.9,.5]),1.)
        r.sheet([('실제 두 부품과 공통재료, 빨강은 증거',render(local,v))],1,850,'동시 회전 후보의 실제 간섭',str(pair['material_intersection_mm3'])+' mm³ / 손 절삭·모든 경로 불가능의 근거가 아님').save(images/'actual_block.png')
    s.a.save(images/'report.json',dict(completed=True,source_sha256=source,native_recomputed=0,actual_saved_CAD=True,
        arm_visuals='SEAT041 exact native STEP-derived mm meshes',illustrative_frame_duration_ms=220,
        outputs={z.name:r.sha(z) for z in images.iterdir() if z.is_file()}))


if __name__=='__main__':main()
