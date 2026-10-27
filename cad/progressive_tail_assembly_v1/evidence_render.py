"""Save only new candidate overlap material and render actual saved CAD."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import check_progressive as k
n,s,t,a,m,g,c,p,np,ROOT,HERE=k.n,k.s,k.t,k.a,k.m,k.g,k.c,k.p,k.np,k.ROOT,k.HERE
measure=k.old.measure
renderer=m.drive.module('_form042_render',ROOT/'cad/whole_line_threading_v1/render.py')
r,engine=renderer.r,renderer.engine


def inputs():
    prior=p.read(ROOT/'cad/motor_guide_prethread_v1/reviews/check.json')
    return k.old.inputs(prior)


def fresh(case,record,hw):
    built=[n.build(case,i,record['formed_mm'][i],hw[i]) for i in [0,1]]
    return sum([z[0] for z in built],[])+[z[1] for z in built]


def main():
    out=HERE/'reviews/evidence.json';assert not out.exists();raw=p.read(HERE/'reviews/check.json');assert raw['completed']
    case,data,full,groups,hw=inputs();bad=next((z for z in raw['cases'] if not z['clear']),None);proof=[];commons=[]
    restore=p.read(HERE/'reviews/restoration.json') if (HERE/'reviews/restoration.json').exists() else None
    if restore:
        assert restore['completed']
        for path in restore['paths']:
            for sample in path['samples']:
                if not sample['clear']:bad=dict(id='restore_'+path['moving'],native=sample['native'],scene=sample['scene'])
        if restore['final'] and not restore['final']['clear']:bad=dict(id='final',**restore['final'])
    if bad:
        moving=fresh(case,raw['cases'][-1] if restore else bad,hw)
        if restore:moving+=c.load(p.read(ROOT/bad['scene'])['parts'])
        lookup={z['name']:z for _,zz in full for z in zz};lookup.update({z['name']:z for z in moving})
        pairs=[(key,pair) for key,test in bad['native'].items() for pair in test['overlaps']]
        for key,pair in sorted(pairs,key=lambda z:z[1]['material_intersection_mm3'],reverse=True)[:1]:
            sh=lookup[pair['lead']]['shape'].intersect(lookup[pair['part']]['shape'],tol=1e-6)
            proof.append(dict(group=key,pair=pair,adaptive_source_common_values=measure.metrics(sh)['global_values']))
            commons.append(dict(name='progressive_actual_common_'+str(len(commons)),shape=sh,body='palm',installed_part_candidate=False,color=[.95,.08,.07],pair=pair))
    scene=k.export(commons,'actual_common') if commons else None
    a.save(out,dict(completed=True,source_sha256=restore['source_sha256'] if restore else raw['source_sha256'],pairs=proof,evidence_scene=scene,native_group_replays=0,
                    failed_state=bad['id'] if bad else None,adaptive_epsilon=1e-10,absolute_comparison_tolerance=1e-5,
                    selection='One largest original native overlap, all raw pairs remain in check.json. No representative pair if clear.'))
    arm=p.read(ROOT/'cad/motor_guide_prethread_v1/images/report.json')['arm_visual_references']
    allbase=[z for z in renderer.current_metadata() if z['name'] not in [v['name'] for v in arm]]+arm
    base=[z for z in allbase if z['name'] not in raw['initial_missing_clamp_names']]
    first=p.read(ROOT/raw['cases'][0]['scene'])['parts'];last=p.read(ROOT/raw['cases'][-1]['scene'])['parts'];afterbase=base
    if restore and restore['conditional_CAD_sequence_clear']:
        afterbase=allbase;last=p.read(ROOT/restore['final']['scene'])['parts']
    elif restore and bad:last+=p.read(ROOT/bad['scene'])['parts']
    common=p.read(ROOT/scene)['parts'] if scene else [];source=dict(raw['source_sha256'])
    def make(meta,new):
        parts=[]
        for z in meta:
            path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl']
            parts.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),color=z.get('color',[.56,.64,.7]),
                              alpha=1 if z in common else .85 if z in new else .23,neutral_inverse=np.eye(4)))
        return dict(parts=parts,design=g.DESIGN)
    before=make(base+first,first);after=make(afterbase+last+common,last+common)
    view=r.view_fit([before,after],[dict(angles_deg=[0.]*20)],np.array([.8,-.9,.5]),1.)
    def render(ss,v):
        try:
            ss['_render_state']=r.make_renderer(ss,{'v':v},850,engine)
            for j,z in enumerate(ss['parts']):ss['_render_state']['model'].geom_rgba[j,3]=z['alpha']
            return r.render_one(ss,[0.]*20,'v',{'v':v},850,engine)[0]
        finally:
            if '_render_state' in ss:ss['_render_state']['renderer'].close()
    images=HERE/'images';images.mkdir(exist_ok=False)
    caption='새 순서 첫 간섭, 빨강은 실제 공통재료' if bad else '두 피복 CAD 안착, 원래 죔쇠·체결품 복구' if restore else '차례로 굽힌 두 피복과 실제 안내관'
    r.sheet([('선끼우기 뒤 시작: 두 안내관과 긴 꼬리는 수동 유지',render(before,view)),(caption,render(after,view))],2,850,
            '실제 뒤안내대 쪽부터 한 선씩 굽혀 가져오는 조립 후보',
            '전체 재료 길이 유지 / 팔받침 포함 / 실제 줄끼우기·체결력·힘·마찰·왕복운동·실물 승인 없음').save(images/'forming_before_after.png')
    if common:
        lookup={z['name']:z for z in base+last};chosen=max(common,key=lambda z:z['pair']['material_intersection_mm3']);pair=chosen['pair']
        local=make([lookup[pair['lead']],lookup[pair['part']],chosen],[lookup[pair['lead']],lookup[pair['part']],chosen])
        for z in local['parts']:
            if z['name']!=chosen['name']:z['alpha']=.22
        v=r.view_fit([local],[dict(angles_deg=[0.]*20)],np.array([.8,-.9,.5]),1.)
        r.sheet([('겹친 두 실제 CAD와 빨간 공통재료',render(local,v))],1,850,'새 조립 순서의 실제 간섭',
                pair['lead']+' / '+pair['part']+' / '+str(round(pair['material_intersection_mm3'],6))+' mm³').save(images/'actual_block.png')
    source['cad/progressive_tail_assembly_v1/evidence_render.py']=m.drive.sha(Path(__file__))
    a.save(images/'report.json',dict(completed=True,source_sha256=source,outputs={z.name:r.sha(z) for z in images.iterdir() if z.is_file()},
                                    native_recomputed=0,arm_visual_units='mm from SEAT041 exact-native-STEP display references',saved_actual_CAD=True))


if __name__=='__main__':main()
