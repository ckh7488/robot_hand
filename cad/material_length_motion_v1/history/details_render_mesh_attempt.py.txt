"""Actual wrist CAD and common material; separate mathematical bend diagnostic."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import check_motion as k
a,m,g,c,p,np,cq,ROOT,HERE=k.a,k.m,k.g,k.c,k.p,k.np,k.cq,k.ROOT,k.HERE
measure=m.drive.module('_length037_metrics',ROOT/'cad/long_housing_route_v1/review_core.py')
helper=m.drive.module('_length037_render',ROOT/'cad/thumb_root_guide_v1/guide_render.py');r,engine=helper.r,helper.engine
def source_piece(case,z):
    edge=a.full_edges(case,z['physical_line'],z['kind'])[z['segment']]
    return m.swept(edge,1.5).cut(m.swept(edge,.65)).clean() if z['kind']=='housing' else m.swept(edge,.3)
def main():
    raw=p.read(HERE/'reviews/check.json');analysis=p.read(HERE/'reviews/analysis.json');assert raw['completed'];source=dict(raw['source_sha256']);data,pack,guide,wrist=a.s.prior.inputs();hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,np.zeros(20));hand+=c.load([z for z in p.read(ROOT/'cad/thumb_entry_bend_v1/generated/parts/scene.json')['parts'] if z['installed_part_candidate']],data[0]);assert len(hand)==397
    failed=next(z for z in raw['cases'] if not z['clear']);case=analysis['cases'][failed['id']];meta=p.read(ROOT/failed['scene'])['parts'];lookup={z['name']:z for z in meta};overlaps=[z for v in failed['internal'] for z in v['overlaps']];commons=[];precision=[]
    for j,pair in enumerate(overlaps):
        left,right=lookup[pair['lead']],lookup[pair['part']];common=source_piece(case,left).intersect(source_piece(case,right),tol=1e-6);values=measure.metrics(common)['global_values'];commons.append(dict(name='actual_wrist_common_'+str(j),shape=common,body='palm',color=[1.,.03,.02],evidence_only=True,original_pair=pair));precision.append(dict(pair=pair,source_source_common_adaptive_values=values,source_source_basic_volume_mm3=common.Volume(),bbox_mm=c.m.pack._bbox(common)))
    scene=a.a.prior.export(commons,HERE/'generated/actual_common');common_meta=p.read(ROOT/scene)['parts'];a.save(HERE/'reviews/actual_common.json',dict(completed=True,case_id=case['id'],scene=scene,pairs=precision,only_three_source_source_common_pairs_computed=True,full_native_groups_repeated=0,adaptive_epsilon=1e-10,reporting_and_boolean_thresholds_unchanged=True))
    neutral=p.read(ROOT/'cad/contact_housing_link_v1/generated/neutral/scene.json')['parts']
    def assembly(which,local=False):
        parts=[]
        def add(z,T,alpha):
            path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl'];parts.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),color=z.get('color',[.56,.64,.7]),alpha=alpha,neutral_inverse=T))
        H=np.eye(4) if which is None else np.asarray(analysis['cases'][which['id']]['solution']['wrist_H']);P=np.eye(4) if which is None else np.asarray(analysis['cases'][which['id']]['solution']['wrist_P']);rows=neutral if which is None else p.read(ROOT/which['scene'])['parts']
        if not local:
            for z in hand:add(z,H,.17)
            for z in pack:add(z,H if z.get('body')=='palm' else np.eye(4),.21)
            for z in data[6]:add(z,np.eye(4),.32)
            for z in data[7]:add(z,H if z['name']=='Palm_wrist_saddle' else P if z['name'] in ['Wrist_box_carrier','AK_wrist_deviation'] else np.eye(4),.24)
            for z in wrist:add(z,H,.82)
        for z in rows:
            if local and not (z['kind']=='housing' and z['segment'] in [1,2,3]):continue
            add(z,np.eye(4),.78 if z['kind']=='housing' else .92)
        if which and not which['clear']:
            for z in common_meta[:1]:add(z,np.eye(4),1.)
        return dict(parts=parts,design=g.DESIGN)
    scenes=[assembly(None),*[assembly(z) for z in raw['cases']]];view=r.view_fit(scenes,[dict(angles_deg=[0.]*20)],np.array([.8,-.8,.5]),1.);imgs=[]
    def render(ss,view,size=850):
        try:
            ss['_render_state']=r.make_renderer(ss,{'v':view},size,engine)
            for j,z in enumerate(ss['parts']):ss['_render_state']['model'].geom_rgba[j,3]=z['alpha']
            return r.render_one(ss,[0.]*20,'v',{'v':view},size,engine)[0]
        finally:
            if '_render_state' in ss:ss['_render_state']['renderer'].close()
    for ss in scenes:imgs.append(render(ss,view))
    out=HERE/'images';out.mkdir(exist_ok=False);r.sheet([('기준: 저장 편손 기하 · 새 길이의 기준만 채택',imgs[0]),('굽힘−20°/편위−15° · 실제 재료검사0',imgs[1]),('굽힘+20°/편위+15° · 두 피복/줄 관통3기록',imgs[2])],3,850,'같은 재료길이의 손목 기하 후보 · 전체 운동은 미채택','실제 힘·마찰/예압·장력/설치 승인 아님 · 부모 잡기는 별도 반경 실패 · 손/모터/전선 변경0 · 빨강은 실제 공통재료').save(out/'wrist_before_after.png')
    close=assembly(failed,True);cv=r.view_fit([close],[dict(angles_deg=[0.]*20)],np.array([.7,-1.,.6]),1.);ci=render(close,cv,1000);r.sheet([('상류 두 피복의 실제 공통재료 · 큰 루프 중간',ci)],1,1000,'손목 +20°/+15°의 실제 관통','피복-피복 기본체적30.825403439mm³ · 원래 관통3쌍 보존 · 원호/반경 축소나 손 절삭 근거 아님').save(out/'actual_cross.png')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams['font.family']='Malgun Gothic';plt.rcParams['axes.unicode_minus']=False
    fig,axs=plt.subplots(1,2,figsize=(12,5));ts=np.linspace(0,1,501);C0=np.asarray(analysis['cases'][0]['solution']['controls'])[3];C1=np.asarray(analysis['cases'][1]['solution']['controls'])[3];xyz0=a.s.SPL(ts)@C0;xyz1=a.s.SPL(ts)@C1
    for ax,(u,v) in zip(axs,[(0,1),(0,2)]):
        ax.plot(xyz0[:,u],xyz0[:,v],label='편손 수학 중심선');ax.plot(xyz1[:,u],xyz1[:,v],color='crimson',label='부모25% 정해진 변형');ax.scatter([xyz1[-1,u]],[xyz1[-1,v]],c='crimson');ax.set_xlabel('XYZ'[u]+' mm');ax.set_ylabel('XYZ'[v]+' mm');ax.axis('equal');ax.grid(True,alpha=.2);ax.legend()
    fig.suptitle('피복 솔리드 생성 전 실패: 위 하류 끝의 Rmin 0.624399879mm < 피복 외반경1.5mm');fig.text(.5,.02,'수학 중심선 진단이며 실제 관통/손 CAD 결함 증명이 아님 · 이 부모후보의 피복 솔리드/native는 만들지 않음',ha='center');fig.tight_layout(rect=[0,.04,1,.93]);fig.savefig(out/'parent_curve_diagnostic.png',dpi=160);plt.close(fig)
    source['cad/material_length_motion_v1/details_render.py']=m.drive.sha(Path(__file__));a.save(out/'report.json',dict(completed=True,source_sha256=source,outputs={z.name:r.sha(z) for z in out.iterdir() if z.is_file()},actual_saved_CAD_wrist_panels=True,parent_panel_is_mathematical_centerline_only=True,only_three_new_common_pairs_computed=True,whole_native_groups_repeated=0,whole_hand_drive_approved=False))
if __name__=='__main__':main()
