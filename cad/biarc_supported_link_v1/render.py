"""Saved current CAD and derived-length diagnostics, no native group replay."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import link_model as l
a,m,g,c,p,np,ROOT,HERE=l.a,l.m,l.g,l.c,l.p,l.np,l.ROOT,l.HERE
helper=m.drive.module('_biarc039_render',ROOT/'cad/thumb_root_guide_v1/guide_render.py');r,engine=helper.r,helper.engine

def main():
    raw=p.read(HERE/'reviews/check.json');an=p.read(HERE/'reviews/analysis.json');prec=p.read(HERE/'reviews/overlap_precision.json');source=dict(raw['source_sha256']);ret=p.read(ROOT/'cad/sheath_retention_v1/selected/generated/parts/scene.json')['parts'];ps=p.read(ROOT/'cad/motor_tendon_terminal_v1/selected/generated/assembly_scene.json');pack=ps['new_parts']+ps['retained_parts']+[z for z in ret if z['body']=='forearm'];support=p.read(ROOT/'cad/support_attachment_basis_v1/generated/checked/scene.json')['parts'];newhw=[z for z in support if z['installed_part_candidate']];removed=[z['replaced_part'] for z in newhw];pack=[z for z in pack if z['name'] not in removed]+newhw;assert len(pack)==385
    hand=p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')['hand'];hand=[z for z in hand if z['name'] not in ['thumb_segment2_dark_faceplate_clamp1_bolt','thumb_segment2_dark_faceplate_clamp2_bolt']];offset=p.read(ROOT/'cad/thumb_sheath_terminal_v2/selected/generated/candidate_scene.json');hand+=offset['retained_guide_parts']+[z for z in offset['parts'] if z['name'] not in ['distal_flanged_ferrule_0','distal_flanged_ferrule_1']]+[z for z in ret if z['body']=='thumb_mcp'];entry=p.read(ROOT/'cad/thumb_entry_bend_v1/generated/parts/scene.json')['parts'];hand=[z for z in hand if z['name']!=a.a.REPLACED]+[z for z in entry if z['installed_part_candidate']];assert len(hand)==397
    elect=p.read(ROOT/'cad/remote_motor_lead_v2/generated/scene.json')['parts'];ws=p.read(ROOT/'cad/paired_rear_guide_v1/generated/assembly_scene.json');wrist=[z for z in ws['new_parts']+ws['retained_parts'] if z['installed_part_candidate']];arm=[]
    for z in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference']:
        f='cad/modular_roots_v9/meshes/'+z['asset']+'.stl';arm.append(dict(name=z['name'],mesh=f,sha256_stl=r.sha(ROOT/f),color=[.6,.65,.7]))
    base=hand+pack+elect+arm+wrist;new=p.read(ROOT/raw['scene'])['parts'];common=p.read(ROOT/prec['evidence_scene'])['parts']
    def scene(meta,alpha=.26):
        parts=[]
        for z in meta:
            path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl'];parts.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),color=z.get('color',[.56,.64,.7]),alpha=1 if z in common else .8 if z in new else alpha,neutral_inverse=np.eye(4)))
        return dict(parts=parts,design=g.DESIGN)
    before=scene(base+[z for z in support if not z['installed_part_candidate']]);after=scene(base+new+common);view=r.view_fit([before,after],[dict(angles_deg=[0.]*20)],np.array([.8,-.9,.5]),1.)
    def render(ss,view,size=850):
        try:
            ss['_render_state']=r.make_renderer(ss,{'v':view},size,engine)
            for j,z in enumerate(ss['parts']):ss['_render_state']['model'].geom_rgba[j,3]=z['alpha']
            return r.render_one(ss,[0.]*20,'v',{'v':view},size,engine)[0]
        finally:
            if '_render_state' in ss:ss['_render_state']['renderer'].close()
    out=HERE/'images';out.mkdir(exist_ok=True);r.sheet([('이전: 현재 하드웨어와 모터 국부 참고만',render(before,view)),('새 편손 후보: 두 피복과 선언된 양끝 사이 줄 연결',render(after,view))],2,850,'새 실제 출구를 잇는 두 원호 후보 · 전체 운동/설치는 미채택','실제 손397/포장385/전기64/팔10/PAIR6 유지 · 빨강은 실제 공통재료 · 길이 유지/힘·장력·외형 승인이 아님').save(out/'connection_before_after.png')
    if common:
        z=max(common,key=lambda z:z['pair']['material_intersection_mm3']);names=[z['pair']['lead'],z['pair']['part']];lookup={z['name']:z for z in base+new};local=scene([lookup[n] for n in names]+[z],.3);solo=scene([z]);cv=r.view_fit([local],[dict(angles_deg=[0.]*20)],np.array([.7,-1,.6]),1.);sv=r.view_fit([solo],[dict(angles_deg=[0.]*20)],np.array([.7,-1,.6]),1.);[(part.update(alpha=.22)) for part in local['parts'] if part['name']!=z['name']];r.sheet([('같은 줄의 꼬리/감김 CAD 조각과 접합부',render(local,cv)),('같은 공통재료만 확대 · 별도 배율',render(solo,sv))],2,850,'같은 구동줄의 인접 CAD 조각 접합부',z['pair']['lead']+' / '+z['pair']['part']+' · 기본체적 '+str(round(z['pair']['material_intersection_mm3'],6))+' mm³').save(out/'adjacent_seam.png')
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams['font.family']='Malgun Gothic';plt.rcParams['axes.unicode_minus']=False
    fig,ax=plt.subplots(figsize=(10,4));labels=['편손','부모25%','부모50%','부모75%','부모100%','손목-20/-15','손목+20/+15']
    for i in [0,1]:
        vals=[z['lines'][i]['housing_length_mm']-an['cases'][0]['lines'][i]['housing_length_mm'] for z in an['cases']];ax.plot(labels,vals,marker='o',label=['아래 선','위 선'][i])
    ax.axhline(0,color='gray',lw=.7);ax.set_ylabel('편손 대비 규정 경로 길이 변화 mm');ax.set_title('같은 길이 재료의 운동이 아닌, 각 자세별 두 원호 기하');ax.legend();ax.grid(alpha=.2);fig.tight_layout();fig.savefig(out/'length_diagnostic.png',dpi=150);plt.close(fig)
    source['cad/biarc_supported_link_v1/render.py']=m.drive.sha(Path(__file__));a.save(out/'report.json',dict(completed=True,source_sha256=source,outputs={z.name:r.sha(z) for z in out.iterdir() if z.is_file()},native_recomputed=0,actual_saved_CAD=True,old_local_references_not_duplicated_in_after_scene=True,diagnostic_is_variable_length_geometry_not_material_motion=True))
if __name__=='__main__':main()
