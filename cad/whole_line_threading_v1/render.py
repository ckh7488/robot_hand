"""Actual saved CAD comparison for the partial assembly branch."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import thread_model as t
a,m,g,p,np,ROOT,HERE=t.a,t.m,t.g,t.p,t.np,t.ROOT,t.HERE
helper=m.drive.module('_thread040_render',ROOT/'cad/thumb_root_guide_v1/guide_render.py');r,engine=helper.r,helper.engine


def current_metadata():
    ret=p.read(ROOT/'cad/sheath_retention_v1/selected/generated/parts/scene.json')['parts'];ps=p.read(ROOT/'cad/motor_tendon_terminal_v1/selected/generated/assembly_scene.json');pack=ps['new_parts']+ps['retained_parts']+[z for z in ret if z['body']=='forearm'];support=p.read(ROOT/'cad/support_attachment_basis_v1/generated/checked/scene.json')['parts'];newhw=[z for z in support if z['installed_part_candidate']];removed=[z['replaced_part'] for z in newhw];pack=[z for z in pack if z['name'] not in removed]+newhw;assert len(pack)==385
    hand=p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')['hand'];hand=[z for z in hand if z['name'] not in ['thumb_segment2_dark_faceplate_clamp1_bolt','thumb_segment2_dark_faceplate_clamp2_bolt']];offset=p.read(ROOT/'cad/thumb_sheath_terminal_v2/selected/generated/candidate_scene.json');hand+=offset['retained_guide_parts']+[z for z in offset['parts'] if z['name'] not in ['distal_flanged_ferrule_0','distal_flanged_ferrule_1']]+[z for z in ret if z['body']=='thumb_mcp'];entry=p.read(ROOT/'cad/thumb_entry_bend_v1/generated/parts/scene.json')['parts'];hand=[z for z in hand if z['name']!=a.a.REPLACED]+[z for z in entry if z['installed_part_candidate']];assert len(hand)==397
    elect=p.read(ROOT/'cad/remote_motor_lead_v2/generated/scene.json')['parts'];ws=p.read(ROOT/'cad/paired_rear_guide_v1/generated/assembly_scene.json');wrist=[z for z in ws['new_parts']+ws['retained_parts'] if z['installed_part_candidate']];arm=[]
    for z in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference']:
        f='cad/modular_roots_v9/meshes/'+z['asset']+'.stl';arm.append(dict(name=z['name'],mesh=f,sha256_stl=r.sha(ROOT/f),color=[.6,.65,.7]))
    return hand+pack+elect+arm+wrist


def main():
    raw=p.read(HERE/'reviews/check.json');assert raw['completed'] and raw['sequence_geometry_clear'];source=dict(raw['source_sha256']);base=current_metadata();before_parts=p.read(ROOT/'cad/biarc_supported_link_v1/generated/neutral/scene.json')['parts'];after_parts=[]
    for plan in raw['plans']:
        last=next(z for z in reversed(raw['cases']) if z['kind']==plan['kind'] and z['line']==plan['line']);after_parts+=p.read(ROOT/last['scene'])['parts']
    def scene(meta,new):
        parts=[]
        for z in meta:
            path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl'];parts.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),color=z.get('color',[.56,.64,.7]),alpha=.9 if z in new else .23,neutral_inverse=np.eye(4)))
        return dict(parts=parts,design=g.DESIGN)
    before=scene(base+before_parts,before_parts);after=scene(base+after_parts,after_parts);view=r.view_fit([before,after],[dict(angles_deg=[0.]*20)],np.array([.8,-.9,.5]),1.)
    def render(ss):
        try:
            ss['_render_state']=r.make_renderer(ss,{'v':view},900,engine)
            for j,z in enumerate(ss['parts']):ss['_render_state']['model'].geom_rgba[j,3]=z['alpha']
            return r.render_one(ss,[0.]*20,'v',{'v':view},900,engine)[0]
        finally:
            if '_render_state' in ss:ss['_render_state']['renderer'].close()
    out=HERE/'images';out.mkdir(exist_ok=True);r.sheet([('이전 편손 연결 후보: 설치·운동은 미채택',render(before)),('이번 끼우기 순서 끝: 긴 모터 쪽 꼬리는 자유 상태',render(after))],2,900,'전체 길이 피복·줄의 조립 경로 후보 · 실제 CAD 비교','손·모터·전기·안내대 유지 / 엄지 쪽까지 규정 경로로 투입 / 모터 쪽 안착·감김·고정, 힘·마찰·연속 운동은 미검증').save(out/'threading_before_after.png')
    source['cad/whole_line_threading_v1/render.py']=m.drive.sha(Path(__file__));a.save(out/'report.json',dict(completed=True,source_sha256=source,outputs={z.name:r.sha(z) for z in out.iterdir() if z.is_file()},native_recomputed=0,saved_actual_CAD=True,after_excludes_old_housings_cords_and_axis_probes=True,motor_end_not_installed=True))
if __name__=='__main__':main()
