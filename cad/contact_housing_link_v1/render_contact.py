"""Actual new connected CAD candidate and any native common material."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import build_contact as b
s,a,m,g,c,p,np,cq,ROOT,HERE=b.s,b.a,b.m,b.g,b.c,b.p,b.np,b.cq,b.ROOT,b.HERE
helper=m.drive.module('_contact036_render',ROOT/'cad/thumb_root_guide_v1/guide_render.py');r,engine=helper.r,helper.engine
def main():
    raw=p.read(HERE/'reviews/check.json');assert raw['completed'];source={};hand=p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')['hand'];hand=[z for z in hand if z['name'] not in ['thumb_segment2_dark_faceplate_clamp1_bolt','thumb_segment2_dark_faceplate_clamp2_bolt']];d=p.read(ROOT/'cad/thumb_sheath_terminal_v2/selected/generated/candidate_scene.json');ret=p.read(ROOT/'cad/sheath_retention_v1/selected/generated/parts/scene.json')['parts'];hand+=d['retained_guide_parts']+[z for z in d['parts'] if z['name'] not in ['distal_flanged_ferrule_0','distal_flanged_ferrule_1']]+[z for z in ret if z['body']=='thumb_mcp'];assert len(hand)==397
    entry=p.read(ROOT/'cad/thumb_entry_bend_v1/generated/parts/scene.json')['parts'];entry=[z for z in entry if z['installed_part_candidate']];pack=p.read(ROOT/'cad/motor_tendon_terminal_v1/selected/generated/assembly_scene.json');pack=pack['new_parts']+pack['retained_parts']+[z for z in ret if z['body']=='forearm'];elec=p.read(ROOT/'cad/remote_motor_lead_v2/generated/scene.json')['parts'];oldw=p.read(ROOT/'cad/wrist_external_feed_v1/generated/assembly_scene.json');oldw=[z for z in oldw['new_parts']+oldw['retained_parts'] if z['installed_part_candidate']];neww=p.read(ROOT/'cad/paired_rear_guide_v1/generated/assembly_scene.json');neww=[z for z in neww['new_parts']+neww['retained_parts'] if z['installed_part_candidate']];old=p.read(ROOT/'cad/elastic_housing_link_v1/generated/pose_0/scene.json')['parts'];fresh=p.read(ROOT/raw['scene'])['parts'];commons=[]
    overlaps=[z for t in raw['native'].values() for z in t['overlaps']]+[z for t in raw['internal'] for z in t['overlaps']]
    if overlaps:
        data,pp,guide,ww=s.prior.inputs();hh,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,np.zeros(20));hh+=c.load(entry,data[0]);parts=c.load(fresh,data[0]);lookup={z['name']:z for z in hh+pp+data[6]+data[7]+ww+parts};selected=[]
        for group,items in [('external',[z for t in raw['native'].values() for z in t['overlaps']]),('internal',[z for t in raw['internal'] for z in t['overlaps']])]:
            if not items:continue
            z=max(items,key=lambda v:v['material_intersection_mm3']);common=lookup[z['lead']]['shape'].intersect(lookup[z['part']]['shape'],tol=1e-6);selected.append(dict(name='actual_'+group+'_common',shape=common,body='palm',color=[1.,.06,.04],evidence_only=True,original_pair=z))
        common_scene=a.prior.export(selected,HERE/'generated/actual_common');commons=p.read(ROOT/common_scene)['parts'];p.write(HERE/'reviews/actual_common.json',dict(completed=True,scene=common_scene,only_selected_native_pairs_recomputed=len(selected),whole_groups_recomputed=0))
    def scene(before=False,local=False):
        parts=[]
        def add(z,alpha):
            path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl'];parts.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),color=z.get('color',[.58,.64,.7]),alpha=alpha,neutral_inverse=np.eye(4)))
        for z in hand:
            if not before and z['name']==a.REPLACED:continue
            if local and not (z['body']=='palm' or z['body'].startswith('thumb')):continue
            add(z,.17)
        if not before:
            for z in entry:add(z,.9)
        if not local:
            for z in pack:add(z,.2)
            for z in elec:add(z,.35)
        for z in (oldw if before else neww):add(z,.8)
        for z in (old if before else fresh):add(z,.85 if z.get('kind')=='housing' else .9)
        if not before:
            for z in commons:add(z,1.)
        return dict(parts=parts,design=g.DESIGN)
    scenes=[scene(True),scene()];view=r.view_fit(scenes,[dict(angles_deg=[0.]*20)],np.array([.8,-.8,.5]),1.);images=[]
    try:
        for ss in scenes:
            ss['_render_state']=r.make_renderer(ss,{'v':view},1100,engine)
            for j,z in enumerate(ss['parts']):ss['_render_state']['model'].geom_rgba[j,3]=z['alpha']
            im,_=r.render_one(ss,[0.]*20,'v',{'v':view},1100,engine);images.append(im)
        folder=HERE/'images';folder.mkdir(exist_ok=False);status='접촉/길이 조건과 재료검사 모두 맞춘 기하 후보' if raw['candidate_admissible'] else '길이/최적화 또는 실제 재료검사가 남은 미채택 후보';r.sheet([('이전: FEED031/기존종단의 독립 막대 실패',images[0]),('이번: PAIR034/새엄지종단을 잇는 접촉 결합 계산',images[1])],2,1100,'실제 지지를 따르는 두 피복/중심줄의 편손 연결 후보',status+' · 피복OD3/줄0.6 유지 · 접촉/길이는 계산 범위 · 실제힘·장력/왕복·설치/외형승인 아님').save(folder/'contact_before_after.png')
    finally:
        for ss in scenes:
            if '_render_state' in ss:ss['_render_state']['renderer'].close()
    source['cad/contact_housing_link_v1/render_contact.py']=m.drive.sha(Path(__file__));p.write(HERE/'images/report.json',dict(completed=True,source_sha256=source,outputs={z.name:r.sha(z) for z in (HERE/'images').iterdir() if z.is_file()},actual_CAD=True,whole_hand_drive_approved=False,old_rejected_candidate_visual_only=True))
if __name__=='__main__':main()
