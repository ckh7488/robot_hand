"""Saved CAD views of the two terminal clamps and actual failed material."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import retention_model as m
g,c,p,np,cq,ROOT,HERE=m.g,m.c,m.p,m.np,m.cq,m.ROOT,m.HERE
helper=m.drive.module('_retain026_renderer',ROOT/'cad/thumb_root_guide_v1/guide_render.py');r,engine=helper.r,helper.engine
from PIL import ImageDraw
def main():
    out=HERE/'images';out.mkdir(exist_ok=False);report=p.read(HERE/'selected/reviews/check.json');raw=p.read(HERE/'reviews/check.json');assert report['completed'];source={}
    fresh=p.read(ROOT/report['scene'])['parts'];hand=p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')['hand']
    root=p.read(ROOT/'cad/thumb_sheath_terminal_v2/selected/generated/candidate_scene.json');guide=root['parts']+root['retained_guide_parts']
    pack=p.read(ROOT/'cad/motor_tendon_terminal_v1/selected/generated/assembly_scene.json');motor=[z for z in pack['new_parts']+pack['retained_parts'] if z['name'].startswith('thumb_ip_')]
    distalhousing=p.read(ROOT/'cad/thumb_sheath_terminal_v2/selected/generated/housing_reference/scene.json')['parts'];motorhousing=p.read(ROOT/'cad/motor_tendon_terminal_v1/selected/generated/housing_reference/scene.json')['parts']
    def scene(rows):
        parts=[]
        for z,alpha in rows:
            path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl'];parts.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),color=z.get('color',[.65,.69,.73]),alpha=alpha,neutral_inverse=np.eye(4)))
        return dict(parts=parts,design=g.DESIGN)
    motorbase=[(z,.32 if 'terminal_frame' not in z['name'] else .85) for z in motor]+[(z,1.) for z in motorhousing]
    handbase=[(z,.23) for z in hand if z['body'].startswith('thumb') and z['name'] not in ['thumb_segment2_dark_faceplate_clamp1_bolt','thumb_segment2_dark_faceplate_clamp2_bolt']]
    olddist=handbase+[(z,.9) for z in guide]+[(z,1.) for z in distalhousing]
    newdist=handbase+[(z,.85) for z in guide if z['name'] not in report['design']['replaced_distal_parts']]+[(z,.95) for z in fresh if z['body']=='thumb_mcp']+[(z,1.) for z in distalhousing]
    scenes=[scene(motorbase),scene(motorbase+[(z,1.) for z in fresh if z['body']=='forearm']),scene(olddist),scene(newdist)]
    views=[r.view_fit(scenes[:2],[dict(angles_deg=[0.]*20)]*2,np.array([.4,-.8,-.6]),1.),r.view_fit(scenes[2:],[dict(angles_deg=[0.]*20)]*2,np.array([.4,-.5,1.]),1.)];imgs=[]
    try:
        for i,a in enumerate(scenes):
            view=views[i//2];a['_render_state']=r.make_renderer(a,{'v':view},900,engine)
            for j,z in enumerate(a['parts']):a['_render_state']['model'].geom_rgba[j,3]=z['alpha']
            im,_=r.render_one(a,[0.]*20,'v',{'v':view},900,engine);imgs.append(im)
        r.sheet(list(zip(['기존 모터 종단: 피복 압축 어깨','새 모터 종단: 분리형 피복 죔쇠 후보','기존 손 종단: 끝구 플랜지만 고정','새 손 종단: 죔쇠/끝구 포착 턱 후보'],imgs)),2,900,'양 끝 피복 빠짐방지 후보, 실제 당김 저항은 미검증','청록: 분리 죔쇠 · 금색: 기존 종단/새 포착 턱 · 보라: 짧은 피복 참고 · 긴 피복/줄·장력은 아직 미연결').save(out/'retention_before_after.png')
    finally:
        for a in scenes:
            if '_render_state' in a:a['_render_state']['renderer'].close()
    # Save only the actual intersection volume, not a red whole hand.
    failed=[(a,z) for a in raw['cases'] for z in a['native']['hand']['overlaps']]
    if failed:
        case,hit=max(failed,key=lambda x:x[1]['material_intersection_mm3']);check=m.drive.module('_retain026_visual_check',HERE/'check.py');data,pack,retained,housing=check.inputs(report['design']);rows=c.load(p.read(ROOT/raw['scene'])['parts']);hr,v,f,neutral=m.s.k.e.prior.hand_rows(data,retained,case['angles_deg']);a=next(z for z in rows if z['name']==hit['lead']);a=dict(a,shape=v.relocate(a['shape'],f[a['body']],neutral[a['body']])) if a['body']!='forearm' else a;b=next(z for z in hr if z['name']==hit['part']);common=a['shape'].intersect(b['shape'],tol=1e-6);assert abs(common.Volume()-hit['material_intersection_mm3'])<1e-5
        folder=HERE/'generated/visual_evidence';folder.mkdir(exist_ok=False);meta=[]
        for name,sh,color in [('failed_clamp_stage',a['shape'],[.15,.65,.7]),('failed_existing_part',b['shape'],[.7,.73,.77]),('actual_failed_common',common,[.95,.02,.04])]:
            st=folder/(name+'.step');me=st.with_suffix('.stl');cq.exporters.export(sh,str(st));cq.exporters.export(sh,str(me),tolerance=.025,angularTolerance=.1);meta.append(dict(name=name,step=st.relative_to(ROOT).as_posix(),mesh=me.relative_to(ROOT).as_posix(),sha256_step=r.sha(st),sha256_stl=r.sha(me),color=color,installed_part=False))
        p.write(folder/'scene.json',dict(parts=meta,case=case['id'],actual_common_mm3=common.Volume(),evidence_only=True))
        a=scene([(z,.38 if z['name']=='failed_existing_part' else 1.) for z in meta]);view=r.view_fit([a],[dict(angles_deg=[0.]*20)],np.array([.4,-.5,1.]),1.)
        try:
            a['_render_state']=r.make_renderer(a,{'v':view},1000,engine)
            for j,z in enumerate(a['parts']):a['_render_state']['model'].geom_rgba[j,3]=z['alpha']
            im,_=r.render_one(a,[0.]*20,'v',{'v':view},1000,engine);r.sheet([('기존 손과 새 죔쇠의 실제 공통 재료',im)],1,1000,'미채택 후보의 실제 걸림',case['id']+' / '+hit['lead']+' / '+hit['part']+' / '+format(common.Volume(),'.9f')+'mm³').save(out/'actual_block.png')
        finally:a['_render_state']['renderer'].close()
    p.write(out/'report.json',dict(completed=True,source_sha256=source,outputs={z.name:r.sha(z) for z in out.iterdir() if z.is_file()},actual_saved_meshes=True,whole_hand_drive_approved=False,friction_pullout_validated=False))
if __name__=='__main__':main()
