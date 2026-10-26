"""Actual first common material and selected current-motor terminal meshes."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import terminal_selected as model
g,c,p,np,cq,ROOT=model.g,model.c,model.p,model.np,model.cq,model.ROOT
HERE=Path(__file__).resolve().parent
helper=model.drive.module('_terminal022_render_helpers',ROOT/'cad/thumb_root_guide_v1/guide_render.py')
r,engine=helper.r,helper.engine
from PIL import ImageDraw
def main():
    out=HERE/'images';out.mkdir(exist_ok=False);source={}
    old=p.read(HERE/'reviews/check.json');new=p.read(HERE/'selected/reviews/check.json');assert old['completed'] and new['static_candidate_clear']
    oldmeta=p.read(ROOT/old['scene'])['parts'];meta=p.read(ROOT/new['scene'])['parts'];base=p.read(ROOT/'cad/motor_spool_coupling_v1/selected/generated/scene.json');fixed=base['parts']+base['retained_parts']
    frame=next(z for z in c.load(oldmeta) if 'terminal_frame' in z['name']);adapter=next(z for z in c.load([z for z in fixed if z['name']=='thumb_ip_actuator_adapter']))
    common=frame['shape'].intersect(adapter['shape'],tol=1e-6);assert abs(common.Volume()-266.75495697879103)<1e-5
    ev=HERE/'generated/visual_evidence';ev.mkdir(parents=True,exist_ok=False);st=ev/'initial_frame_adapter_common.step';me=st.with_suffix('.stl');cq.exporters.export(common,str(st));cq.exporters.export(common,str(me),tolerance=.03,angularTolerance=.1)
    hit=dict(name='actual_initial_frame_adapter_common',step=st.relative_to(ROOT).as_posix(),mesh=me.relative_to(ROOT).as_posix(),sha256_step=r.sha(st),sha256_stl=r.sha(me),body='palm',color=[.95,.02,.04],installed_part=False,actual_common_mm3=common.Volume())
    p.write(ev/'scene.json',dict(parts=[hit],evidence_only=True))
    selected_base=[z for z in fixed if (z.get('module')==model.NAME or z['name'].startswith(model.NAME+'_')) and z['name'] not in new['design']['replaced_packaging']]
    def assembly(fresh,extra=None,cords=False):
        result=[]
        def add(z,alpha,color=None):
            path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl']
            result.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),color=color or z.get('color',[.6,.66,.72]),alpha=alpha,neutral_inverse=np.eye(4)))
        for z in selected_base:add(z,.22,[.64,.69,.74])
        for z in fresh:add(z,.65 if 'terminal_frame' in z['name'] else 1.)
        if cords:
            for rel in [new['cord_scene'],new['housing_scene']]:
                for z in p.read(ROOT/rel)['parts']:add(z,1.)
        if extra:add(extra,1.)
        return dict(parts=result,design=g.DESIGN)
    scenes=[assembly(oldmeta,extra=hit),assembly(meta,cords=True)]
    view=r.view_fit(scenes,[dict(angles_deg=[0.]*20)]*2,np.array([.5,-.55,-1.]),1.)
    imgs=[]
    try:
        for a in scenes:
            a['_render_state']=r.make_renderer(a,{'view':view},1000,engine)
            for i,z in enumerate(a['parts']):a['_render_state']['model'].geom_rgba[i,3]=z['alpha']
            im,_=r.render_one(a,[0.]*20,'view',{'view':view},1000,engine);imgs.append(im)
        point=np.asarray(common.Center().toTuple())-view['focus'];factor=1000/(2*view['parallel_scale_mm']);x=500+np.dot(point,view['right'])*factor;y=500-np.dot(point,view['up'])*factor
        d=ImageDraw.Draw(imgs[0]);d.ellipse((x-40,y-40,x+40,y+40),outline=(210,15,30),width=4);d.text((max(10,x-90),y+45),'실제 체결부 겹침',font=r.font(25),fill=(185,20,30))
        r.sheet([('첫 지지대: 기존 체결부와 실제 겹침',imgs[0]),('교정: 체결부 옆으로 지지대 이동',imgs[1])],2,1000,
            '모터 쪽 두 줄 통로·피복 받침·길이 조절부 후보',
            '청록: 나사 두 곳에 붙는 일체형 지지대/곡선 통로 · 금색: 조절부 · 보라: 짧은 피복 참고 · 손까지 긴 피복/장력 미연결').save(out/'terminal_before_after.png')
    finally:
        for a in scenes:
            if '_render_state' in a:a['_render_state']['renderer'].close()
    assert all(r.sha(ROOT/z)==h for z,h in source.items())
    p.write(out/'report.json',dict(completed=True,source_sha256=source,outputs={z.name:r.sha(z) for z in out.iterdir() if z.is_file()},simulation_steps=0,asset_compile_threads=False,actual_saved_meshes=True,whole_hand_drive_approved=False))
if __name__=='__main__':main()
