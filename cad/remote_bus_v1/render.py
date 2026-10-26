"""Same-scale actual CAD views of the two partial main runs."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
import model as g
import mujoco
from PIL import Image,ImageDraw

p,b,m,np,HERE,ROOT=g.p,g.b,g.m,g.np,g.HERE,g.ROOT
sys.path.insert(0,str(ROOT/'cad/gesture_hand_v3'))
import poses
r=g.module('_bus_render',ROOT/'cad/gesture_hand_v3/render_gestures.py')


def main():
    out=HERE/'images'
    if out.exists():raise FileExistsError('Preserve earlier figures')
    report=p.read(HERE/'reviews/final.json');assert report['completed'] and report['nominal_space_clear'] and report['continuity_confirmed']
    saved=p.read(ROOT/'cad/palm_motor_mount_v1/selected3/generated/scene.json')
    remote_modules={z['name'] for z in p.read(ROOT/'cad/palm_motor_layout_v1/selected/generated/layout.json')['modules'] if z['zone']!='palm'}
    package=[z for z in saved['new_parts']+saved['retained_parts'] if z.get('body')=='forearm' or z.get('module') in remote_modules]
    assert len(package)==321
    old=p.read(ROOT/'cad/remote_motor_lead_v2/generated/scene.json')['parts'];new=p.read(HERE/'generated/scene.json')['parts']
    design=p.read(ROOT/'cad/functional_motion_v3/generated/catalog.json')['design']
    sources={};assemblies=[];arm_meshes=[]
    folder=HERE/'generated/arm_visual_meshes';folder.mkdir(exist_ok=False)
    survey=p.read(HERE/'reviews/source_survey.json')
    for row in survey['neutral_arm']:
        assert b.sha(ROOT/row['step'])==survey['source_sha256'][row['step']]
        sources[row['step']]=survey['source_sha256'][row['step']]
        path=folder/(row['name']+'.stl')
        g.cq.exporters.export(g.cq.importers.importStep(str(ROOT/row['step'])).val(),str(path),tolerance=.1,angularTolerance=.15)
        arm_meshes.append(dict(name=row['name'],mesh=path.relative_to(ROOT).as_posix(),sha256_stl=b.sha(path)))
    for addition in [[],new]:
        parts=[]
        def add(row,color,alpha):
            path=ROOT/row['mesh'];assert b.sha(path)==row['sha256_stl'];sources[row['mesh']]=row['sha256_stl']
            parts.append(dict(name=row['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),color=color,alpha=alpha,neutral_inverse=np.eye(4)))
        for row in package:add(row,[.2,.4,.6],.45)
        for row in old:add(row,[.48,.52,.56],.65)
        for row in arm_meshes:add(row,[.66,.69,.72],.15)
        for row in addition:add(row,row['color'],.55 if 'reservation' in row['role'] else 1.)
        assemblies.append(dict(parts=parts,design=design))
    camera=r.view_fit(assemblies,[dict(angles_deg=np.zeros(20))],np.array([.12,0,-1.]),1.)
    images=[];bounds=[]
    try:
        for assembly in assemblies:
            state=r.make_renderer(assembly,{'bus':camera},1100,mujoco);assembly['_render_state']=state
            for i,z in enumerate(assembly['parts']):state['model'].geom_rgba[i,3]=z['alpha']
            im,bb=r.render_one(assembly,np.zeros(20),'bus',{'bus':camera},1100,mujoco);images.append(im);bounds.append(bb)
    finally:
        for assembly in assemblies:
            if '_render_state' in assembly:assembly['_render_state']['renderer'].close()
    union=np.asarray(bounds);crop=[max(0,int(union[:,0].min())-35),max(0,int(union[:,1].min())-15),
                                min(1100,int(union[:,2].max())+36),min(1100,int(union[:,3].max())+16)]
    tiles=[im.crop(crop) for im in images];w,h=tiles[0].size
    canvas=Image.new('RGB',(2*w+40,h+175),r.BG);draw=ImageDraw.Draw(canvas)
    draw.text((18,12),'원격 배선 공간 후보 · 3선 묶음 2개',font=r.font(28),fill=r.INK)
    for i,(label,im) in enumerate(zip(['현재: 48선의 끝이 열려 있음','후보: 두 묶음을 임시 종단까지'],tiles)):
        draw.text((i*(w+20)+12,62),label,font=r.font(21),fill=r.INK);canvas.paste(im,(i*(w+20)+10,99))
    draw.text((18,h+113),'보라색: 부품/보드 미정의 임시 공간 · 나머지14모터 연결/고정은 미완',font=r.font(17),fill=r.INK)
    draw.text((18,h+141),'중립 팔의 정적 CAD · 실제 결선/구동/관절 케이블 운동 승인 아님',font=r.font(17),fill=r.INK)
    out.mkdir();canvas.save(out/'main_runs.png')
    assert all(b.sha(ROOT/z)==h for z,h in sources.items())
    p.write(out/'report.json',dict(completed=True,input_mesh_sha256=sources,outputs={'main_runs.png':b.sha(out/'main_runs.png')},
        camera=camera,common_crop_px=crop,foreground_bounds_px=bounds,simulation_steps=0,
        scope='Actual saved forearm CAD meshes in common scale;hand omitted from this static wiring view.No powered/flexible-wire or complete bus claim.'))


if __name__=='__main__':main()
