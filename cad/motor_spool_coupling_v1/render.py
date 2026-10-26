"""Actual saved CAD, one motor before/after. No driven-hand simulation."""
import importlib.util,sys
from pathlib import Path
import mujoco
import numpy as np
from PIL import ImageDraw
from scipy.spatial.transform import Rotation
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
sys.dont_write_bytecode=True;sys.path.insert(0,str(ROOT/'cad/gesture_hand_v3'))
import poses
s=importlib.util.spec_from_file_location('_coupling_render',ROOT/'cad/gesture_hand_v3/render_gestures.py');r=importlib.util.module_from_spec(s);s.loader.exec_module(r)


def main():
    out=HERE/'images';out.mkdir(exist_ok=False)
    saved=r.read_json(HERE/'selected/generated/scene.json');raw=r.read_json(HERE/'selected/reviews/check.json')
    assert raw['completed']
    old=[z for z in saved['retained_parts']+saved['replaced_parts'] if z.get('module')=='thumb_ip_actuator']
    fixed=[z for z in old if not z['name'].endswith(('_rotor','_spool'))]
    # Rack bolts/guide far behind the motor do not help explain its output seat.
    fixed=[z for z in fixed if z['name'].endswith(('_motor','_cradle','_body_screws','_base_screws','_base_nuts','_adapter'))]
    old=fixed+[z for z in saved['replaced_parts']]
    first=fixed+r.read_json(HERE/'generated/scene.json')['parts']
    common=HERE/'selected/generated/visual_evidence/first_tail_boss_adapter_common.stl'
    first+=[dict(name='actual_common_evidence',mesh=common.relative_to(ROOT).as_posix(),sha256_stl=r.sha(common),color=[.92,.06,.08],evidence=True)]
    new=fixed+saved['parts']
    design=r.read_json(ROOT/'cad/functional_motion_v3/generated/catalog.json')['design']
    layout=r.read_json(ROOT/'cad/palm_motor_layout_v1/selected/generated/layout.json')
    T=np.asarray(next(z['transform_world_mm'] for z in layout['modules'] if z['name']=='thumb_ip_actuator'))
    def rotation(angle):
        world=np.eye(4);world[:3,:3]=Rotation.from_rotvec(T[:3,0]*np.deg2rad(angle)).as_matrix();world[:3,3]=T[:3,3]-world[:3,:3]@T[:3,3];return world
    assemblies=[];source={}
    for index,rows in enumerate([old,first,new]):
        parts=[]
        for z in rows:
            path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];source[z['mesh']]=z['sha256_stl']
            color=z.get('color',[.35,.4,.45]);alpha=.8
            if z['name'].endswith('_motor'):color=[.2,.24,.3];alpha=.5
            if index==1:alpha=.12
            if z['name'].endswith('_spool') or z.get('role')=='custom_counterbored_dual_groove_spool':color=[.95,.57,.12];alpha=.42
            if index==1 and z.get('role')=='custom_counterbored_dual_groove_spool':alpha=.15
            if z.get('role')=='unmodified_stock_reference_horn':color=[.16,.44,.82];alpha=1.
            if 'tap6_' in z['name']:color=[.15,.64,.4];alpha=1.
            if z.get('evidence'):color=[.92,.06,.08];alpha=1.
            parts.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),color=color,alpha=alpha,
                neutral_inverse=rotation(140.) if index and z.get('moving',False) else np.eye(4),moving=z.get('moving',False)))
        assemblies.append(dict(parts=parts,design=design))
    camera=r.view_fit(assemblies,[dict(angles_deg=np.zeros(20))],np.array([1.,-.55,-.65]),1.25)
    images=[]
    try:
        for assembly in assemblies:
            state=r.make_renderer(assembly,{'coupling':camera},850,mujoco);assembly['_render_state']=state
            for i,z in enumerate(assembly['parts']):state['model'].geom_rgba[i,3]=z['alpha']
            image,_=r.render_one(assembly,np.zeros(20),'coupling',{'coupling':camera},850,mujoco);images.append(image)
        center=r.read_stl_points(common).mean(axis=0)-np.asarray(camera['focus'])
        factor=850/(2*camera['parallel_scale_mm'])
        x=425+np.dot(center,camera['right'])*factor;y=425-np.dot(center,camera['up'])*factor
        draw=ImageDraw.Draw(images[1]);draw.ellipse((x-26,y-26,x+26,y+26),outline=(218,28,39),width=4)
        draw.line((x+26,y,x+125,y+40),fill=(218,28,39),width=3)
        draw.text((x+50,y+42),'실제 겹침',font=r.font(22),fill=(190,20,30))
        r.sheet([('이전: 출력 원판과 드럼 체결 없음',images[0]),('첫 후보 140°: 빨간 원이 실제 겹친 위치',images[1]),('교정 140°: 목 부분을 늘려 고정부 우회',images[2])],3,850,
            '대표 모터 출력부부터 연결 · 손과 모터 설치 위치는 유지',
            '주황: 투명하게 본 드럼 · 파랑: 기존 출력 원판 · 줄/손가락 연결·장력·체결강도는 아직 미완').save(out/'coupling.png')
        frames=[]
        for angle in range(0,91,10):
            world=rotation(angle)
            for z in assemblies[2]['parts']:z['neutral_inverse']=world if z['moving'] else np.eye(4)
            image,_=r.render_one(assemblies[2],np.zeros(20),'coupling',{'coupling':camera},850,mujoco)
            frames.append(r.sheet([('출력부 0→90→0° · 설명용 재생속도',image)],1,850,'모터 출력부만 회전한 CAD 후보','손가락은 구동하지 않음 · 줄·장력·공급 나사/실물은 미검증'))
        sequence=frames+frames[-2:0:-1]
        sequence[0].save(out/'output_rotation.gif',save_all=True,append_images=sequence[1:],duration=140,loop=0)
    finally:
        for assembly in assemblies:
            if '_render_state' in assembly:assembly['_render_state']['renderer'].close()
    assert all(r.sha(ROOT/z)==h for z,h in source.items())
    r.write_json(out/'report.json',dict(completed=True,input_sha256=source,actual_saved_meshes=True,simulation_steps=0,
        output_sha256={z.name:r.sha(z) for z in out.iterdir() if z.is_file()},scope='One current motor,transparent spool before/after and motor-output-only rotation,not a motor-driven hand or tendon dynamics.'))


if __name__=='__main__':main()
