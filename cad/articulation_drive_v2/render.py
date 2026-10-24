"""Actual CAD mesh preview of the preserved articulated form and root repairs."""
from pathlib import Path
import argparse
import hashlib
import importlib.util
import json
import sys
import numpy as np
import mujoco
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial.transform import Rotation

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
FONT='C:/Windows/Fonts/malgun.ttf'


def body_key(name):
    if name=='palm':return 'world'
    finger=name.split('_',1)[0];suffix=name[len(finger)+1:]
    labels=['base_sweep','opposition','mcp','ip'] if finger=='thumb' else ['spread','mcp','pip','dip']
    return finger,labels.index(suffix)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,default=HERE/'generated')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    sys.path.insert(0,str(ROOT/'cad/webcam_articulation_v1'))
    spec=importlib.util.spec_from_file_location('saved_pose_frames',ROOT/'cad/webcam_articulation_v1/verify.py')
    pose=importlib.util.module_from_spec(spec);spec.loader.exec_module(pose)
    design=json.loads((HERE/'design.json').read_text());scene=json.loads((HERE/'generated/scene.json').read_text())
    rows=scene['hand'];paths=[HERE/'design.json',HERE/'generated/scene.json',Path(__file__),ROOT/'cad/webcam_articulation_v1/verify.py',ROOT/'cad/webcam_articulation_v1/thumb_shell.py']
    paths += [HERE/r['mesh'] for r in rows]
    hashes=lambda:{p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    before=hashes();neutral=pose.frames(design,np.zeros(20))
    for row in rows:
        if before[(HERE/row['mesh']).relative_to(ROOT).as_posix()]!=row['sha256_stl']:
            raise ValueError('Scene/hand mesh mismatch: '+row['name'])
    panels=[('편 손 · 새 엄지 유지',np.zeros(20),90,-90,False),
            ('주먹 · 정면',np.array(design['fist_deg']),90,-90,False),
            ('주먹 · 사선',np.array(design['fist_deg']),135,-42,False),
            ('뿌리 연결부 · 내부',np.zeros(20),125,27,True)]
    canvas=Image.new('RGB',(2400,1610),'#eff4fa');draw=ImageDraw.Draw(canvas)
    draw.text((30,20),'변경된 손 형상에서 뿌리 관절 개선',font=ImageFont.truetype(FONT,34),fill='#203449')
    draw.text((30,69),'실제 CAD · 손 비율과 주먹 자세 유지 · 조립/간섭과 구동 성능은 검토 중',font=ImageFont.truetype(FONT,23),fill='#435870')
    for panel,(label,angles,az,el,cut) in enumerate(panels):
        frames=pose.frames(design,angles);assets=[];bodies=[]
        for i,row in enumerate(rows):
            if cut and (row['name'].startswith('Cover_') or row['name'] in ('Palm_shaped_front_cover','Palm_tapered_chassis')):continue
            key=body_key(row['body']);o,r=frames[key];on,rn=neutral[key]
            rot=r@rn.T;pos=(o-rot@on)*.001;quat=Rotation.from_matrix(rot).as_quat()[[3,0,1,2]]
            path=(HERE/row['mesh']).resolve().as_posix()
            assets.append(f'<mesh name="m{i}" file="{path}" scale=".001 .001 .001" maxhullvert="64"/>')
            fmt=lambda x:' '.join(str(v) for v in x)
            bodies.append(f'<body pos="{fmt(pos)}" quat="{fmt(quat)}"><geom type="mesh" mesh="m{i}" rgba="{fmt(row["color"][:3])} 1" contype="0" conaffinity="0"/></body>')
        xml='<mujoco><visual><global offwidth="1200" offheight="1200"/><headlight ambient=".65 .65 .65" diffuse=".5 .5 .5" specular=".1 .1 .1"/></visual><asset><texture type="skybox" builtin="flat" rgb1=".94 .96 .98" rgb2=".94 .96 .98" width="256" height="1536"/>'+''.join(assets)+'</asset><worldbody><light pos=".4 .3 .9" diffuse=".5 .5 .5"/>'+''.join(bodies)+'</worldbody></mujoco>'
        model=mujoco.MjModel.from_xml_string(xml);data=mujoco.MjData(model);mujoco.mj_forward(model,data)
        renderer=mujoco.Renderer(model,height=1000,width=1000)
        cam=mujoco.MjvCamera();cam.lookat[:]=[0,.12,0];cam.distance=.95;cam.azimuth=az;cam.elevation=el
        renderer.update_scene(data,cam)
        for camera in renderer.scene.camera:
            fw=np.array(camera.forward);up=np.array([0.,1.,0.]);up-=fw*np.dot(up,fw);camera.up[:]=up/np.linalg.norm(up)
        pixels=renderer.render();renderer.close();im=Image.fromarray(pixels)
        border=np.concatenate([pixels[0],pixels[-1],pixels[:,0],pixels[:,-1]])
        yy,xx=np.where(np.max(np.abs(pixels.astype(float)-np.median(border,axis=0)),axis=2)>12)
        if len(xx):im=im.crop((max(0,int(xx.min())-20),max(0,int(yy.min())-20),min(im.width,int(xx.max())+21),min(im.height,int(yy.max())+21)))
        k=min(1120/im.width,635/im.height);im=im.resize((round(im.width*k),round(im.height*k)),Image.Resampling.LANCZOS)
        x=(panel%2)*1200;y=130+(panel//2)*720
        draw.text((x+30,y),label,font=ImageFont.truetype(FONT,26),fill='#203449')
        canvas.paste(im,(x+(1200-im.width)//2,y+44+(635-im.height)//2))
    draw.text((30,1570),'검사 결과와 미해결 항목을 함께 확인해야 합니다. 제작 승인 또는 100ms 성능 증거가 아닙니다.',font=ImageFont.truetype(FONT,21),fill='#435870')
    name='robot_hand_drive002.png';canvas.save(args.output/name)
    after=hashes()
    if before!=after:raise RuntimeError('Render input changed')
    (args.output/'render_inputs.json').write_text(json.dumps(dict(input_sha256=before,inputs_unchanged=True,
        image_sha256=hashlib.sha256((args.output/name).read_bytes()).hexdigest(),
        geometry_scope='Actual candidate meshes in saved neutral/fist poses; covers hidden only in internal view'),indent=2))
    print(str(args.output/name),flush=True)


if __name__=='__main__':main()
