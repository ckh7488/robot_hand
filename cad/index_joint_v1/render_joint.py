"""Render exact tessellated CAD. This is a visual render, NOT a torque simulation."""
import json,math,numpy as np,trimesh,mujoco
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
r=Path(__file__).resolve().parent;d=json.loads((r/'assembly_meshes.json').read_text());(r/'meshes').mkdir(exist_ok=True)
asset='';fixed='';moving=''
for n,a in d.items():
 trimesh.Trimesh(vertices=np.array(a['vertices'])*.001,faces=a['triangles'],process=False).export(r/'meshes'/(n+'.stl'))
 asset+=f'<mesh name="{n}" file="meshes/{n}.stl" maxhullvert="32"/>'
 color=' '.join(map(str,a['color']))+' 1';g=f'<geom name="{n}" type="mesh" mesh="{n}" rgba="{color}" contype="0" conaffinity="0" group="2"/>'
 if a['moving']:moving+=g
 else:fixed+=g
xml=f'<mujoco model="cad_visual_only"><visual><global offwidth="1600" offheight="1200"/><headlight ambient=".6 .6 .6" diffuse=".7 .7 .7" specular=".2 .2 .2"/><quality shadowsize="2048"/></visual><asset><texture type="skybox" builtin="flat" rgb1=".94 .96 .98" rgb2=".94 .96 .98" width="256" height="1536"/>{asset}</asset><worldbody><light pos=".15 .15 .25" diffuse=".6 .6 .6"/><light pos="-.1 -.05 .15" diffuse=".6 .6 .6"/>{fixed}<body name="rotor">{moving}</body></worldbody></mujoco>'
(r/'cad_visual_only.xml').write_text(xml);m=mujoco.MjModel.from_xml_path(str(r/'cad_visual_only.xml'));data=mujoco.MjData(m);ren=mujoco.Renderer(m,750,900);bid=m.body('rotor').id
fontpath='C:/Windows/Fonts/malgun.ttf';font=ImageFont.truetype(fontpath,26);small=ImageFont.truetype(fontpath,20)
canvas=Image.new('RGB',(1800,880),'#eff4fa')
for i,q in enumerate([0,70]):
 m.body_quat[bid]=[math.cos(math.radians(q)/2),math.sin(math.radians(q)/2),0,0];mujoco.mj_forward(m,data)
 cam=mujoco.MjvCamera();cam.lookat[:]=[-.009,.004,.001];cam.distance=.19;cam.azimuth=133;cam.elevation=-26
 ren.update_scene(data,cam);im=Image.fromarray(ren.render());canvas.paste(im,(i*900,90));im.save(r/('joint_'+str(q)+'deg.png'))
draw=ImageDraw.Draw(canvas);draw.text((36,22),'검지 뿌리 1축 · 케이블 창·고정 배선·볼트 규격 반영',font=font,fill='#1d2b3e');draw.text((36,63),'펴기 0°',font=small,fill='#33445a');draw.text((936,63),'굽히기 70°',font=small,fill='#33445a');draw.text((36,840),'배선은 고정부에 고정 · 플러그는 외곽 공간 모델 · 결합 공차·체결 강도는 실물 확인 필요',font=small,fill='#33445a')
canvas.save(r/'assembly_preview.png');ren.close();print('rendered',len(d),'CAD parts')
