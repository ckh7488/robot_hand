import sys,json,copy,numpy as np,mujoco
from pathlib import Path
from scipy.spatial.transform import Rotation
from PIL import Image,ImageDraw,ImageFont
D=Path(__file__).resolve().parent;B=D.parent/'modular_roots_v9';sys.path.insert(0,str(B));from kinematics_numpy import C,META,forward
p=copy.deepcopy(C['poses']['Open'])
for f in ['index','middle','ring','little']:p[f]=[0,89.1,99,79.2]
idx=next(i for i,x in enumerate(META) if x['name']=='little_dip_tip');m=forward(p)[idx]
font='C:/Windows/Fonts/malgun.ttf';F=ImageFont.truetype(font,30);f=ImageFont.truetype(font,22);sm=ImageFont.truetype(font,20)
canvas=Image.new('RGB',(1600,920),'#eff4fa');d=ImageDraw.Draw(canvas);d.text((28,18),'CP002 · 덮개 나사 접촉 수정 후보',font=F,fill='#1d354d')
vec=lambda a:' '.join(str(float(x)) for x in a)
for i in [0,1]:
 assets=[];geoms=[]
 def obj(name,file,scale,mat,col,alpha=1.):
  assets.append('<mesh name="'+name+'" file="'+str(file)+'" scale="'+vec([scale]*3)+'"/>');q=Rotation.from_matrix(mat[:3,:3]).as_quat()
  geoms.append('<geom type="mesh" mesh="'+name+'" pos="'+vec(mat[:3,3]/1000)+'" quat="'+vec([q[3],*q[:3]])+'" rgba="'+vec([*col,alpha])+'" contype="0" conaffinity="0"/>')
 obj('cover',B/'meshes/Palm_front_cover.stl' if i==0 else D/'parts/Palm_front_cover_recessed.stl',1 if i==0 else .001,np.eye(4),[.77,.81,.85])
 obj('screws',B/'meshes/M3_cover_screws.stl' if i==0 else D/'parts/M3x8_button_cover_screws.stl',1 if i==0 else .001,np.eye(4),[.83,.22,.18] if i==0 else [.11,.58,.37])
 obj('tip',B/'meshes/Tip_28.stl',1,m,[.91,.62,.27],.30)
 xml='<mujoco><visual><global offwidth="1000" offheight="1000"/><headlight ambient=".7 .7 .7" diffuse=".5 .5 .5"/></visual><asset><texture type="skybox" builtin="flat" rgb1=".94 .96 .98" rgb2=".94 .96 .98" width="256" height="1536"/>'+''.join(assets)+'</asset><worldbody><light pos="-.15 .1 .2"/>'+''.join(geoms)+'</worldbody></mujoco>'
 model=mujoco.MjModel.from_xml_string(xml);data=mujoco.MjData(model);mujoco.mj_forward(model,data);r=mujoco.Renderer(model,650,780);cam=mujoco.MjvCamera();cam.lookat[:]=[-.086,.098,.026];cam.distance=.083;cam.azimuth=105;cam.elevation=-35;r.update_scene(data,cam)
 im=Image.fromarray(r.render());r.close();canvas.paste(im,(i*800+10,110));d.text((i*800+28,75),'기존 · 나사 머리 3mm 돌출' if i==0 else '수정 · 낮은 머리 + 1.95mm 자리 가공',font=f,fill='#233b50')
d.text((28,785),'반투명 주황: 소지 끝 CAD · 회색: 실제 덮개 CAD · 빨강/초록: 나사 형상 외곽',font=sm,fill='#41586c')
d.text((28,825),'기존 축·외형 유지 | 나사 머리 0.30mm 매립 | 남는 받침 2.05mm | 너트 2.4mm 전체 물림',font=f,fill='#1e4b43')
d.text((28,870),'기하학 검토 후보 · 출력물 체결 강도 / 반복 수명 / 완성 하네스는 미검증',font=sm,fill='#735335');canvas.save(D/'reviews/cover_fix_preview.png')
