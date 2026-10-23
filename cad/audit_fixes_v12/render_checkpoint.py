"""Render actual CAD meshes and geometric wire candidates; no generative imagery."""
import sys,json,copy,numpy as np,mujoco
from pathlib import Path
from scipy.spatial.transform import Rotation
from PIL import Image,ImageDraw,ImageFont
D=Path(__file__).resolve().parent;Ldir=D.parent/'service_loops_v11';B=D.parent/'modular_roots_v9';W=D.parent/'wiring_access_v10';sys.path.insert(0,str(B));from kinematics_numpy import C,META,forward
F=ImageFont.truetype('C:/Windows/Fonts/malgun.ttf',32);f=ImageFont.truetype('C:/Windows/Fonts/malgun.ttf',23);tiny=ImageFont.truetype('C:/Windows/Fonts/malgun.ttf',21)
J=json.loads((Ldir/'reviews'/'service_loop_curves.json').read_text(encoding='utf-8'));L=J['chosen']['length_mm'];geom=json.loads((Ldir/'reviews'/'loop_geometry_verified.json').read_text(encoding='utf-8'));selected=json.loads((Ldir/'selected_routes.json').read_text(encoding='utf-8'))['selected'];ids={it['name']:i for i,it in enumerate(META)};V=lambda a:' '.join(map(str,a))
canvas=Image.new('RGB',(2100,1100),'#eff4fa');dr=ImageDraw.Draw(canvas);dr.text((30,18),'CP001 · V12 CAD 조립 후보와 배선 공간',font=F,fill='#1d354d');dr.text((30,68),'실제 CAD 형상 · 주황 = 추가 받침 / 초록 = 연결 루프 / 청록 = 커넥터 출선',font=f,fill='#455c71')
for k,(q,mode,title) in enumerate([(0,'hand','손 전체 · 엄지 공구 통로 수정 반영'),(0,'detail','편 상태 · 출선과 여유 루프'),(60,'detail','60° 굽힘 · 같은 길이의 루프 후보')]):
 pose=copy.deepcopy(C['poses']['Open']);pose['little'][1]=q;M=forward(pose);assets={};geoms=[]
 def mesh(name,path,scale):
  if name not in assets:assets[name]='<mesh name="'+name+'" file="'+str(path)+'" scale="'+str(scale)+' '+str(scale)+' '+str(scale)+'"/>'
 def obj(name,asset,path,scale,m,col):
  mesh(asset,path,scale);qu=Rotation.from_matrix(m[:3,:3]).as_quat();geoms.append('<geom name="'+name+'" type="mesh" mesh="'+asset+'" pos="'+V(m[:3,3]/1000)+'" quat="'+V([qu[3],*qu[:3]])+'" rgba="'+V([*col,1])+'" contype="0" conaffinity="0"/>')
 for it,m in zip(META,M):
  if it['role'].startswith('arm_'):continue
  if mode=='detail' and (not it['name'].startswith('little_') or it['name'].startswith('little_dip_') or it['name']=='little_pip_link'):continue
  path=D/'parts'/'Thumb_metacarpal_tool_access.stl' if it['asset']=='Thumb_metacarpal' else B/'meshes'/(it['asset']+'.stl');scale=.001 if it['asset']=='Thumb_metacarpal' else 1;obj(it['name'],it['asset'],path,scale,m,it['color'])
 for parent,asset in [('little_spread_carrier','Little_root_cable_saddle_added'),('little_mcp_link','Little_MCP_link_cable_saddle_added')]:obj(parent+'_guide',asset,Ldir/'parts'/(asset+'.stl'),.001,M[ids[parent]],[.94,.47,.12])
 for route in selected:
  if route['motor'] not in ['little_mcp_motor','little_pip_motor']:continue
  for lab in ['plug','wire1','wire2','wire3']:
   asset=route['template']+'_'+lab;obj(route['motor']+'_'+lab,asset,W/'parts'/(asset+'.stl'),.001,M[ids[route['motor']]],[.08,.60,.70])
 for lane in [1,2,3]:
  asset='Little_loop_q'+str(q)+'_wire'+str(lane);obj(asset,asset,Ldir/'parts'/(asset+'.stl'),.001,M[ids['little_mcp_motor']],[.04,.67,.36])
 xml='<mujoco><visual><global offwidth="1200" offheight="1200"/><headlight ambient=".65 .65 .65" diffuse=".5 .5 .5"/></visual><asset><texture type="skybox" builtin="flat" rgb1=".94 .96 .98" rgb2=".94 .96 .98" width="256" height="1536"/>'+''.join(assets.values())+'</asset><worldbody><light pos="-.5 .3 -.5"/>'+''.join(geoms)+'</worldbody></mujoco>';m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m);mujoco.mj_forward(m,d);ren=mujoco.Renderer(m,850,700);cam=mujoco.MjvCamera();cam.lookat[:]=[0,.18,0] if mode=='hand' else [-.114,.178,-.035];cam.distance=.52 if mode=='hand' else .27;cam.azimuth=125 if mode=='hand' else 185;cam.elevation=-35 if mode=='hand' else -14;ren.update_scene(d,cam)
 for eye in ren.scene.camera:
  fw=np.array(eye.forward);up=np.array([0.,1.,0.]);up-=fw*np.dot(up,fw);eye.up[:]=up/np.linalg.norm(up)
 im=Image.fromarray(ren.render());canvas.paste(im,(700*k,180));ren.close();dr.text((700*k+25,126),title,font=f,fill='#203449')
texts=['출선 방향을 손등 쪽으로 통일','고정 받침 2곳을 기존 출력 부품과 일체화','각 가닥 '+str(L)+'mm · 관절 각도에 맞춰 굽힘']
for k,t in enumerate(texts):dr.text((700*k+25,1030),t,font=tiny,fill='#3f576c')
dr.text((25,1071),'기하학 배선 후보: 전선 탄성·처짐·케이블타이 체결·반복 수명 및 실제 서보 속도는 미검증',font=tiny,fill='#735335');canvas.save(D/'reviews'/'checkpoint_preview.png');print('rendered',flush=True)
