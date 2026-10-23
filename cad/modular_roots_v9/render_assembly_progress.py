"""Render the exact new CAD meshes. Not an idealized box model or image generation."""
from pathlib import Path
import json,math,numpy as np,mujoco
from scipy.spatial.transform import Rotation
from PIL import Image,ImageDraw,ImageFont
D=Path(__file__).resolve().parent;S=json.loads((D/'scene.json').read_text(encoding='utf-8'));C=json.loads((D/'cad_manifest.json').read_text(encoding='utf-8'));F=ImageFont.truetype('C:/Windows/Fonts/malgun.ttf',30);f=ImageFont.truetype('C:/Windows/Fonts/malgun.ttf',23)
def model(pose,mode='hand'):
 items=S[pose];assets=[];geoms=[];pts=[]
 used=set(it['asset'] for it in items)
 for n in used:assets.append(f'<mesh name="{n}" file="{(D/"meshes"/(n+".stl")).as_posix()}" maxhullvert="64"/>')
 for it in items:
  if mode=='feet' and it['name'] not in ['Palm_back_chassis','Thumb_internal_cradle','Palm_thumb_mount_screws','Palm_thumb_mount_nuts'] and '_root_cartridge' not in it['name'] and '_root_mount_' not in it['name']:continue
  if '_root_cartridge' in it['name'] or it['name']=='Thumb_internal_cradle':it=it|{'color':[.18,.58,.61]}
  if mode!='whole' and it['role'].startswith('arm_'):continue
  if mode=='root' and not (it['name'].startswith('index_root_') or it['name']=='index_spread_carrier' or (it['name'].startswith('index_mcp_') and not it['name'].endswith('_link'))):continue
  if mode=='inside' and it['name']=='Palm_front_cover':continue
  p=np.array(it['pos_mm'])/1000;r=np.array(it['R']);q=Rotation.from_matrix(r).as_quat();qq=[q[3],*q[:3]];v=lambda a:' '.join(str(float(x)) for x in a)
  geoms.append(f'<geom name="{it["name"]}" type="mesh" mesh="{it["asset"]}" pos="{v(p)}" quat="{v(qq)}" rgba="{v([*it["color"],1])}" contype="0" conaffinity="0"/>')
 xml='<mujoco><visual><global offwidth="1500" offheight="1500"/><quality shadowsize="4096"/><headlight ambient=".6 .6 .6" diffuse=".5 .5 .5" specular=".12 .12 .12"/></visual><asset><texture type="skybox" builtin="flat" rgb1=".94 .96 .98" rgb2=".94 .96 .98" width="256" height="1536"/>'+''.join(assets)+'</asset><worldbody><light pos=".4 .3 .9" diffuse=".6 .6 .6"/>'+''.join(geoms)+'</worldbody></mujoco>'
 m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m);mujoco.mj_forward(m,d)
 # Bounds from actual mesh vertex positions, not bounding capsules.
 for g in range(m.ngeom):
  mesh=m.geom_dataid[g];start=m.mesh_vertadr[mesh];num=m.mesh_vertnum[mesh]
  rr=d.geom_xmat[g].reshape(3,3);ps=m.mesh_vert[start:start+num]@rr.T+d.geom_xpos[g];pts.extend(ps)
 pts=np.array(pts);return m,d,(pts.min(0),pts.max(0))
def sheet(path,pose,mode,title,subtitle,views):
 m,d,(lo,hi)=model(pose,mode);ren=mujoco.Renderer(m,1100,800);look=(lo+hi)/2;dist=max(hi-lo)*2.1
 w=len(views)*800;canvas=Image.new('RGB',(w,1240),'#eff4fa');dr=ImageDraw.Draw(canvas);dr.text((28,14),title,font=F,fill='#203449');dr.text((28,58),subtitle,font=f,fill='#435870')
 for i,(label,az,el) in enumerate(views):
  cam=mujoco.MjvCamera();cam.lookat[:]=look;cam.distance=dist;cam.azimuth=az;cam.elevation=el;ren.update_scene(d,cam)
  for eye in ren.scene.camera:
   fw=np.array(eye.forward);up=np.array([0.,1.,0.]);up-=fw*np.dot(up,fw);eye.up[:]=up/np.linalg.norm(up)
  im=Image.fromarray(ren.render());ren.enable_segmentation_rendering();seg=ren.render();ren.disable_segmentation_rendering();yy,xx=np.where(seg[:,:,0]>=0)
  if len(xx):im=im.crop((max(0,int(xx.min())-20),max(0,int(yy.min())-20),min(im.width,int(xx.max())+21),min(im.height,int(yy.max())+21)))
  scale=min(760/im.width,1040/im.height);im=im.resize((round(im.width*scale),round(im.height*scale)),Image.Resampling.LANCZOS);tile=Image.new('RGB',(800,1070),'#eff4fa');tile.paste(im,((800-im.width)//2,(1070-im.height)//2));canvas.paste(tile,(i*800,150));ImageDraw.Draw(canvas).text((i*800+28,105),label,font=f,fill='#203449')
 canvas.save(D/path);ren.close();print(path,flush=True)

sheet('reviews/all_cartridges.png','Open','inside','V9 · 다섯 손가락의 분리형 뿌리 구조','관절축·손가락 비율 유지 / 고정대만 손바닥 밖에서 조립한 뒤 장착하는 후보',[('손바닥 내부',90,-90),('사선',125,-35),('측면',180,0)])
sheet('reviews/cartridge_feet.png','Open','feet','뿌리 모듈 고정대와 손바닥 체결','청록색 고정대 / 손바닥 밑면 M3×12 두 개씩 · 조립 경로와 실제 CAD 간섭 검사 중',[('고정 발 배치',90,-90),('밑면 체결',270,75),('사선',125,-35)])
sheet('reviews/whole_hand_forearm.png','Open','whole','손·손목·전완 전체 CAD 배치','팔꿈치 → 손목 굽힘축280mm 유지 / 팔 모터는 외곽 모델 · 플랜지 체결과 하중 여유 검토 필요',[('정면',90,-90),('사선',125,-30),('측면',180,0)])
