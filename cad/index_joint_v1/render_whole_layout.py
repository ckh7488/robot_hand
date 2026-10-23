"""Render existing full CAD packaging layout, not a completed manufactured hand."""
import sys,json,math,itertools,numpy as np,mujoco
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
from scipy.spatial.transform import Rotation
r=Path(__file__).resolve().parent;sys.path.insert(0,str(r.parent.parent));import model as robot
from control import pose
root=r.parent.parent;layout=json.loads((root/'models/cad_layout.json').read_text());tf,*_=robot.fk(pose('Flat'));Rp,pp=tf['palm'];Rp=np.array(Rp);pp=np.array(pp)
assets='<mesh name="xl330" file="'+(root/'models/motor_visual.stl').as_posix()+'" maxhullvert="32"/>'
geoms=[];allpts=[];handpts=[];body_hand=[]
for g in layout['items']:
 name=g['name'];p=np.array(g['world_pos']);rot=np.array(g['world_R']);ishand=g['body']=='palm' or g['body'].split('_')[0] in ['index','middle','ring','little','thumb']
 if g['role']=='XL330-M077':
  p=np.array(g['motor_anchor_world']);typ='mesh';extra='mesh="xl330"';color=[.29,.32,.37,1];lo=np.array([-.029,-.0245,-.0101]);hi=np.array([0,.0095,.0101])
 else:
  typ=g['kind'];sz=np.array(g['size']);color=g['color'];extra='size="'+' '.join(map(str,sz/2 if typ=='box' else sz))+'"';lo=-(sz/2 if typ=='box' else np.array([sz[0],sz[0],sz[1]]));hi=-lo
 if name=='motor_index_mcp' or name.startswith('index_yoke_1_'):color=[.95,.55,.16,1]
 elif 'yoke' in name:color=[.36,.5,.68,1]
 if 'pad' in name and name!='motor_index_mcp':color=[.84,.87,.9,1]
 if name=='palm_plate':color=[.65,.7,.77,1]
 hp=Rp.T@(p-pp);hR=Rp.T@rot;xyzw=Rotation.from_matrix(hR).as_quat();q=[xyzw[3],*xyzw[:3]]
 textv=lambda vals:' '.join(str(float(v)) for v in vals)
 geoms.append(f'<geom name="{name}" type="{typ}" {extra} pos="{textv(hp)}" quat="{textv(q)}" rgba="{textv(color)}" contype="0" conaffinity="0" group="2"/>')
 points=np.array([hp+hR@np.array(a) for a in itertools.product(*zip(lo,hi))]);allpts.extend(points)
 if ishand:handpts.extend(points)
xml='<mujoco model="whole_CAD_packaging_visual"><visual><global offwidth="1800" offheight="1500"/><headlight ambient=".6 .6 .6" diffuse=".6 .6 .6" specular=".1 .1 .1"/></visual><asset><texture type="skybox" builtin="flat" rgb1=".94 .96 .98" rgb2=".94 .96 .98" width="256" height="1536"/>'+assets+'</asset><worldbody><light pos=".4 .3 .8" diffuse=".5 .5 .5"/>'+''.join(geoms)+'</worldbody></mujoco>'
(r/'whole_layout_visual.xml').write_text(xml);m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m);mujoco.mj_forward(m,d);ren=mujoco.Renderer(m,1050,800)
allpts=np.array(allpts);handpts=np.array(handpts);alo,ahi=allpts.min(0),allpts.max(0);hlo,hhi=handpts.min(0),handpts.max(0)
# Actual CAD placement bounds, not inferred anatomical scale.
(r/'whole_layout_dimensions.json').write_text(json.dumps({'status':'Existing full-hand packaging concept; new one-axis fixture is separate, not integrated.','hand_bbox_mm':((hhi-hlo)*1000).tolist(),'palm_plate_mm':[196,70,6],'full_bbox_mm':((ahi-alo)*1000).tolist()},indent=2))
font=ImageFont.truetype('C:/Windows/Fonts/malgun.ttf',30);small=ImageFont.truetype('C:/Windows/Fonts/malgun.ttf',23)
def sheet(name,bounds,views,title,subtitle):
 lo,hi=bounds;look=(lo+hi)/2;extent=max(hi-lo);distance=extent*2.1
 canvas=Image.new('RGB',(len(views)*800,1220),'#eff4fa');dr=ImageDraw.Draw(canvas);dr.text((30,16),title,font=font,fill='#1d2b3e');dr.text((30,58),subtitle,font=small,fill='#40526c')
 for i,(label,az,el) in enumerate(views):
  cam=mujoco.MjvCamera();cam.lookat[:]=look;cam.distance=distance;cam.azimuth=az;cam.elevation=el
  ren.update_scene(d,cam)
  for eye in ren.scene.camera:
   fw=np.array(eye.forward);up=np.array([0.,1.,0.]);up-=fw*np.dot(up,fw);eye.up[:]=up/np.linalg.norm(up)
  im=Image.fromarray(ren.render());ren.enable_segmentation_rendering();seg=ren.render();ren.disable_segmentation_rendering();mask=seg[:,:,0]>=0;yy,xx=np.where(mask)
  if len(xx):
   im=im.crop((max(0,int(xx.min())-22),max(0,int(yy.min())-22),min(im.width,int(xx.max())+23),min(im.height,int(yy.max())+23)))
   scale=min(770/im.width,1010/im.height);im=im.resize((round(im.width*scale),round(im.height*scale)),Image.Resampling.LANCZOS)
  tile=Image.new('RGB',(800,1050),'#eff4fa');tile.paste(im,((800-im.width)//2,(1050-im.height)//2));canvas.paste(tile,(800*i,140));ImageDraw.Draw(canvas).text((800*i+30,95),label,font=small,fill='#24334c')
 canvas.save(r/name)
sheet('whole_assembly_overview.png',(alo,ahi),[('정면 / 손바닥 쪽',90,-90),('사선',125,-30),('측면',180,0)],'손·손목·전완 전체 배치','기존 배치 초안 · 팔 모터는 공칭 외곽 · 주황 관절은 상세 검토 중 · 보기별 확대비 다름')
# Clip distant mounting stand from the close-up view using a separate model hide group.
for g in layout['items']:
 ishand=g['body']=='palm' or g['body'].split('_')[0] in ['index','middle','ring','little','thumb']
 if not ishand:m.geom(g['name']).rgba[3]=0
sheet('whole_hand_closeup.png',(hlo,hhi),[('손바닥 쪽',90,-90),('손등 쪽',270,90),('손 사선',120,-35)],'손 전체 모양 · 현재 직결 20모터 배치','손바닥판 폭 196mm / 손 전체 폭 약 290mm · 전체 조립 승인 전의 외형 검토용 배치')
ren.close();print('Rendered',len(layout['items']),'CAD placement items; hand bounds mm',((hhi-hlo)*1000).round(1).tolist())
