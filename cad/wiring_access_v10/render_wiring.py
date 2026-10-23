"""Render actual V9 CAD and measured routing templates, without modifying audit CAD."""
import os,sys,json,numpy as np,mujoco
from pathlib import Path
from scipy.spatial.transform import Rotation
from PIL import Image,ImageDraw,ImageFont
D=Path(__file__).resolve().parent;B=D.parent/'modular_roots_v9';S=json.loads((B/'scene.json').read_text(encoding='utf-8'))['Open'];motor=next(it for it in S if it['name']=='index_pip_motor');F=ImageFont.truetype('C:/Windows/Fonts/malgun.ttf',29);f=ImageFont.truetype('C:/Windows/Fonts/malgun.ttf',21)
canvas=Image.new('RGB',(1800,1070),'#eff4fa');draw=ImageDraw.Draw(canvas);draw.text((25,16),'전선도 부품처럼 넣어서 검사 · 검지–중지 사이',font=F,fill='#21364b');draw.text((25,60),'실제 V9 CAD + 공칭 전선 OD1.9mm / 반경 여유0.3mm · 커넥터 실물 체결·연속 하네스 검증은 별도',font=f,fill='#475e75')
for k,(template,title,color) in enumerate([('negativeZ_back','기존 한 관절 시험 경로: 옆 링크와 겹침',[.85,.23,.15]),('negativeZ_outboard','출선 방향 변경 후보: 손등 바깥으로 우회',[.05,.64,.48])]):
 items=[it for it in S if it['name'].startswith(('index_pip_','index_dip_','middle_pip_','middle_dip_')) or it['name'] in ['index_mcp_link','middle_mcp_link']];assets=[];geoms=[];used=set(it['asset'] for it in items);vec=lambda a:' '.join(map(str,a))
 for n in used:assets.append('<mesh name="'+n+'" file="'+str(B/'meshes'/(n+'.stl'))+'"/>')
 for it in items:
  q=Rotation.from_matrix(it['R']).as_quat();col=it['color'];col=[.75,.51,.28] if it['name']=='middle_mcp_link' else col
  geoms.append('<geom type="mesh" mesh="'+it['asset']+'" pos="'+vec(np.array(it['pos_mm'])/1000)+'" quat="'+vec([q[3],*q[:3]])+'" rgba="'+vec([*col,1])+'" contype="0" conaffinity="0"/>')
 for lane in [1,2,3]:
  n=template+'_wire'+str(lane);assets.append('<mesh name="'+n+'" file="'+str(D/'parts'/(n+'.stl'))+'" scale=".001 .001 .001"/>');q=Rotation.from_matrix(motor['R']).as_quat();geoms.append('<geom type="mesh" mesh="'+n+'" pos="'+vec(np.array(motor['pos_mm'])/1000)+'" quat="'+vec([q[3],*q[:3]])+'" rgba="'+vec([*color,1])+'" contype="0" conaffinity="0"/>')
 xml='<mujoco><visual><global offwidth="1300" offheight="1300"/><headlight ambient=".6 .6 .6" diffuse=".5 .5 .5"/></visual><asset><texture type="skybox" builtin="flat" rgb1=".94 .96 .98" rgb2=".94 .96 .98" width="256" height="1536"/>'+''.join(assets)+'</asset><worldbody><light pos=".4 .2 .8"/>'+''.join(geoms)+'</worldbody></mujoco>';m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m);mujoco.mj_forward(m,d);ren=mujoco.Renderer(m,850,900);cam=mujoco.MjvCamera();cam.lookat[:]=[.035,.280,0];cam.distance=.26;cam.azimuth=108;cam.elevation=-33;ren.update_scene(d,cam)
 for eye in ren.scene.camera:
  fw=np.array(eye.forward);up=np.array([0.,1.,0.]);up-=fw*np.dot(up,fw);eye.up[:]=up/np.linalg.norm(up)
 im=Image.fromarray(ren.render());canvas.paste(im,(k*900,165));ren.close();draw.text((k*900+25,118),title,font=f,fill='#21364b')
canvas.save(D/'reviews'/'wiring_comparison.png');print('Actual CAD wiring comparison rendered')
