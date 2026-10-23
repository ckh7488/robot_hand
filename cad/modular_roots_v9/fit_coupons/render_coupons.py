from pathlib import Path
import json,trimesh,numpy as np,mujoco
from PIL import Image,ImageDraw,ImageFont
D=Path(__file__).resolve().parent
F=ImageFont.truetype('C:/Windows/Fonts/malgun.ttf',26);f=ImageFont.truetype('C:/Windows/Fonts/malgun.ttf',19)
canvas=Image.new('RGB',(1600,920),'#eff4fa');draw=ImageDraw.Draw(canvas);draw.text((25,16),'전체 출력 전 · 너트와 맞춤턱 공차 시험 조각',font=F,fill='#203449')
for i,(n,label,sub) in enumerate([('M2_nut_fit','M2 너트 포켓','왼쪽부터 AF4.2 /4.3 /4.4mm'),('M3_nut_fit','M3 너트 포켓','왼쪽부터 AF5.8 /5.9 /6.0mm'),('Root_foot_seat_fit','고정 발 맞춤턱','한쪽 여유0.2 /0.3 /0.4mm'),('Root_foot_test_male','고정 발 시험편','실제 발17×45×5mm와 나사 간격')]):
 xml='<mujoco><visual><global offwidth="800" offheight="600"/><headlight ambient=".6 .6 .6" diffuse=".6 .6 .6"/></visual><asset><texture type="skybox" builtin="flat" rgb1=".94 .96 .98" rgb2=".94 .96 .98" width="256" height="1536"/><mesh name="c" file="'+(D/(n+'_fine.stl')).as_posix()+'" scale=".001 .001 .001"/></asset><worldbody><geom type="mesh" mesh="c" rgba=".22 .58 .64 1"/></worldbody></mujoco>'
 m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m);mujoco.mj_forward(m,d);mesh=trimesh.load_mesh(D/(n+'.stl'));lo,hi=mesh.bounds*.001;cam=mujoco.MjvCamera();cam.lookat[:]=(lo+hi)/2;cam.distance=max(hi-lo)*2.0;cam.azimuth=235;cam.elevation=-55
 ren=mujoco.Renderer(m,500,750);ren.update_scene(d,cam);im=Image.fromarray(ren.render());ren.close();im.thumbnail((750,330))
 x=(i%2)*800;y=70+(i//2)*420;canvas.paste(im,(x+(800-im.width)//2,y+75));dr=ImageDraw.Draw(canvas);dr.text((x+28,y),label,font=F,fill='#203449');dr.text((x+28,y+39),sub,font=f,fill='#435870')
canvas.save(D/'fit_coupons.png')
