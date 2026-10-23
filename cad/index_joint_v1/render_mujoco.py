import json,numpy as np,trimesh,mujoco
from PIL import Image
from pathlib import Path
r=Path(r'E:/hand_recog/mujoco-direct/cad/index_joint_v1');d=json.loads((r/'proto_meshes.json').read_text());(r/'meshes').mkdir(exist_ok=True)
colors={'body':'.24 .28 .34 1','horn':'.95 .65 .19 1','idler':'.95 .65 .19 1','hinge':'.15 .48 .85 1','fixed':'.25 .65 .45 1'}
for n,a in d.items():
 t=trimesh.Trimesh(vertices=np.array(a['vertices'])*.001,faces=a['triangles'],process=False);t.export(r/'meshes'/(n+'.stl'))
assets=''.join(f'<mesh name="{n}" file="meshes/{n}.stl" maxhullvert="32"/>' for n in d)
geoms=''.join(f'<geom type="mesh" mesh="{n}" rgba="{colors[n]}" contype="0" conaffinity="0"/>' for n in d)
xml=f'<mujoco><visual><global offwidth="1500" offheight="900"/><quality shadowsize="2048"/></visual><asset>{assets}</asset><worldbody><light pos=".1 .1 .2"/><light pos="-.1 -.05 .1"/>{geoms}</worldbody></mujoco>'
(r/'visual_only.xml').write_text(xml);m=mujoco.MjModel.from_xml_path(str(r/'visual_only.xml'));data=mujoco.MjData(m);mujoco.mj_forward(m,data)
ren=mujoco.Renderer(m,700,1000)
for name,az,el in [('assembly_probe',135,-22),('assembly_probe_back',-45,-22)]:
 cam=mujoco.MjvCamera();cam.lookat[:]=[0,-.004,.001];cam.distance=.12;cam.azimuth=az;cam.elevation=el
 ren.update_scene(data,cam);Image.fromarray(ren.render()).save(r/(name+'.png'))
ren.close();print('rendered')
