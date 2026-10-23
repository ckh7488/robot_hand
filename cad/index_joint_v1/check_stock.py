import FreeCAD as App,Part,json,math
from pathlib import Path
r=Path(r'E:/hand_recog/mujoco-direct');out=r/'cad/index_joint_v1'
def place(s,rot=App.Rotation(),v=App.Vector()):
 t=s.copy();t.Placement=App.Placement(v,rot).multiply(s.Placement);return t
m=Part.read(str(r/'sources/XL330.stp'))
h=Part.read(str(out/'sources/FPX330-H101.step'));h=place(h,v=App.Vector(0,14,-8))
f=Part.read(str(out/'sources/FPX330-S101.step'));f=place(f,App.Rotation(App.Vector(0,0,1),90),App.Vector(-10,-7.5,-8))
parts={'body':Part.makeCompound([s for i,s in enumerate(m.Solids) if i not in [3,9,10]]),'horn':m.Solids[3],'idler':m.Solids[10],'hinge':h,'fixed':f}
# transform source motor coordinates -> hand axes X shaft, Y finger, Z palm
R=App.Rotation(App.Vector(0,1,0),90);T=App.Vector(8,0,0)
parts={n:place(s,R,T) for n,s in parts.items()}

import time
log=open(str(out/'brep_check.log'),'w',buffering=1)
for q in [-20,0,30,60,90,100]:
 v=place(parts['hinge'],App.Rotation(App.Vector(1,0,0),q))
 for n in ['body','fixed']:
  t=time.time();dist=v.distToShape(parts[n])[0];log.write(json.dumps({'q':q,'pair':n,'distance_mm':dist,'seconds':time.time()-t})+'\n')
log.close()
