import FreeCAD as App,Part,json
from pathlib import Path
r=Path(r'E:/hand_recog/mujoco-direct');out={}
for n,p in [('motor',r/'sources/XL330.stp'),('hinge',r/'cad/index_joint_v1/sources/FPX330-H101.step'),('fixed',r/'cad/index_joint_v1/sources/FPX330-S101.step')]:
 s=Part.read(str(p));rows=[]
 for i,f in enumerate(s.Faces):
  sf=f.Surface
  if isinstance(sf,Part.Cylinder):
   rows.append({'face':i,'r':sf.Radius,'axis':list(sf.Axis),'center':list(sf.Center),'area':f.Area,'bbox':[f.BoundBox.XMin,f.BoundBox.YMin,f.BoundBox.ZMin,f.BoundBox.XMax,f.BoundBox.YMax,f.BoundBox.ZMax]})
 out[n]=rows
(r/'cad/index_joint_v1/cylinders.json').write_text(json.dumps(out,indent=2))
print(json.dumps({k:[a for a in v if .7<a['r']<1.1 or a['r'] in [3,4,8]] for k,v in out.items()}))
