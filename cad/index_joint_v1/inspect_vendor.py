import FreeCAD as App, Part,json
from pathlib import Path
r=Path(r'E:/hand_recog/mujoco-direct')
out={}
for n,p in [('motor',r/'sources/XL330.stp'),('hinge',r/'cad/index_joint_v1/sources/FPX330-H101.step'),('fixed',r/'cad/index_joint_v1/sources/FPX330-S101.step')]:
 s=Part.read(str(p));b=s.BoundBox
 out[n]={'valid':s.isValid(),'bbox':[b.XMin,b.YMin,b.ZMin,b.XMax,b.YMax,b.ZMax],'solids':[]}
 for i,t in enumerate(s.Solids):
  b=t.BoundBox;out[n]['solids'].append({'i':i,'volume':t.Volume,'bbox':[b.XMin,b.YMin,b.ZMin,b.XMax,b.YMax,b.ZMax],'center':list(t.CenterOfMass)})
(r/'cad/index_joint_v1/inspection.json').write_text(json.dumps(out,indent=2))
print(json.dumps(out))
