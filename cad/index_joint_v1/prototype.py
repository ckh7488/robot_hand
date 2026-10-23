import FreeCAD as App,Part,json,math
from pathlib import Path
r=Path(r'E:/hand_recog/mujoco-direct');out=r/'cad/index_joint_v1'
def place(s,rot=App.Rotation(),v=App.Vector()):
 t=s.copy();t.Placement=App.Placement(v,rot).multiply(s.Placement);return t
m=Part.read(str(r/'sources/XL330.stp'))
h=Part.read(str(out/'sources/FPX330-H101.step'));h=place(h,v=App.Vector(0,14,-8))
f=Part.read(str(out/'sources/FPX330-S101.step'));f=place(f,App.Rotation(App.Vector(0,0,1),-90),App.Vector(10,-7.5,-8))
parts={'body':Part.makeCompound([s for i,s in enumerate(m.Solids) if i not in [3,9,10]]),'horn':m.Solids[3],'idler':m.Solids[10],'hinge':h,'fixed':f}
# transform source motor coordinates -> hand axes X shaft, Y finger, Z palm
R=App.Rotation(App.Vector(0,1,0),90);T=App.Vector(8,0,0)
parts={n:place(s,R,T) for n,s in parts.items()}
meshes={}
for n,s in parts.items():
 vs,ts=s.tessellate(.15);meshes[n]={'vertices':[list(v) for v in vs],'triangles':ts}
(out/'proto_meshes.json').write_text(json.dumps(meshes))
print('parts',[(n,s.isValid()) for n,s in parts.items()])
for n,s in parts.items():
 if n!='hinge':print('hinge',n,'common',h.common(m).Volume if False else parts['hinge'].common(s).Volume,'distance',parts['hinge'].distToShape(s)[0])
for q in [-20,0,30,60,90,100]:
 v=place(parts['hinge'],App.Rotation(App.Vector(1,0,0),q))
 print(q,[(n,round(v.common(s).Volume,6),round(v.distToShape(s)[0],5)) for n,s in parts.items() if n in ['body','fixed']])
