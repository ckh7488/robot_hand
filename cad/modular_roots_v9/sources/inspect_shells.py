import FreeCAD as A
import Part,json,math,sys
from pathlib import Path
sys.stdout.reconfigure(encoding='utf-8');D=Path(__file__).resolve().parent
for name in ['AK70','AK60','AK45']:
 s=Part.read(str(D/(name+'.step')))
 print(name,'shapes',s.ShapeType,'shells',len(s.Shells),'faces',len(s.Faces),'closed',s.isClosed(),flush=True)
 for i,sh in enumerate(s.Shells):
  try:
   ss=sh.copy();ss.sewShape();solid=Part.makeSolid(ss);solid.fix(.01,.00001,.01)
   print('shell',i,'faces',len(sh.Faces),'closed',sh.isClosed(),'solid',solid.isValid(),len(solid.Solids),solid.Volume,flush=True)
  except Exception as e:print('repair',str(e),flush=True)
 # Unique external axial thread centres from the actual face geometry.
 rows={}
 for f in s.Faces:
  if not isinstance(f.Surface,Part.Cylinder):continue
  c=f.Surface;axis=list(c.Axis);ax=max(range(3),key=lambda i:abs(axis[i]))
  if abs(axis[ax])<.99 or not .95<c.Radius<1.7:continue
  v=list(c.Center);rad=math.hypot(v[0],v[2]) if name!='AK60' else math.hypot(v[0]+65.86838355104845,v[1]-27.163855135051268)
  b=f.BoundBox;key=(round(c.Radius,3),ax,tuple(round(q,3) for q in v))
  rows[key]=[round(rad,3),[round(getattr(b,k),3) for k in ['XMin','XMax','YMin','YMax','ZMin','ZMax']]]
 print('thread faces',rows,flush=True)
