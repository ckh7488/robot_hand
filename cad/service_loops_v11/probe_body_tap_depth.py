import FreeCAD as A,Part,json
from pathlib import Path
D=Path(__file__).resolve().parent;B=D.parent/'modular_roots_v9';V=A.Vector;s=Part.read(str(B/'parts'/'M077_stator.step'));frame=Part.read(str(B/'parts'/'S101_relief.step'));rows=[]
for side in [-1,1]:
 for y in [7.5,-22.5]:
  x=-1.7 if side>0 else -27.3;direction=V(-side,0,0);head=V(x,y,-8);points=[]
  for k in range(45):
   depth=k*.2;p=head+direction*depth;points.append({'from_underhead_mm':depth,'x_mm':p.x,'motor_centerline_material':s.isInside(p,1e-6,True),'frame_centerline_material':frame.isInside(p,1e-6,True)})
  core=Part.makeCylinder(.6,8,head,direction);thread=Part.makeCylinder(.95,8,head,direction);rows.append({'side':side,'y':y,'underhead':list(head),'model_length_mm':8,'core1p2mm_overlap_motor_mm3':core.common(s).Volume,'thread1p9mm_overlap_motor_mm3':thread.common(s).Volume,'points':points})
out={'scope':'Diagnostic of modeled8mmTAPs. Centerline CAD material is not an accepted thread-depth specification; compare manufacturer drawing.','rows':rows};(D/'reviews'/'body_tap_depth_probe.json').write_text(json.dumps(out,indent=2),encoding='utf-8');print(json.dumps(rows,indent=2))
