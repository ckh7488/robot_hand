import FreeCAD as App,Part,json
from pathlib import Path
V=App.Vector;r=Path(__file__).resolve().parent;d=App.openDocument(str(r/'index_root_joint.FCStd'));o={x.Name:x.Shape for x in d.Objects if x.TypeId=='PartDesign::Feature'}
for start in [-11.1,-11.5,-12.0]:
 p=Part.makeBox(7.1,9.5,3.8,V(start,-13.75,6.1))
 print(start,[(n,round(p.common(o[n]).Volume,6)) for n in ['Motor_part_00','Motor_part_01','Motor_part_02','Motor_part_04']])
# Header volume sampling identifies PCB pins vs plastic body along shaft.
s=o['Motor_part_14']
for x in [-8,-7,-5,-3,0]:
 c=Part.makeBox(.1,12,5,V(x,-15,5.5));print('header slice',x,s.common(c).Volume)
