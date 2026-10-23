import FreeCAD as App,Part,json
from pathlib import Path
r=Path(__file__).resolve().parent;d=App.openDocument(str(r/'index_root_joint.FCStd'))
for name,fn,axis in [('Base_M2_nut_1','FPX330_S101',App.Vector(0,0,1)),('Tab_M2_nut_1','FPX330_H101',App.Vector(0,1,0))]:
 s=d.getObject(name).Shape;center=s.BoundBox.Center
 print(name,[(a, (lambda t:t.common(d.getObject(fn).Shape).Volume)((lambda t:(t.rotate(center,axis,a),t)[1])(s.copy()))) for a in [0,15,30,45]])
