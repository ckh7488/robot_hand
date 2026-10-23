import FreeCAD as A,Part,Mesh,json
from pathlib import Path
D=Path(__file__).resolve().parent;C=json.loads((D/'design.json').read_text(encoding='utf-8'));f=C['hand']['fingers'][0];x,y,z=f['root'];s=Part.read(str(D/'parts/Palm_back_chassis.step')).common(Part.makeBox(40,65,34,A.Vector(x+5,y-43,-72)));v,t=s.tessellate(.12);Mesh.Mesh([[(v[i].x*.001,v[i].y*.001,v[i].z*.001) for i in q] for q in t]).write(str(D/'meshes/Display_palm_cutaway.stl'));print('Display section only',len(t),'triangles')
