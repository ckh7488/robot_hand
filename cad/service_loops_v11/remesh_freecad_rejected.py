import FreeCAD,Part,Mesh,json
from pathlib import Path
D=Path(__file__).resolve().parent
for q in [-20,0,60]:
 for lane in [1,2,3]:
  n='Little_loop_q'+str(q)+'_wire'+str(lane);s=Part.read(str(D/'parts'/(n+'.step')));Mesh.Mesh(s.tessellate(.1)).write(str(D/'parts'/(n+'.stl')))
p=D/'reviews'/'loop_cad_export.json';j=json.loads(p.read_text(encoding='utf-8'));j['wire_display_mesh_deflection_mm']=.1;j['collision_uses_display_mesh']=False;p.write_text(json.dumps(j,indent=2),encoding='utf-8');print('Nine display meshes remeshed. STEP/control-point/collision geometry unchanged.')
