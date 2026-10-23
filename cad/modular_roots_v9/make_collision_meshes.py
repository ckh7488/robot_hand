"""Fine CAD tessellation for independent FCL triangle-distance checks. Units mm."""
import FreeCAD as A
import Part,numpy as np,json,hashlib,sys
from pathlib import Path
D=Path(__file__).resolve().parent;out={};payload={}
for p in sorted((D/'parts').glob('*.step')):
 s=Part.read(str(p));v,t=s.tessellate(.05)
 vs=np.array([list(x) for x in v],dtype=np.float64);tt=np.array(t,dtype=np.int32)
 payload[p.stem+'_vertices']=vs;payload[p.stem+'_faces']=tt
 out[p.stem]={'step_sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'vertices':len(vs),'triangles':len(tt),'deflection_mm':.05}
np.savez_compressed(D/'collision_meshes.npz',**payload)
(D/'collision_mesh_manifest.json').write_text(json.dumps(out,indent=2))
print('Fine collision meshes',len(out),'triangles',sum(x['triangles'] for x in out.values()),flush=True)
