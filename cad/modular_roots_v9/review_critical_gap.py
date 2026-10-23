import sys,json
from pathlib import Path
import FreeCAD as A,Part
sys.stdout.reconfigure(encoding='utf-8');D=Path(__file__).resolve().parent;B=D.parent/'modular_roots_v8';S=json.loads((D/'scene.json').read_text(encoding='utf-8'))['Backbend']
def placed(s,it):
 m=A.Matrix()
 for i in range(3):
  for j in range(3):setattr(m,'A%d%d'%(i+1,j+1),it['R'][i][j])
  setattr(m,'A%d4'%(i+1),it['pos_mm'][i])
 o=s.copy();o.Placement=A.Placement(m).multiply(o.Placement);return o
it=next(it for it in S if it['name']=='index_mcp_link');link=placed(Part.read(str(D/'parts'/(it['asset']+'.step'))),it);it=next(it for it in S if it['name']=='Middle_root_cartridge');rows=[]
for rev,p in [(8,B),(9,D)]:
 s=placed(Part.read(str(p/'parts/Root_cartridge.step')),it);gap,witness,meta=link.distToShape(s);row={'revision':rev,'distance_mm':gap,'nearest_points_mm':[[list(a),list(b)] for a,b in witness]};rows.append(row);print(row,flush=True)
(D/'reviews/critical_gap.json').write_text(json.dumps({'pose':'Backbend','pair':['index_mcp_link','Middle_root_cartridge'],'method':'Exact BRep distToShape','rows':rows},indent=2))
