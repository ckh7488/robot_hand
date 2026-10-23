import FreeCAD as A,Part,json,time
from pathlib import Path
D=Path(__file__).resolve().parent;B=D.parent/'modular_roots_v9';W=D.parent/'wiring_access_v10';J=json.loads((D/'reviews'/'guide_exact_cases.json').read_text(encoding='utf-8'));meta=J['meta'];G={'little_spread_carrier':'Little_root_cable_saddle','little_mcp_link':'Little_MCP_link_cable_saddle'};assets={};V=A.Vector
for it in meta:
 if it['role'].startswith('arm_'):continue
 a=G.get(it['name'],it['asset'])
 if a not in assets:assets[a]=Part.read(str((D if a in G.values() else W if a=='Thumb_metacarpal' else B)/'parts'/('Thumb_metacarpal_channel_candidate.step' if a=='Thumb_metacarpal' else a+'.step')))
for a in G.values():assets[a+'_added']=Part.read(str(D/'parts'/(a+'_added.step')))
def posed(s,F):
 m=A.Matrix()
 for i in range(3):
  for j in range(4):setattr(m,'A%d%d'%(i+1,j+1),F[i][j])
 r=s.copy();r.Placement=A.Placement(m).multiply(r.Placement);return r

def bound(a,b):
 x,y=a.BoundBox,b.BoundBox;return max(0,x.XMin-y.XMax,y.XMin-x.XMax)**2+max(0,x.YMin-y.YMax,y.YMin-x.YMax)**2+max(0,x.ZMin-y.ZMax,y.ZMin-x.ZMax)**2
rows=[];calls=0;start=time.monotonic()
for case in J['cases']:
 lookup={it['name']:F for it,F in zip(meta,case['frames'])};obs=[(it['name'],posed(assets[G.get(it['name'],it['asset'])],F)) for it,F in zip(meta,case['frames']) if not it['role'].startswith('arm_')]
 for parent,a in G.items():
  s=posed(assets[a+'_added'],lookup[parent]);minimum=4.;pair=None;bad=[]
  for name,o in obs:
   if name==parent or bound(s,o)>16:continue
   d=s.distToShape(o)[0];calls+=1;vol=s.common(o).Volume if d<1e-7 else 0
   if d<minimum:minimum=d;pair=[parent,name]
   if d<.3-1e-6 or vol>1e-6:bad.append({'pair':[parent,name],'gap_mm':d,'overlap_mm3':vol})
  rows.append({'case':case['name'],'parent':parent,'minimum_added_gap_capped_4mm':minimum,'pair':pair,'failures':bad})
 print(case['name'],'complete',flush=True)
out={'scope':'Exact BRep added-material checks at6 stored poses and8 little MCP/spread endpoint combinations. Includes new guides, fasteners and V10 thumb channel. Not a continuous full joint-space certificate; complements1458mesh samples.','cases':len(J['cases']),'part_cases':len(rows),'minimum_gap_mm':min(r['minimum_added_gap_capped_4mm'] for r in rows),'failed_part_cases':sum(bool(r['failures']) for r in rows),'uncached_BRep_distance_calls':calls,'elapsed_s':time.monotonic()-start,'rows':rows,'manufacturing_release':False};(D/'reviews'/'guide_exact.json').write_text(json.dumps(out,indent=2),encoding='utf-8');print({k:v for k,v in out.items() if k!='rows'},flush=True)
