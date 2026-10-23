"""Mutual clearance of all chosen fixed lead tails. Still not a connected flexible harness."""
import json,numpy as np,trimesh,fcl
from pathlib import Path
D=Path(__file__).resolve().parent;W=D.parent/'wiring_access_v10';B=D.parent/'modular_roots_v9';S=json.loads((B/'scene.json').read_text(encoding='utf-8'));C=json.loads((D/'selected_routes.json').read_text(encoding='utf-8'))['selected'];models={};bounds={}
for c in C:
 for label in ['plug','wire1','wire2','wire3']:
  n=c['template']+'_'+label
  if n in models:continue
  m=trimesh.load_mesh(W/'parts'/(n+'.stl'));b=fcl.BVHModel();b.beginModel(len(m.vertices),len(m.faces));b.addSubModel(m.vertices,m.faces);b.endModel();models[n]=b;lo,hi=m.bounds;bounds[n]=np.array([[x,y,z] for x in [lo[0],hi[0]] for y in [lo[1],hi[1]] for z in [lo[2],hi[2]]])
rows=[];rq=fcl.CollisionRequest(num_max_contacts=1);dq=fcl.DistanceRequest()
for pn,scene in S.items():
 lookup={it['name']:it for it in scene};objects=[]
 for c in C:
  it=lookup[c['motor']];R=np.array(it['R']);p=np.array(it['pos_mm'])
  for label in ['plug','wire1','wire2','wire3']:
   n=c['template']+'_'+label;v=bounds[n]@R.T+p;objects.append((c['motor'],label,fcl.CollisionObject(models[n],fcl.Transform(R,p)),v.min(0),v.max(0)))
 hits=[];best=1.;nearest=None
 for i,(motor,label,o,lo,hi) in enumerate(objects):
  for motor2,label2,o2,l,h in objects[i+1:]:
   if motor==motor2:continue
   gap=np.maximum(0,np.maximum(lo-h,l-hi))
   if gap@gap>1.:continue
   collision=bool(fcl.collide(o,o2,rq,fcl.CollisionResult()));dist=0. if collision else max(0.,float(fcl.distance(o,o2,dq,fcl.DistanceResult())))
   if dist<best:best=dist;nearest=[motor,label,motor2,label2]
   if collision or dist<.3:hits.append({'a':[motor,label],'b':[motor2,label2],'distance_mm':dist,'surface_collision':collision})
 rows.append({'pose':pn,'minimum_gap_capped_1mm':best,'nearest':nearest,'failures':hits,'passed':not hits});print(pn,'failures',len(hits),flush=True)
out={'scope':'Mutual fixed tails only, one connector/three wires per motor. Same-motor wire lane reserve shells intentionally ignored; original conductor OD1.9mm has0.6mm nominal lane spacing. No connected flexible cable or dynamic service loop.','poses':rows,'all6sampled_fixed_tail_pairs_passed':all(r['passed'] for r in rows),'full_harness_validated':False};(D/'reviews'/'fixed_tail_pairs.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
