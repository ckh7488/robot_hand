"""Fast actual-CAD mesh screening of fixed lead exit alternatives, not full harness validation."""
import json,time,hashlib,sys,numpy as np,trimesh,fcl
from pathlib import Path
D=Path(__file__).resolve().parent;B=D.parent/'modular_roots_v9';sys.path.insert(0,str(B))
from kinematics_numpy import C,META,forward
T=json.loads((D/'route_templates.json').read_text(encoding='utf-8'));data=np.load(B/'collision_meshes.npz');models={};meshes={};bounds={}
def add(n,v,f):
 m=trimesh.Trimesh(v,f,process=True);meshes[n]=m;b=fcl.BVHModel();b.beginModel(len(v),len(f));b.addSubModel(v,f);b.endModel();models[n]=b;lo,hi=m.bounds;bounds[n]=np.array([[x,y,z] for x in [lo[0],hi[0]] for y in [lo[1],hi[1]] for z in [lo[2],hi[2]]])
for n in sorted({it['asset'] for it in META}):add(n,data[n+'_vertices'],data[n+'_faces'])
for t in T['templates']:
 for n in [t['plug_asset'],*t['wire_assets']]:
  m=trimesh.load_mesh(D/'parts'/(n+'.stl'));add(n,m.vertices,m.faces)
ids=[i for i,it in enumerate(META) if not it['role'].startswith('arm_')];motorids=[i for i in ids if META[i]['asset']=='M077_stator'];cache={};results=[];req=fcl.DistanceRequest(enable_nearest_points=False);creq=fcl.CollisionRequest(num_max_contacts=1);tick=time.monotonic()
for pn,pose in C['poses'].items():
 transforms=forward(pose);obs=[]
 for i in ids:
  it=META[i];m=transforms[i];p=bounds[it['asset']]@m[:3,:3].T+m[:3,3];obs.append((i,fcl.CollisionObject(models[it['asset']],fcl.Transform(m[:3,:3],m[:3,3])),p.min(0),p.max(0)))
 for mid in motorids:
  motor=META[mid];mat=transforms[mid]
  for t in T['templates']:
   hits=[];best=1.;checks=0
   for label,asset in [('plug',t['plug_asset']),*[(str(i+1),n) for i,n in enumerate(t['wire_assets'])]]:
    w=fcl.CollisionObject(models[asset],fcl.Transform(mat[:3,:3],mat[:3,3]));p=bounds[asset]@mat[:3,:3].T+mat[:3,3];lo,hi=p.min(0),p.max(0)
    for i,o,l,h in obs:
     it=META[i]
     if label=='plug' and i==mid:continue
     gap=np.maximum(0,np.maximum(lo-h,l-hi))
     if gap@gap>1.0:continue
     m=transforms[i];rel=np.round(np.r_[ (mat[:3,:3].T@m[:3,:3]).ravel(),mat[:3,:3].T@(m[:3,3]-mat[:3,3])],8);rel[rel==0]=0.;key=(asset,it['asset'],rel.tobytes());r=cache.get(key)
     if r is None:
      checks+=1;hit=bool(fcl.collide(w,o,creq,fcl.CollisionResult()));dist=0. if hit else max(0.,float(fcl.distance(w,o,req,fcl.DistanceResult())))
      if not hit:
       point=(meshes[asset].vertices[0]@mat[:3,:3].T+mat[:3,3]-m[:3,3])@m[:3,:3]
       if meshes[it['asset']].is_watertight and bool(meshes[it['asset']].contains([point])[0]):hit=True;dist=0.
      r=(dist,hit);cache[key]=r
     dist,hit=r;best=min(best,dist)
     if hit or dist<.3:hits.append({'source':label,'part':it['name'],'distance_mm':dist,'surface_collision_or_containment':hit})
   results.append({'pose':pn,'motor':motor['name'],'template':t['name'],'minimum_distance_capped_1mm':best,'hits':hits,'passed_at_extra_0p3mm':not hits,'uncached_queries':checks})
  print(pn,motor['name'],'complete',flush=True)
 (D/'reviews'/'route_options.json').write_text(json.dumps({'scope':'Fixed lead templates in6stored poses against hand CAD meshes. Includes1.9mm wire OD+0.3mm radial reserve and additional0.3mm gap. No wire-wire or connected flexible harness. Connector/own-case mating intentionally unresolved and excluded.','mesh_deflection_mm':.05,'geometry_modified':False,'full_harness_validated':False,'elapsed_seconds':time.monotonic()-tick,'rows':results},indent=2),encoding='utf-8')
summary=[]
for mid in motorids:
 n=META[mid]['name'];options=[]
 for t in T['templates']:
  rows=[r for r in results if r['motor']==n and r['template']==t['name']];options.append({'template':t['name'],'passed_all6stored_poses':len(rows)==6 and all(r['passed_at_extra_0p3mm'] for r in rows),'minimum_capped_gap_mm':min(r['minimum_distance_capped_1mm'] for r in rows),'failed_poses':[r['pose'] for r in rows if r['hits']]})
 summary.append({'motor':n,'options':options})
(D/'reviews'/'route_summary.json').write_text(json.dumps({'motors':summary,'nominal_fixed_exit_only':True,'full_harness_validated':False},indent=2),encoding='utf-8');print('FINISHED',flush=True)
