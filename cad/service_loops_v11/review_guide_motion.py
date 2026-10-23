"""Only added saddle material versus actual hand CAD, not inherited baseline contacts."""
import sys,json,copy,time,numpy as np,trimesh,fcl
from pathlib import Path
D=Path(__file__).resolve().parent;B=D.parent/'modular_roots_v9';W=D.parent/'wiring_access_v10';sys.path.insert(0,str(B));from kinematics_numpy import C,META,forward
n=np.load(B/'collision_meshes.npz');models={};bounds={}
def add(name,v,f):
 b=fcl.BVHModel();b.beginModel(len(v),len(f));b.addSubModel(v,f);b.endModel();models[name]=b;lo=v.min(0);hi=v.max(0);bounds[name]=np.array([[x,y,z] for x in [lo[0],hi[0]] for y in [lo[1],hi[1]] for z in [lo[2],hi[2]]])
for a in sorted({it['asset'] for it in META}):
 if a=='Thumb_metacarpal':m=trimesh.load_mesh(W/'parts'/'Thumb_metacarpal_channel_candidate.stl');v,f=m.vertices,m.faces
 else:v,f=n[a+'_vertices'],n[a+'_faces']
 add(a,v,f)
G=[('little_spread_carrier','Little_root_cable_saddle'),('little_mcp_link','Little_MCP_link_cable_saddle')];ids={it['name']:i for i,it in enumerate(META)}
for parent,a in G:
 for suffix in ['', '_added']:
  m=trimesh.load_mesh(D/'parts'/(a+suffix+'.stl'));add(a+suffix,m.vertices,m.faces)
req=fcl.DistanceRequest(enable_nearest_points=True);creq=fcl.CollisionRequest(num_max_contacts=1);rows=[];cache={};start=time.monotonic()
for pn,bp in C['poses'].items():
 for spread in [-8,0,3]:
  for q in np.arange(-20,60.01,1):
   p=copy.deepcopy(bp);p['little'][0]=spread;p['little'][1]=float(q);M=forward(p);obs=[]
   for i,it in enumerate(META):
    if it['role'].startswith('arm_'):continue
    asset=dict(G).get(it['name'],it['asset']);m=M[i];pts=bounds[asset]@m[:3,:3].T+m[:3,3];obs.append((i,asset,fcl.CollisionObject(models[asset],fcl.Transform(m[:3,:3],m[:3,3])),pts.min(0),pts.max(0)))
   best=3.;pair=None;fail=[]
   for parent,a in G:
    i=ids[parent];ma=M[i];pts=bounds[a+'_added']@ma[:3,:3].T+ma[:3,3];lo,hi=pts.min(0),pts.max(0);ob=fcl.CollisionObject(models[a+'_added'],fcl.Transform(ma[:3,:3],ma[:3,3]));ol=np.array([z[3] for z in obs]);oh=np.array([z[4] for z in obs]);gap=np.maximum(0,np.maximum(lo-oh,ol-hi));near=np.where(np.einsum('ij,ij->i',gap,gap)<best*best)[0]
    for oi in near:
     j,asset,other,ll,hh=obs[oi]
     if i==j:continue
     rel=np.linalg.inv(ma)@M[j];key=(parent,META[j]['name'],np.round(rel,8).tobytes());cached=cache.get(key)
     if cached is None:
      hit=bool(fcl.collide(ob,other,creq,fcl.CollisionResult()));dist=0. if hit else max(0.,float(fcl.distance(ob,other,req,fcl.DistanceResult())));cache[key]=(dist,hit)
     else:dist,hit=cached
     if dist<best:best=dist;pair=[parent,META[j]['name']]
     if hit or dist<.3:fail.append({'pair':[parent,META[j]['name']],'gap_mm':dist,'collision':hit})
   rows.append({'background_pose':pn,'spread_deg':spread,'mcp_deg':float(q),'minimum_added_gap_capped_3mm':best,'pair':pair,'failures':fail})
 print(pn,'done',flush=True)
out={'scope':'Added material only in two little-finger cable saddle candidates vs all hand CAD, including hardware and each other. 6 saved backgrounds x3 spread x81 MCP samples. Original baseline body checks not repeated or claimed. Near-limit mesh results require exact BRep confirmation.','samples':len(rows),'failed_samples':sum(bool(r['failures']) for r in rows),'minimum_added_gap_mm':min(r['minimum_added_gap_capped_3mm'] for r in rows),'required_gap_mm':.3,'rows':rows,'elapsed_s':time.monotonic()-start,'manufacturing_release':False};(D/'reviews'/'guide_motion_mesh.json').write_text(json.dumps(out,indent=2),encoding='utf-8');print({k:v for k,v in out.items() if k!='rows'},flush=True)
