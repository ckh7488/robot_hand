"""Conservative capsule enclosure of spatial Bezier wires vs actual CAD mesh, plus physical-wire pair distance."""
import sys,json,time,copy,math,numpy as np,trimesh,fcl
from pathlib import Path
D=Path(__file__).resolve().parent;B=D.parent/'modular_roots_v9';W=D.parent/'wiring_access_v10';sys.path.insert(0,str(B));from kinematics_numpy import C,META,forward
J=json.loads((D/'reviews'/'service_loop_curves.json').read_text(encoding='utf-8'));assert J['chosen'];curves=J['chosen']['curves'];n=np.load(B/'collision_meshes.npz');models={};corners={}
def add(asset,v,f):
 b=fcl.BVHModel();b.beginModel(len(v),len(f));b.addSubModel(v,f);b.endModel();models[asset]=b;lo=v.min(0);hi=v.max(0);corners[asset]=np.array([[x,y,z] for x in [lo[0],hi[0]] for y in [lo[1],hi[1]] for z in [lo[2],hi[2]]])
for a in sorted({it['asset'] for it in META}):
 if a=='Thumb_metacarpal':
  tm=trimesh.load_mesh(W/'parts'/'Thumb_metacarpal_channel_candidate.stl');v,f=tm.vertices,tm.faces
 else:v,f=n[a+'_vertices'],n[a+'_faces']
 add(a,v,f)
selected=json.loads((D/'selected_routes.json').read_text(encoding='utf-8'))['selected'];templates={t['name']:t for t in json.loads((W/'route_templates.json').read_text(encoding='utf-8'))['templates']}
for r in selected:
 t=templates[r['template']]
 for a in [t['plug_asset'],*t['wire_assets']]:
  if a not in models:
   m=trimesh.load_mesh(W/'parts'/(a+'.stl'));add(a,m.vertices,m.faces)
guide_map={'little_spread_carrier':'Little_root_cable_saddle','little_mcp_link':'Little_MCP_link_cable_saddle'}
for a in guide_map.values():
 m=trimesh.load_mesh(D/'parts'/(a+'.stl'));add(a,m.vertices,m.faces)
ids={it['name']:i for i,it in enumerate(META)};mid=ids['little_mcp_motor'];handids=[i for i,it in enumerate(META) if not it['role'].startswith('arm_')];req=fcl.DistanceRequest(enable_nearest_points=True);creq=fcl.CollisionRequest(num_max_contacts=1)
def bound(P):
 a,b=P[0],P[-1];d=b-a;den=d@d;pr=np.clip(((P-a)@d)/max(den,1e-20),0,1);return float(np.max(np.linalg.norm(P-a-pr[:,None]*d,axis=1)))
def split(P):
 levels=[P]
 while len(levels[-1])>1:levels.append((levels[-1][1:]+levels[-1][:-1])/2)
 return np.array([p[0] for p in levels]),np.array([p[-1] for p in levels[::-1]])
def segments(P):
 if bound(P)<=.01 and np.linalg.norm(P[-1]-P[0])<=2:return [(P[0],P[-1],bound(P))]
 a,b=split(P);return segments(a)+segments(b)
def capseg(a,b,r):
 d=b-a;L=np.linalg.norm(d);z=d/L;h=np.array([1.,0,0]) if abs(z[0])<.8 else np.array([0.,1,0]);x=np.cross(h,z);x/=np.linalg.norm(x);y=np.cross(z,x);R=np.column_stack([x,y,z]);return fcl.CollisionObject(fcl.Capsule(r,float(L)),fcl.Transform(R,(a+b)/2)),np.minimum(a,b)-r,np.maximum(a,b)+r
prepared={};maxerr=0.;wirepairs=[]
for q in sorted({r['q_deg'] for r in curves}):
 wires=[]
 for r in sorted([r for r in curves if r['q_deg']==q],key=lambda r:r['lane']):
  segs=segments(np.array(r['controls_mm']));maxerr=max(maxerr,max(s[2] for s in segs));wires.append(segs)
 prepared[q]=wires;best=2.;hit=False
 for i,j in [(0,1),(1,2),(0,2)]:
  A=[capseg(a,b,.96) for a,b,e in wires[i]];Z=[capseg(a,b,.96) for a,b,e in wires[j]]
  for o,l,h in A:
   for p,ll,hh in Z:
    gap=np.maximum(0,np.maximum(l-hh,ll-h))
    if gap@gap>best*best:continue
    dd=float(fcl.distance(o,p,req,fcl.DistanceResult()));best=min(best,max(0,dd));hit=hit or dd<0
 wirepairs.append({'q_deg':q,'minimum_actual_wire_gap_capped_2mm':best,'surface_collision':hit})
rows=[];start=time.monotonic()
for pn,basepose in C['poses'].items():
 for q,wires in prepared.items():
  pose=copy.deepcopy(basepose);pose['little'][1]=q;M=forward(pose);A=M[mid];obs=[]
  for i in handids:
   it=META[i];asset=guide_map.get(it['name'],it['asset']);m=np.linalg.inv(A)@M[i];p=corners[asset]@m[:3,:3].T+m[:3,3];obs.append((it['name'],fcl.CollisionObject(models[asset],fcl.Transform(m[:3,:3],m[:3,3])),p.min(0),p.max(0)))
  for s in selected:
   if s['motor'] in ['little_mcp_motor','little_pip_motor']:continue # connected endpoints/bundle joins; body still checked
   m=np.linalg.inv(A)@M[ids[s['motor']]];t=templates[s['template']]
   for asset in [t['plug_asset'],*t['wire_assets']]:
    p=corners[asset]@m[:3,:3].T+m[:3,3];obs.append((s['motor']+':'+asset,fcl.CollisionObject(models[asset],fcl.Transform(m[:3,:3],m[:3,3])),p.min(0),p.max(0)))
  obslo=np.array([o[2] for o in obs]);obshi=np.array([o[3] for o in obs]);best=2.;pair=None;bad=[]
  for lane,segs in enumerate(wires):
   for a,b,e in segs:
    ob,l,h=capseg(a,b,1.26)
    gaps=np.maximum(0,np.maximum(l-obshi,obslo-h));near=np.where(np.einsum('ij,ij->i',gaps,gaps)<=max(best,.3)**2)[0]
    for oi in near:
     name,obj,ll,hh=obs[oi]
     res=fcl.DistanceResult();dd=float(fcl.distance(ob,obj,req,res));hit=dd<=0 and bool(fcl.collide(ob,obj,creq,fcl.CollisionResult()));dd=0. if hit else max(0,dd)
     if dd<best:best=dd;pair=[lane,name]
     if hit or dd<.3:bad.append({'lane':lane,'part':name,'gap_mm':dd,'collision':hit})
  rows.append({'background_pose':pn,'little_mcp_deg':q,'minimum_reserved_gap_capped_2mm':best,'pair':pair,'failures':list({(r['lane'],r['part']):r for r in bad}.values())})
 print(pn,'done',flush=True)
out={'scope':'6 background saved finger configurations x41 little MCP angles; actual nonconvex CAD meshes and18 other motor fixed exits. Loop endpoints join their own two fixed exits; own bundles are excluded only from wire-to-wire checking, not body collision. Does not certify every independent joint state, connector mating, dynamic wire equilibrium or electrical topology.','conservative_curve_enclosure':'Recursive de Casteljau subdivision; each control hull within0.01mm of chord; capsules radius1.26mm contain nominal0.95mm wire+0.3mm reserve+0.01mm bound.','maximum_control_hull_error_mm':maxerr,'wire_wire_radius_mm':.96,'body_required_reserved_gap_mm':.3,'cases':len(rows),'failed_cases':sum(bool(r['failures']) for r in rows),'minimum_body_gap_mm':min(r['minimum_reserved_gap_capped_2mm'] for r in rows),'minimum_wire_wire_gap_mm':min(r['minimum_actual_wire_gap_capped_2mm'] for r in wirepairs),'wire_pair_cases':wirepairs,'rows':rows,'elapsed_s':time.monotonic()-start,'full_harness_approved':False};(D/'reviews'/'loop_mesh_clearance.json').write_text(json.dumps(out,indent=2),encoding='utf-8');print({k:v for k,v in out.items() if k not in ['rows','wire_pair_cases']},flush=True)
