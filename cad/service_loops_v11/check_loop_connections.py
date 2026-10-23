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

# Physical-wire envelopes for the two endpoint fixed exits, separate from the reserved CAD templates.
rows=[]
for q in sorted({r['q_deg'] for r in curves}):
 c=math.cos(math.radians(q));s=math.sin(math.radians(q));R=np.array([[1,0,0],[0,c,-s],[0,s,c]]);fixed=[]
 for motor in [0,1]:
  for lane,y in enumerate([-11.5,-9,-6.5]):
   points=[np.array([-25.6,y,-8.]),np.array([-34.5,y,-8.])]
   for t in np.linspace(0,math.pi/2,46)[1:]:points.append(np.array([-34.5-10*math.sin(t),y,-8-10*(1-math.cos(t))]))
   points.append(np.array([-44.5,y,-36.]))
   if motor:points=[R@(p+np.array([0.,55.,0.])) for p in points]
   for a,b in zip(points,points[1:]):fixed.append((motor,lane,*capseg(a,b,.952)))
 bestself=2.;besttail=2.;selfhit=False;tailhit=False;nearest=None
 for row in [r for r in curves if r['q_deg']==q]:
  lane=row['lane'];segs=segments(np.array(row['controls_mm']));caps=[capseg(a,b,.96) for a,b,e in segs];lengths=np.array([np.linalg.norm(b-a) for a,b,e in segs]);ends=np.r_[0,np.cumsum(lengths)];centers=(ends[:-1]+ends[1:])/2
  for i,(o,l,h) in enumerate(caps):
   for j in range(i+1,len(caps)):
    if centers[j]-centers[i]<=6:continue
    ob,ll,hh=caps[j];gap=np.maximum(0,np.maximum(l-hh,ll-h))
    if gap@gap>bestself**2:continue
    dd=float(fcl.distance(o,ob,req,fcl.DistanceResult()));hit=dd<=0 and bool(fcl.collide(o,ob,creq,fcl.CollisionResult()));selfhit|=hit;bestself=min(bestself,max(0.,dd))
   for motor,otherlane,ob,ll,hh in fixed:
    if lane==otherlane and ((motor==0 and centers[i]<3) or(motor==1 and ends[-1]-centers[i]<3)):continue
    gap=np.maximum(0,np.maximum(l-hh,ll-h))
    if gap@gap>besttail**2:continue
    dd=float(fcl.distance(o,ob,req,fcl.DistanceResult()));hit=dd<=0 and bool(fcl.collide(o,ob,creq,fcl.CollisionResult()));tailhit|=hit
    if dd<besttail:besttail=max(0.,dd);nearest={'loop_lane':lane,'tail_motor':motor,'tail_lane':otherlane}
 rows.append({'q_deg':q,'minimum_nonlocal_self_gap_capped_2mm':bestself,'self_collision':selfhit,'minimum_own_exit_gap_capped_2mm':besttail,'own_exit_collision':tailhit,'nearest_exit_pair':nearest})
out={'scope':'41 little MCP angles. Physical-wire enclosures0.96mm(loop) and0.952mm(fixed exits), original OD1.9mm. Each wire self-contact checked only between portions separated by>6mm chordwise arclength. Connected same-wire joins within3mm of either end are excluded as expected tube adjacency; all other own-tail pairs are checked. Complements3-wire mutual check; no cable mechanics or electrical approval.','cases':len(rows),'self_collision_cases':sum(r['self_collision'] for r in rows),'own_exit_collision_cases':sum(r['own_exit_collision'] for r in rows),'minimum_nonlocal_self_gap_capped_2mm':min(r['minimum_nonlocal_self_gap_capped_2mm'] for r in rows),'minimum_own_exit_gap_capped_2mm':min(r['minimum_own_exit_gap_capped_2mm'] for r in rows),'rows':rows,'full_harness_approved':False};(D/'reviews'/'loop_connections.json').write_text(json.dumps(out,indent=2),encoding='utf-8');print({k:v for k,v in out.items() if k!='rows'},flush=True)
