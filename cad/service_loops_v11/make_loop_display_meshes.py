"""Light display-only tubes derived from validated Bezier controls. Collision never uses these meshes."""
import json,math,numpy as np,trimesh
from pathlib import Path
from scipy.special import comb
D=Path(__file__).resolve().parent;J=json.loads((D/'reviews'/'service_loop_curves.json').read_text(encoding='utf-8'));rows=[]
def hull_error(P):
 a,b=P[0],P[-1];d=b-a;f=np.clip((P-a)@d/max(d@d,1e-20),0,1);return max(np.linalg.norm(P-a-f[:,None]*d,axis=1))
def leaves(P,t0=0.,t1=1.):
 if hull_error(P)<=.005 and np.linalg.norm(P[-1]-P[0])<=1:return [(P[-1],t1)]
 lev=[P]
 while len(lev[-1])>1:lev.append((lev[-1][1:]+lev[-1][:-1])/2)
 a=np.array([p[0] for p in lev]);b=np.array([p[-1] for p in lev[::-1]]);tm=(t0+t1)/2;return leaves(a,t0,tm)+leaves(b,tm,t1)
for row in [r for r in J['chosen']['curves'] if r['q_deg'] in [-20,0,60]]:
 P=np.array(row['controls_mm']);pts=[(P[0],0.),*leaves(P)];xyz=np.array([p[0] for p in pts]);t=np.array([p[1] for p in pts]);basis=np.array([comb(4,i)*t**i*(1-t)**(4-i) for i in range(5)]).T;tan=basis@(5*np.diff(P,axis=0));tan/=np.linalg.norm(tan,axis=1)[:,None];norm=np.array([1.,0.,0.]);verts=[];sides=16
 for p,v in zip(xyz,tan):
  norm-=v*np.dot(norm,v);norm/=np.linalg.norm(norm);b=np.cross(v,norm)
  for a in np.arange(sides)*2*math.pi/sides:verts.append(p+.95*(math.cos(a)*norm+math.sin(a)*b))
 faces=[]
 for i in range(len(pts)-1):
  for k in range(sides):a=i*sides+k;bb=i*sides+(k+1)%sides;c=(i+1)*sides+(k+1)%sides;dd=(i+1)*sides+k;faces.extend([(a,bb,c),(a,c,dd)])
 first=len(verts);verts.extend([xyz[0],xyz[-1]])
 for k in range(sides):faces.extend([(first,(k+1)%sides,k),(first+1,(len(pts)-1)*sides+k,(len(pts)-1)*sides+(k+1)%sides)])
 mesh=trimesh.Trimesh(np.array(verts),np.array(faces),process=False);assert mesh.is_watertight and mesh.is_winding_consistent;name='Little_loop_q'+str(int(row['q_deg']))+'_wire'+str(row['lane']+1);mesh.export(D/'parts'/(name+'.stl'));rows.append({'asset':name,'vertices':len(verts),'faces':len(faces),'watertight':True})
p=D/'reviews'/'loop_cad_export.json';j=json.loads(p.read_text(encoding='utf-8'));j.pop('wire_display_mesh_deflection_mm',None);j['wire_display_mesh']={'circle_sides':16,'centerline_control_hull_error_mm':.005,'maximum_centerline_chord_mm':1,'collision_uses_display_mesh':False};p.write_text(json.dumps(j,indent=2),encoding='utf-8');(D/'reviews'/'display_meshes.json').write_text(json.dumps({'scope':'Display only; actual STEP/Bezier control points and capsule collision geometry unchanged.','assets':rows},indent=2),encoding='utf-8');print('Display meshes complete',sum(r['faces'] for r in rows),'faces total',flush=True)
