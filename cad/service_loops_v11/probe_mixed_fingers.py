"""Structured independently moving fingers using frozen actual CAD; source geometry is read-only."""
import sys,json,copy,itertools,time,numpy as np,trimesh
from pathlib import Path
D=Path(__file__).resolve().parent;B=D.parent/'modular_roots_v9';W=D.parent/'wiring_access_v10';sys.path.insert(0,str(B))
from kinematics_numpy import C
from mesh_collision import Checker
m=trimesh.load_mesh(W/'parts'/'Thumb_metacarpal_channel_candidate.stl');np.savez_compressed(D/'thumb_candidate_mesh.npz',vertices=m.vertices,faces=m.faces);ck=Checker('hand',overrides={'Thumb_metacarpal':str(D/'thumb_candidate_mesh.npz')})
fingers=[f['name'] for f in C['hand']['fingers']];states={'open':[0,0,0,0],'curl':[0,35,35,20],'close':[0,60,45,30],'hook':[0,0,60,45],'backbend':[0,-20,0,0]};thumbs={p:C['poses'][p]['thumb'] for p in ['Open','Curl','Close','ThumbSweep']};probes=[]
for bits in itertools.product([0,1],repeat=5):
 p=copy.deepcopy(C['poses']['Open'])
 for f,b in zip(fingers,bits):p[f]=copy.deepcopy(C['poses']['Close' if b else 'Open'][f])
 probes.append(('binary_'+''.join(map(str,bits)),p))
for a,b in zip(fingers[:3],fingers[1:4]):
 for sa,sb in itertools.product(states,repeat=2):
  p=copy.deepcopy(C['poses']['Open']);p[a]=states[sa].copy();p[b]=states[sb].copy();probes.append((a+'_'+sa+'__'+b+'_'+sb,p))
for who in fingers[:4]:
 for sn,sv in states.items():
  for tn,tv in thumbs.items():
   p=copy.deepcopy(C['poses']['Open']);p[who]=sv.copy();p['thumb']=tv.copy();probes.append((who+'_'+sn+'__thumb_'+tn,p))
unique={json.dumps(p,sort_keys=True):(n,p) for n,p in probes};rows=[];t0=time.monotonic()
for n,p in unique.values():
 r=ck.evaluate(p,cap=1.3);rows.append({'name':n,'pose':p,**r});print(n,'near',len(r['below_threshold']),'collisions',sum(x['surface_collision_or_containment'] for x in r['below_threshold']),flush=True)
 if len(rows)%10==0:(D/'reviews'/'mixed_fingers.json').write_text(json.dumps({'scope':'Structured independent finger states; no global reachability/continuous guarantee. Uses frozen V9 CAD plus V10 thumb cable channel.','rows':rows,'elapsed_seconds':time.monotonic()-t0},indent=2),encoding='utf-8')
out={'scope':'Structured independent finger states; not a complete joint configuration space. Body only, cables excluded.','rows':rows,'elapsed_seconds':time.monotonic()-t0,'sampled_poses':len(rows),'poses_with_body_collision':sum(any(h['surface_collision_or_containment'] for h in r['below_threshold']) for r in rows),'poses_below_1p1mm':sum(bool(r['below_threshold']) for r in rows)};(D/'reviews'/'mixed_fingers.json').write_text(json.dumps(out,indent=2),encoding='utf-8');print('FINISHED',out['sampled_poses'],out['poses_with_body_collision'],flush=True)
