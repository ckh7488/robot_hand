import sys,json,copy,numpy as np,fcl
from pathlib import Path
D=Path(__file__).resolve().parent;B=D.parent/'modular_roots_v9';sys.path.insert(0,str(B));from kinematics_numpy import C,META,forward,rot
from mesh_collision import Checker
P=copy.deepcopy(C['poses']['Splay']);P['middle'][0]+=3;P['ring'][0]-=3;K=forward(P);O=forward(C['poses']['Splay']);check=Checker();out=[]
for n,err in [('middle_spread_carrier',3),('ring_spread_carrier',-3)]:
 i=next(i for i,x in enumerate(META) if x['name']==n);f=next(f for f in C['hand']['fingers'] if n.startswith(f['name']+'_'));rx,ry,rz=f['base_rpy'];axis=rot(2,rz)@rot(1,ry)@rot(0,rx)@np.array([0.,0.,1.]);a=np.deg2rad(-err);V=axis;X=np.array([[0,-V[2],V[1]],[V[2],0,-V[0]],[-V[1],V[0],0]]);R=np.eye(3)+np.sin(a)*X+(1-np.cos(a))*(X@X);M=O[i].copy();M[:3,:3]=R@M[:3,:3];M[:3,3]=f['root']+R@(M[:3,3]-f['root']);out.append({'name':n,'rpy':f['base_rpy'],'axis':axis.tolist(),'diff':np.max(abs(K[i]-M)),'matrix':K[i].tolist()})
print(json.dumps(out,indent=2));print(check.evaluate(P,threshold=1.1)['below_threshold'][:8])
