import sys,json,copy,numpy as np
from pathlib import Path
D=Path(__file__).resolve().parent;B=D.parent/'modular_roots_v9';sys.path.insert(0,str(B));from kinematics_numpy import C,META,forward
n=np.load(B/'collision_meshes.npz');o=[]
for a in ['Compact_root','Flex_link_55','M077_stator','S101_relief','H101_vendor']:
 v=n[a+'_vertices'];o.append({'asset':a,'min':v.min(0).tolist(),'max':v.max(0).tolist()})
for q in [-20,0,30,60]:
 p=copy.deepcopy(C['poses']['Open']);p['little'][1]=q;m=forward(p);base=m[next(i for i,x in enumerate(META) if x['name']=='little_mcp_motor')];dst=np.linalg.inv(base)@m[next(i for i,x in enumerate(META) if x['name']=='little_pip_motor')];o.append({'q':q,'start':[-44.5,-9,-36],'end':(dst@[-47,-36,-8,1])[:3].tolist(),'end_tangent':(dst[:3,:3]@np.array([0,1,0])).tolist()})
print(json.dumps(o,indent=2))
