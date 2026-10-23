import numpy as np,json,copy
from kinematics_numpy import D,C
from mesh_collision import Checker
p=np.load(D/'collision_meshes.npz');v=p['Compact_root_vertices'];sel=v[(v[:,1]>6.9)&(v[:,1]<11.1)&(v[:,2]>12.9)&(v[:,2]<20)]
print('source',D,C['assembly']);print('slice',sel.tolist());ck=Checker('hand');names=[it['name'] for it in ck.items];ck.pairs=np.array([[names.index('index_spread_carrier'),names.index('index_mcp_hinge')]])
pose=copy.deepcopy(C['poses']['Backbend']);pose['index'][1]=-23;print(ck.evaluate(pose,cap=20,threshold=1.1))
