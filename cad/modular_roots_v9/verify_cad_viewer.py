import json,numpy as np,trimesh,mujoco
from scipy.spatial import cKDTree
from cad_viewer import display_model,apply_pose
from kinematics_numpy import C,D,META,forward,verify
m,d,inner=display_model();trees={};rows=[]
for asset in set(it['asset'] for it in META):trees[asset]=cKDTree(trimesh.load_mesh(D/'meshes'/(asset+'.stl')).vertices)
for pn,p in C['poses'].items():
 apply_pose(m,d,inner,p);raw=forward(p);worst=0
 for g,it in enumerate(META):
  mid=m.geom_dataid[g];start=m.mesh_vertadr[mid];num=m.mesh_vertnum[mid];sel=np.linspace(0,num-1,min(64,num),dtype=int);v=m.mesh_vert[start+sel];world=v@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g];local=(world-raw[g,:3,3]/1000)@raw[g,:3,:3];dist=trees[it['asset']].query(local,k=1)[0];worst=max(worst,float(dist.max()*1000))
 row={'pose':pn,'max_actual_mesh_vertex_position_error_mm':worst};rows.append(row);print(row,flush=True)
assert max(r['max_actual_mesh_vertex_position_error_mm'] for r in rows)<.001
(D/'reviews/viewer_cad_alignment.json').write_text(json.dumps({'kinematics':verify(),'parts':len(META),'dynamic_bodies':m.nbody-1,'actuators':m.nu,'claims_motor_response':False,'mesh_alignment':rows,'passed':True},indent=2))
