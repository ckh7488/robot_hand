import os
os.environ['MPLCONFIGDIR']=r'E:/hand_recog/mujoco-direct/cad/index_joint_v1/tmp/mpl'
import json, numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from pathlib import Path
r=Path(r'E:/hand_recog/mujoco-direct/cad/index_joint_v1')
d=json.loads((r/'proto_meshes.json').read_text())
fig=plt.figure(figsize=(12,6),facecolor='#eff3f7')
cols={'body':'#404956','horn':'#f4bd54','idler':'#f4bd54','hinge':'#408bd6','fixed':'#62ad89'}
for k,(az,el) in enumerate([(32,24),(-140,25)]):
 ax=fig.add_subplot(1,2,k+1,projection='3d');ax.set_proj_type('ortho')
 for n,m in d.items():
  v=np.array(m['vertices']);t=np.array(m['triangles']);pc=Poly3DCollection(v[t],facecolor=cols[n],edgecolor='none',linewidth=0,rasterized=True);ax.add_collection3d(pc)
 ax.set_xlim(-28,28);ax.set_ylim(-28,28);ax.set_zlim(-18,25);ax.set_box_aspect((56,56,43));ax.view_init(elev=el,azim=az);ax.set_xlabel('X shaft (mm)');ax.set_ylabel('Y finger (mm)');ax.set_zlabel('Z palm (mm)')
 ax.set_title('Vendor CAD assembly / neutral')
fig.tight_layout();fig.savefig(r/'assembly_probe.png',dpi=140);print('rendered')
