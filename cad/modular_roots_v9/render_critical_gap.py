import os,json,numpy as np,trimesh
from pathlib import Path
D=Path(__file__).resolve().parent;B=D.parent/'modular_roots_v8';os.environ['MPLCONFIGDIR']=str(D/'mpl_config')
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
plt.rcParams['font.family']='Malgun Gothic';plt.rcParams['axes.unicode_minus']=False
S=json.loads((D/'scene.json').read_text(encoding='utf-8'))['Backbend'];gap=json.loads((D/'reviews/critical_gap.json').read_text(encoding='utf-8'));fig,axs=plt.subplots(1,2,figsize=(11,5),facecolor='#eff4fa')
for ax,rev,p,r in zip(axs,[8,9],[B,D],gap['rows']):
 data=np.load(p/'collision_meshes.npz');ax.set_facecolor('#eff4fa')
 for name,col,label in [('index_mcp_link','#b88654','검지 링크'),('Middle_root_cartridge','#278b93','중지 고정대')]:
  it=next(x for x in S if x['name']==name);a=it['asset'];v=data[a+'_vertices']@np.array(it['R']).T+it['pos_mm'];m=trimesh.Trimesh(v,data[a+'_faces'],process=False);segments=trimesh.intersections.mesh_plane(m,[1,0,0],[41.8,0,0])
  ax.add_collection(LineCollection(segments[:,:,[1,2]],colors=col,linewidths=2,label=label))
 w=r['nearest_points_mm'][0];p0=np.array(w[0])[1:];p1=np.array(w[1])[1:];ax.annotate('',xy=p0,xytext=p1,arrowprops={'arrowstyle':'<->','color':'#a6453b','lw':1.8});ax.text(181,-11.5,format(r['distance_mm'],'.2f')+'mm',color='#a6453b',fontsize=15,ha='center');ax.set_xlim(172,194);ax.set_ylim(-34,-9);ax.set_aspect('equal');ax.set_title(('수정 전' if rev==8 else '수정 후')+' · V'+str(rev),pad=12);ax.set_xlabel('손가락 방향 Y(mm)');ax.set_ylabel('손바닥 방향 Z(mm)');ax.grid(alpha=.2)
axs[0].legend(loc='lower left',fontsize=9);fig.suptitle('뒤로 젖힐 때의 국부 간섭 여유 · 실제 CAD 단면',fontsize=17);fig.text(.08,.015,'X=41.8mm단면 / 수치는3D BRep최소 거리 · 관절축·손가락 길이·볼트 위치 유지',fontsize=10,color='#455a70');fig.tight_layout(rect=[0,.05,1,.94]);fig.savefig(D/'reviews/critical_gap_comparison.png',dpi=170);plt.close(fig)
