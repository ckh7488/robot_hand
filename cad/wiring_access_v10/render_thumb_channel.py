import os,json,numpy as np,trimesh
from pathlib import Path
D=Path(__file__).resolve().parent;B=D.parent/'modular_roots_v9';os.environ['MPLCONFIGDIR']=str(D/'mpl_config')
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
plt.rcParams['font.family']='Malgun Gothic';plt.rcParams['axes.unicode_minus']=False
fig=plt.figure(figsize=(13,5.8),facecolor='#eff4fa');fig.suptitle('엄지 링크 · 배선을 위한 국부 홈 후보',fontsize=18,y=.98)
old=trimesh.load_mesh(B/'parts'/'Thumb_metacarpal.stl');new=trimesh.load_mesh(D/'parts'/'Thumb_metacarpal_channel_candidate.stl');rep=json.loads((D/'reviews'/'thumb_channel.json').read_text(encoding='utf-8'))
for i,m in enumerate([old,new]):
 ax=fig.add_subplot(1,2,i+1,projection='3d');ax.set_facecolor('#eff4fa');ax.add_collection3d(Poly3DCollection(m.vertices[m.faces],facecolors='#aebdc9',edgecolor='none',alpha=1,shade=True))
 if i==1:
  S=json.loads((B/'scene.json').read_text(encoding='utf-8'))['Open'];it=next(it for it in S if it['name']=='thumb_mcp_link');mt=next(it for it in S if it['name']=='thumb_pip_motor');R=np.array(it['R']);Rm=np.array(mt['R']);t=np.array(it['pos_mm']);tm=np.array(mt['pos_mm'])
  for lane in [1,2,3]:
   w=trimesh.load_mesh(D/'parts'/('negativeZ_outboard_wire'+str(lane)+'.stl'));v=(w.vertices@Rm.T+tm-t)@R;ax.add_collection3d(Poly3DCollection(v[w.faces],facecolor='#179579',edgecolor='none'))
 lo,hi=old.bounds;center=(lo+hi)/2;span=max(hi-lo)*.65;ax.set(xlim=(center[0]-span,center[0]+span),ylim=(center[1]-span,center[1]+span),zlim=(center[2]-span,center[2]+span));ax.set_box_aspect((1,1,1));ax.view_init(24,55);ax.set_axis_off();ax.set_title('V9 원본' if i==0 else '별도 후보 · 홈과 전선 여유',fontsize=14)
fig.text(.05,.04,'제거 부피 '+format(rep['removed_volume_mm3'],'.1f')+'mm³ / 약'+format(100*rep['removed_volume_mm3']/rep['old_volume_mm3'],'.2f')+'% · 체결 부품과 제거 부위 간 최소 '+format(min(r['removed_material_distance_mm'] for r in rep['related_fasteners']),'.2f')+'mm',fontsize=12,color='#3d556d');fig.text(.05,.007,'실제 CAD 차집합 / 볼트 위치·축 유지 · 출력 강도와 연결된 하네스는 아직 검증 전',fontsize=11,color='#526477');fig.tight_layout(rect=[0,.075,1,.94]);fig.savefig(D/'reviews'/'thumb_channel.png',dpi=160);plt.close(fig)
