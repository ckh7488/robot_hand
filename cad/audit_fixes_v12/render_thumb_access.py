import os,json,numpy as np,trimesh
from pathlib import Path
D=Path(__file__).resolve().parent;B=D.parent/'modular_roots_v9';W=D.parent/'wiring_access_v10';os.environ['MPLCONFIGDIR']=str(D/'mpl_config')
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
plt.rcParams['font.family']='Malgun Gothic';plt.rcParams['axes.unicode_minus']=False
J=json.loads((D/'reviews'/'thumb_tool_access.json').read_text(encoding='utf-8'));S=json.loads((B/'scene.json').read_text(encoding='utf-8'))['Open'];it=next(x for x in S if x['name']=='thumb_mcp_link');R=np.array(it['R']);p=np.array(it['pos_mm']);head=(np.array(J['exact_head_top_world_mm'])-p)@R;axis=np.array(J['exact_outward_axis_world'])@R
meshes=[trimesh.load_mesh(W/'parts'/'Thumb_metacarpal_channel_candidate.stl'),trimesh.load_mesh(D/'parts'/'Thumb_metacarpal_tool_access.stl')];fig=plt.figure(figsize=(13,6),facecolor='#eff4fa');fig.suptitle('V12 · 엄지 나사 드라이버 통로 수정',fontsize=18)
for k,m in enumerate(meshes):
 ax=fig.add_subplot(1,2,k+1,projection='3d');ax.set_facecolor('#eff4fa');ax.add_collection3d(Poly3DCollection(m.vertices[m.faces],facecolors='#b6c5cf',edgecolor='none',alpha=1,shade=True))
 tool=trimesh.creation.cylinder(radius=1.4,height=52,sections=24);T=trimesh.geometry.align_vectors([0,0,1],axis);T[:3,3]=head+axis*34;tool.apply_transform(T);ax.add_collection3d(Poly3DCollection(tool.vertices[tool.faces],facecolors='#cc4d37' if k==0 else '#2a91bb',edgecolor='none',alpha=.65,shade=True))
 bounds=np.vstack([meshes[0].vertices,head+axis*60]);lo,hi=bounds.min(0),bounds.max(0);c=(lo+hi)/2;span=max(hi-lo)*.55;ax.set(xlim=(c[0]-span,c[0]+span),ylim=(c[1]-span,c[1]+span),zlim=(c[2]-span,c[2]+span));ax.set_box_aspect((1,1,1),zoom=1.35);ax.view_init(25,50);ax.set_axis_off();ax.set_title('수정 전 · 축과 벽이 겹침' if k==0 else '수정 후 · Ø3.5mm 통로',fontsize=15)
fig.text(.045,.08,'실제 CAD 부품 + Ø2.8mm 드라이버 축 · 겹침24.63mm³ → 0 / 축–벽 간격0.35mm',fontsize=13,color='#29475e');fig.text(.045,.025,'기존 배선 홈 유지 · 제거38.23mm³ · 최소 부분조립 단계 검사 / 강도·너트 고정·전체 정비성은 추가 검토',fontsize=11,color='#526477');fig.tight_layout(rect=[0,.11,1,.94]);fig.savefig(D/'reviews'/'thumb_tool_access.png',dpi=150);plt.close(fig)
