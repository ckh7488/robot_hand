import os
from pathlib import Path
os.environ['MPLCONFIGDIR']=str(Path(__file__).resolve().parent/'mpl_config')
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib import colors
from pathlib import Path
F=Path(__file__).resolve().parent;I=np.load(F/'mesh_index.npz');fields=np.load(F/'fields.npz');res=json.loads((F/'results.json').read_text(encoding='utf-8'));xyz=I['xyz'];conn=I['conn'];faces=[]
facepat=[[0,2,1,6,5,4],[0,1,3,4,8,7],[1,2,3,5,9,8],[2,0,3,6,7,9]]
for pat in facepat:faces.append(conn[:,pat])
faces=np.concatenate(faces);keys=np.sort(faces[:,:3],axis=1);_,inverse,counts=np.unique(keys,axis=0,return_inverse=True,return_counts=True);b=faces[counts[inverse]==1];tris=np.concatenate([b[:,[0,3,5]],b[:,[3,1,4]],b[:,[5,4,2]],b[:,[3,4,5]]])
plt.rcParams['font.family']='Malgun Gothic';plt.rcParams['axes.unicode_minus']=False
fig=plt.figure(figsize=(18,7.2),facecolor='#f0f4fa');fig.suptitle('실제 손바닥 CAD · 중력에 따른 휨 검토',fontsize=20,y=.98);fig.text(.035,.91,'등방성 탄성계수 1.5GPa 가정 / 변형은 15배 강조 · 출력물 강도 합격 판정이 아닌 수치 비교',fontsize=12,color='#456')
labels=['손바닥에 수직인 하중','손가락 길이 방향 하중','옆 방향 하중'];maxval=max(x['maximum_displacement_mm'] for x in res['cases']);norm=colors.Normalize(0,maxval)
for k,rr in enumerate(res['cases']):
 ax=fig.add_subplot(1,3,k+1,projection='3d');ax.set_facecolor('#f0f4fa');u=fields['u_'+str(k)];mag=np.linalg.norm(u,axis=1);deformed=xyz+15*u;vals=mag[tris].mean(1);poly=Poly3DCollection(deformed[tris],facecolors=plt.cm.viridis(norm(vals)),edgecolors='none',rasterized=True);ax.add_collection3d(poly)
 fixed=xyz[I['support']];ax.scatter(fixed[::30,0],fixed[::30,1],fixed[::30,2],s=8,c='#d54233',depthshade=False)
 ax.set(xlim=(-125,125),ylim=(-5,205),zlim=(-85,65));ax.set_box_aspect((250,210,150),zoom=1.20);ax.view_init(25,-62);ax.set_axis_off();ax.set_title(labels[k]+'\n최대 '+format(rr['maximum_displacement_mm'],'.2f')+'mm',fontsize=15,pad=0)
cax=fig.add_axes([.88,.17,.018,.58]);fig.colorbar(plt.cm.ScalarMappable(norm=norm,cmap='viridis'),cax=cax,label='변위 크기 (mm)');fig.text(.035,.075,'손목 고정 구멍 4곳 구속 · 손가락 질량/모멘트와 손바닥 자체 무게 포함 · 메시 수렴·적층 방향·체결 접촉은 추가 검증 필요',fontsize=11,color='#456');fig.subplots_adjust(left=.01,right=.87,bottom=.08,top=.82,wspace=-.04);fig.savefig(F/'palm_deformation.png',dpi=150,facecolor=fig.get_facecolor());plt.close(fig)
print('Rendered',len(tris),'surface triangles',flush=True)
