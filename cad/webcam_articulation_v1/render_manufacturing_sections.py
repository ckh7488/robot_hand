"""Show exact saved-BRep cross-sections of three assembly blockers."""
import os
import sys
os.environ['MPLBACKEND']='Agg'
sys.path.append('D:/Codex-work/robot-hand-workspace/work/hand_follow_deps')
import numpy as np
import cadquery as cq
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Patch
from matplotlib.collections import PolyCollection
from manufacturing_audit import load,OUT,save


def draw_section(ax,section,axis,color):
    if not section.Faces():return
    vertices,triangles=section.tessellate(.02,.1)
    if not triangles:return
    coordinates=np.array([v.toTuple() for v in vertices])
    points=coordinates[:,[1,2] if axis==0 else [0,1]]
    ax.add_collection(PolyCollection(points[np.array(triangles)],facecolors=color,
        edgecolors='none',antialiased=False))


def main():
    design,scene,parts,hashes=load()
    root=np.array(design['fingers'][0]['root'])
    local={n:s.translate(tuple(-root)) for n,s in parts.items() if n.startswith('index_')}
    font_manager.fontManager.addfont('C:/Windows/Fonts/malgun.ttf')
    plt.rcParams.update({'font.family':'Malgun Gothic','axes.unicode_minus':False})
    palette=['#9fb4c6','#3294c9','#e4b345','#54a885','#6549a3']
    configurations=[
        dict(title='1. 지지살이 핀 구멍을 막음',subtitle='핀과 지지대 겹침 18.85 mm³ / 한쪽',axis=0,offset=13,
            extent=(-10,10,-11,10),names=['index_root_fork_13','index_joint_pin_0'],labels=['관절 지지대','관절 핀']),
        dict(title='2. 회전 받침과 굽힘 마디가 충돌',subtitle='편 손에서도 받침–마디 겹침 약631 mm³',axis=0,offset=0,
            extent=(-19,25,-12,15),names=['index_spread_housing','index_segment1_segment_shell','index_segment1_proximal_bushing','index_spread_ring','index_joint_pin_0'],
            labels=['고정 받침','움직이는 마디','부싱','회전 링','핀']),
        dict(title='3. 회전 링과 양쪽 지지대가 떨어짐',subtitle='양쪽 모두 1.00 mm 틈 · 연결부 없음',axis=2,offset=-3.5,
            extent=(-18,18,-13,13),names=['index_root_fork_-13','index_spread_ring','index_root_fork_13'],labels=['왼쪽 지지대','회전 링','오른쪽 지지대'])]
    figure,axes=plt.subplots(1,3,figsize=(18,7),dpi=140)
    figure.patch.set_facecolor('#f5f7fa')
    figure.suptitle('제작성 검토: 현재 CAD는 그대로 조립할 수 없습니다',fontsize=22,y=.97,fontweight='bold')
    for ax,config in zip(axes,configurations):
        xmin,xmax,ymin,ymax=config['extent']
        origin=(config['offset'],0,0) if config['axis']==0 else (0,0,config['offset'])
        normal=(1,0,0) if config['axis']==0 else (0,0,1)
        plane=cq.Face.makePlane(basePnt=origin,dir=normal)
        handles=[];sections=[]
        for i,(name,label) in enumerate(zip(config['names'],config['labels'])):
            section=local[name].intersect(plane);sections.append(section)
            draw_section(ax,section,config['axis'],palette[i])
            handles.append(Patch(color=palette[i],label=label))
        for i,a in enumerate(sections):
            for b in sections[i+1:]:
                if a.Faces() and b.Faces():draw_section(ax,a.intersect(b),config['axis'],'#e94b45')
        ax.set_xlim(xmin,xmax);ax.set_ylim(ymin,ymax)
        ax.set_aspect('equal');ax.set_title(config['title']+'\n'+config['subtitle'],fontsize=12,pad=14)
        ax.set_xlabel('Y [mm]' if config['axis']==0 else 'X [mm]')
        ax.set_ylabel('Z [mm]' if config['axis']==0 else 'Y [mm]')
        ax.legend(handles=handles,loc='upper center',bbox_to_anchor=(.5,-.15),ncol=2,frameon=False,fontsize=9)
        ax.grid(alpha=.14)
        if config['axis']==2:
            ax.annotate('1 mm',xy=(11,0),xytext=(15,-10),fontsize=10,ha='center',
                arrowprops=dict(arrowstyle='->',color='#202b3c'))
            ax.annotate('1 mm',xy=(-11,0),xytext=(-15,-10),fontsize=10,ha='center',
                arrowprops=dict(arrowstyle='->',color='#202b3c'))
        elif config['offset']==13:
            ax.annotate('핀 아래쪽 막힘',xy=(0,-1),xytext=(-8,6),fontsize=10,
                arrowprops=dict(arrowstyle='->',color='#202b3c'))
        print(config['title'],flush=True)
    figure.text(.5,.055,'빨강 = 서로 다른 부품이 같은 공간을 차지함  |  실제 STEP 평면 절단 · 검지 뿌리 기준 좌표 · 표시용 삼각화',ha='center',fontsize=11)
    figure.subplots_adjust(top=.79,bottom=.24,left=.05,right=.98,wspace=.30)
    OUT.mkdir(parents=True,exist_ok=True)
    figure.savefig(OUT/'assembly_blockers.png',facecolor=figure.get_facecolor())
    plt.close(figure)
    save('section_inputs.json',dict(input_sha256=hashes,root_mm=root.tolist(),sections=configurations,tessellation_tolerance_mm=.02,
        method='BRep common with a plane; planar intersections shown in red; volumes from separate solid Boolean intersections'))


if __name__=='__main__':main()
