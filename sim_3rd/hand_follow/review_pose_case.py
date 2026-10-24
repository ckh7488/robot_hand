"""Render untouched MediaPipe 3D points from a user-marked case; no CAD fit.

python review_pose_case.py --case <case-folder> --output <separate-folder>
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def render_case(case_path, output):
    path=Path(case_path)
    if path.is_dir():path=path/'case.json'
    case=json.loads(path.read_text(encoding='utf-8'))
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    result=dict(case_id=case['case_id'],seq=case['seq'],case_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        detected=case['detection'] is not None,scope='Original detector points, no CAD, angle calibration, smoothing or bone normalization.')
    if not result['detected']:
        (output/'review.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
        return result
    p=np.asarray(case['detection']['world'],float)
    if p.shape!=(21,3) or not np.isfinite(p).all():raise ValueError('Invalid raw world landmarks')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    from matplotlib.lines import Line2D
    font=Path('C:/Windows/Fonts/malgun.ttf')
    if font.exists():matplotlib.rcParams['font.family']=FontProperties(fname=str(font)).get_name()
    matplotlib.rcParams['axes.unicode_minus']=False
    # Rigid display rotation, determinant +1. All distances/angles are preserved.
    xyz=p[:,[0,2,1]]*np.array([1,1,-1])*1000
    xyz-=(xyz.max(0)+xyz.min(0))/2
    radius=max(np.ptp(xyz,axis=0).max()*.55,20)
    groups=[('엄지',[1,2,3,4],'#35c7c1'),('검지',[5,6,7,8],'#ff9e52'),
        ('중지',[9,10,11,12],'#78d292'),('약지',[13,14,15,16],'#eb8ebb'),('소지',[17,18,19,20],'#b39be4')]
    bg='#141a24';fg='#eef3f8';muted='#93a5b8'
    fig=plt.figure(figsize=(15,5.2),facecolor=bg)
    fig.suptitle(f"{case['case_id']} · MediaPipe 원본 3D",x=.045,y=.975,ha='left',fontsize=20,color=fg)
    for i,(label,azim,elev) in enumerate([('사진 쪽에서',-90,0),('사선에서',-35,15),('옆에서',0,0)]):
        ax=fig.add_subplot(1,3,i+1,projection='3d')
        for a,b in [(0,1),(0,5),(5,9),(9,13),(13,17),(17,0)]:
            ax.plot(*xyz[[a,b]].T,color=muted,alpha=.65,lw=1.7)
        for name,ids,color in groups:
            ax.plot(*xyz[ids].T,color=color,lw=4 if name in ('엄지','검지') else 2.6)
            ax.scatter(*xyz[ids].T,color=color,s=30,depthshade=True)
        ax.view_init(elev=elev,azim=azim);ax.set_proj_type('ortho')
        ax.set_xlim(-radius,radius);ax.set_ylim(-radius/2,radius/2);ax.set_zlim(-radius/2,radius/2)
        # Expand any narrower dimension if the observed hand does not fit it.
        spans=np.maximum(np.ptp(xyz,axis=0)*1.1,[2*radius,radius,radius])
        for setter,span in zip((ax.set_xlim,ax.set_ylim,ax.set_zlim),spans):setter(-span/2,span/2)
        ax.set_box_aspect(spans);ax.set_axis_off();ax.set_facecolor(bg)
        ax.set_title(label,color=fg,fontsize=16,pad=0)
    fig.legend([Line2D([0],[0],color=c,lw=4,marker='o') for _,_,c in groups],
        [n for n,_,_ in groups],loc='lower center',bbox_to_anchor=(.5,.105),ncol=5,frameon=False,labelcolor=fg,fontsize=13)
    fig.text(.045,.06,f"프레임 {case['seq']} · {case['capture_elapsed_s']:.3f}초 · 민트색 엄지 / 주황색 검지",fontsize=12,color=muted)
    fig.text(.045,.025,'21개 원본 관절점 · 길이·각도·깊이를 고치지 않고 보는 방향만 회전',fontsize=11,color=muted)
    fig.subplots_adjust(left=.02,right=.98,top=.83,bottom=.2,wspace=.01)
    fig.savefig(output/'mediapipe_raw_3d.png',dpi=140,facecolor=bg);plt.close(fig)
    result['render']='mediapipe_raw_3d.png'
    (output/'review.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    return result


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--case',required=True,type=Path)
    ap.add_argument('--output',required=True,type=Path)
    a=ap.parse_args()
    print(json.dumps(render_case(a.case,a.output),ensure_ascii=False))
