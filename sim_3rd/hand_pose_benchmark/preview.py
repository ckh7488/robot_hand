"""Render a reproducible visual evidence sheet without altering model outputs."""
import argparse,json,pathlib,re
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from io import BytesIO
import base64

ROOT=pathlib.Path(r'E:\hand_recog\vision-results\20261026\report')
FONTPATH=r'C:\Windows\Fonts\malgun.ttf'
def font(n):return ImageFont.truetype(FONTPATH,n)

def render(root=ROOT):
    text=(root/'hand_pose_review.html').read_text(encoding='utf8')
    d=json.loads(text.split('<script>const D=',1)[1].split(';</script>',1)[0])
    cases=d['cases'];selected=[]
    selected.append(next(c for c in cases if 'hanco/inspection/sample-03' in c['id']))
    selected.append(max((c for c in cases if c['dataset']=='FreiHAND'),key=lambda c:sum(m['metrics'].get('different_finger_pairs',0) for m in c['models'].values())))
    selected.append(next(c for c in cases if 'interhand/inspection/train-H-503834' in c['id']))
    out=Image.new('RGB',(1680,2060),'#10151d');dr=ImageDraw.Draw(out)
    dr.text((35,24),'실제 추론 · 관절과 표면 교차 비교',font=font(33),fill='#eef3fb')
    dr.text((35,74),'녹색: 정답  ·  노랑: 예측  ·  빨강: 손가락 사이 교차  ·  주황: 그 외 표면 접힘',font=font(19),fill='#bdcbdc')
    dr.text((35,104),'정답은 손목 위치만 정렬했습니다. 회전·크기를 정답에 맞추지 않았습니다.',font=font(17),fill='#91a6ba')
    colors=['#75e1ac','#ffd079','#ffd079','#ffd079'];ids=['gt','mediapipe','wilor','hamer']
    def skel(draw,pts,valid,color,width=3):
        for a,b in d['edges']:
            if valid is not None and not(valid[a] and valid[b]):continue
            draw.line([tuple(pts[a,:2]),tuple(pts[b,:2])],fill=color,width=width)
        for i,p in enumerate(pts):
            if valid is not None and not valid[i]:continue
            draw.ellipse((p[0]-2,p[1]-2,p[0]+2,p[1]+2),fill=color)
    for row,c in enumerate(selected):
        top=150+row*625
        dr.text((35,top),c['id'],font=font(17),fill='#cfdeef')
        img=Image.open(BytesIO(base64.b64decode(c['image'].split(',')[1]))).convert('RGB')
        yaw=.6;pitch=-.12
        rot=np.array([[np.cos(yaw),0,np.sin(yaw)],[-np.sin(pitch)*np.sin(yaw),np.cos(pitch),np.sin(pitch)*np.cos(yaw)],[-np.cos(pitch)*np.sin(yaw),-np.sin(pitch),np.cos(pitch)*np.cos(yaw)]])
        allq=[]
        for key in ids:
            hs=c['gt'] if key=='gt' else c['models'][key]['hands']
            for j,h in enumerate(hs):
                q=np.asarray(h.get('vertices',h['xyz']))-np.asarray(h['xyz'])[0];q[:,0]+=(j-(len(hs)-1)/2)*.16;allq.append(q@rot.T)
        aq=np.concatenate(allq);center=(aq[:,:2].min(0)+aq[:,:2].max(0))/2
        viewscale=min(350/max(np.ptp(aq[:,0]),.01),205/max(np.ptp(aq[:,1]),.01))
        for col,key in enumerate(ids):
            x=35+col*410;y=top+35;w=380
            dr.rounded_rectangle((x,y,x+w,y+562),radius=8,fill='#1b2635')
            title={'gt':'원본 + 정답','mediapipe':'MediaPipe','wilor':'WiLoR-mini','hamer':'HaMeR'}[key]
            dr.text((x+12,y+8),title,font=font(22),fill='#eef3fb')
            im=img.copy();im.thumbnail((w,225));ox=x+(w-im.width)/2;oy=y+45+(225-im.height)/2;out.paste(im,(int(ox),int(oy)))
            scale=im.width/c['image_size'][0]
            for h in c['gt']:skel(dr,np.asarray(h['uv'])*scale+np.array([ox,oy]),h['valid'],'#75e1ac',2)
            hands=c['gt'] if key=='gt' else c['models'][key]['hands']
            if key!='gt':
                for h in hands:skel(dr,np.asarray(h['uv'])*scale+np.array([ox,oy]),None,'#ffd079',2)
            origin=np.array([x+w/2,y+389])
            def transform(p,h,j):
                q=np.asarray(p)-np.asarray(h['xyz'])[0];q[:,0]+=(j-(len(hands)-1)/2)*.16
                q=q@rot.T;q[:,:2]=(q[:,:2]-center)*viewscale+origin;return q
            for j,h in enumerate(hands):
                if 'vertices' in h:
                    v=transform(h['vertices'],h,j);faces=np.asarray(d['faces'][key]);g=h.get('geometry',{});red=set(g.get('different_finger_faces',[]));orange=set(g.get('mesh_crossing_faces',[]))
                    for fi in np.argsort(v[faces,:,][...,2].mean(axis=1))[::-1]:
                        f=faces[fi];dr.polygon([tuple(p[:2]) for p in v[f]],fill='#ef4968' if fi in red else '#d18e42' if fi in orange else '#356d87')
                skel(dr,transform(h['xyz'],h,j),h.get('valid'),colors[col])
            if key!='gt':
                m=c['models'][key]['metrics'];a=m.get('matches',[])
                for match in a:
                    g=c['gt'][match['gt']];skel(dr,transform(g['xyz'],g,match['pred']),g['valid'],'#75e1ac',1)
                err=np.mean([z['mpjpe_mm'] for z in a]) if a else None
                info=f"관절 오차 {err:.1f} mm" if err is not None else '정답과 매칭된 손 없음'
                dr.text((x+12,y+505),info,font=font(18),fill='#d9e6f4')
                dr.text((x+12,y+535),f"손가락 사이 교차: {m.get('different_finger_hands','측정 불가')}",font=font(16),fill='#e7b3bb')
            else:dr.text((x+12,y+518),'데이터셋 제공 3D 관절',font=font(18),fill='#75e1ac')
    out.save(root/'preview.png');print(root/'preview.png')

if __name__=='__main__':render()
