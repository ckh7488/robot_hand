"""Orthographic z-buffer drawing of WiLoR's unmodified predicted MANO surface."""
import numpy as np
from PIL import Image,ImageDraw
from render_comparison import BG,PANEL,TEXT,MUTED,font,rigid_view


def mesh_view(joints,vertices,faces,oblique=False,size=(570,600)):
    p=np.asarray(vertices,dtype=float)
    _,rotation=rigid_view(joints,oblique)
    q=(p-np.asarray(joints)[0]) @ rotation
    scale=min(size[0]/.25,size[1]/.25)
    xy=(q[:,:2]-(q[:,:2].min(0)+q[:,:2].max(0))/2)*[scale,-scale]+np.array(size)/2
    pixels=np.full((size[1],size[0],3),PANEL,dtype=np.uint8)
    depth=np.full(size[::-1],-np.inf)
    light=np.array([-.3,.45,1.]);light/=np.linalg.norm(light)
    for face in np.asarray(faces,dtype=int):
        tri=xy[face];xyz=q[face]
        x0,y0=np.maximum(np.floor(tri.min(0)).astype(int),[0,0])
        x1,y1=np.minimum(np.ceil(tri.max(0)).astype(int),np.array(size)-1)
        if x1<x0 or y1<y0:continue
        a,b,c=tri
        den=(b[1]-c[1])*(a[0]-c[0])+(c[0]-b[0])*(a[1]-c[1])
        if abs(den)<1e-10:continue
        xx,yy=np.meshgrid(np.arange(x0,x1+1)+.5,np.arange(y0,y1+1)+.5)
        u=((b[1]-c[1])*(xx-c[0])+(c[0]-b[0])*(yy-c[1]))/den
        v=((c[1]-a[1])*(xx-c[0])+(a[0]-c[0])*(yy-c[1]))/den
        w=1-u-v
        z=u*xyz[0,2]+v*xyz[1,2]+w*xyz[2,2]
        target=depth[y0:y1+1,x0:x1+1]
        mask=(u>=0)&(v>=0)&(w>=0)&(z>target)
        normal=np.cross(xyz[1]-xyz[0],xyz[2]-xyz[0])
        normal/=max(np.linalg.norm(normal),1e-12)
        shade=.32+.68*abs(float(normal@light))
        target[mask]=z[mask]
        pixels[y0:y1+1,x0:x1+1][mask]=np.array([105,177,206])*shade
    return Image.fromarray(pixels)


def render_mesh_case(case,bgr,result,faces,output):
    if result['status']!='tracked':return
    pred=result['result']['wilor_preds']
    joints=np.asarray(pred['pred_keypoints_3d'])[0]
    vertices=np.asarray(pred['pred_vertices'])[0]
    image=Image.new('RGB',(1800,760),BG);d=ImageDraw.Draw(image)
    d.text((22,20),f"장면 {case['number']} · WiLoR가 추정한 손 표면",font=font(30),fill=TEXT)
    for x,title in [(20,'입력 사진'),(615,'WiLoR · 정면'),(1210,'WiLoR · 사선')]:
        d.text((x,85),title,font=font(24),fill=TEXT)
    image.paste(Image.fromarray(bgr[:,:,::-1]).resize((580,435)),(20,130))
    image.paste(mesh_view(joints,vertices,faces),(615,130))
    image.paste(mesh_view(joints,vertices,faces,True),(1210,130))
    d.text((25,595),'모델이 낸 778개 정점을 그대로 표시',font=font(22),fill=MUTED)
    d.text((25,633),'손 모양 보정 / 충돌 수정 없음',font=font(22),fill=MUTED)
    d.text((25,671),'표면이 자연스러워도 깊이가 맞는지는 별도 확인',font=font(20),fill=MUTED)
    image.save(output/'mesh.png')
