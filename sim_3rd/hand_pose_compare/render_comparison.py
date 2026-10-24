"""Diagnostic plots, using only rigid display transforms and a common metric scale."""
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageFont

BG = (18,24,33)
PANEL = (24,32,44)
TEXT = (229,237,245)
MUTED = (148,167,185)
COLORS = [(53,199,193),(255,158,82),(120,210,146),(235,142,187),(179,155,228)]
PALM = [(0,1),(0,5),(5,9),(9,13),(13,17),(17,0)]
EDGES = [(a,b,(112,133,153),2) for a,b in PALM]
for k,start in enumerate([1,5,9,13,17]):
    EDGES.extend((i,i+1,COLORS[k],5) for i in range(start,start+3))


def font(size):
    return ImageFont.truetype('C:/Windows/Fonts/malgun.ttf', size)


def rigid_view(world, oblique=False):
    p = np.asarray(world,dtype=float)
    if p.shape != (21,3) or not np.isfinite(p).all():
        raise ValueError('Expected 21 finite 3D points')
    y = p[9]-p[0]
    if np.linalg.norm(y) < 1e-6:
        raise ValueError('Degenerate palm height')
    y /= np.linalg.norm(y)
    x = p[5]-p[17]
    x -= (x@y)*y
    if np.linalg.norm(x) < 1e-6:
        raise ValueError('Degenerate palm width')
    x /= np.linalg.norm(x)
    rotation = np.column_stack((x,y,np.cross(x,y)))
    if oblique:
        a,e = np.radians([135,-32])
        eye = -np.array([np.cos(e)*np.cos(a),np.cos(e)*np.sin(a),np.sin(e)])
        up = np.array([0.,1.,0.]); up -= (up@eye)*eye; up /= np.linalg.norm(up)
        rotation = rotation @ np.column_stack((np.cross(up,eye),up,eye))
    return (p-p[0]) @ rotation, rotation


def skeleton(world, oblique=False, size=(570,380)):
    image = Image.new('RGB',size,PANEL)
    d = ImageDraw.Draw(image)
    if world is None:
        d.text((30,160),'손 검출 실패',font=font(28),fill=MUTED)
        return image
    xyz,_ = rigid_view(world,oblique)
    scale = min(size[0]/.25,size[1]/.25)
    center = (xyz[:,:2].max(0)+xyz[:,:2].min(0))/2
    xy = (xyz[:,:2]-center)*[scale,-scale]+np.array(size)/2
    for a,b,color,width in sorted(EDGES,key=lambda v:(xyz[v[0],2]+xyz[v[1],2])/2):
        points = [tuple(xy[a]),tuple(xy[b])]
        d.line(points,fill=PANEL,width=width+4)
        d.line(points,fill=color,width=width)
    for i in np.argsort(xyz[:,2]):
        color = MUTED if i==0 else COLORS[(i-1)//4]
        x,y = xy[i]
        d.ellipse((x-5,y-5,x+5,y+5),fill=color)
    # Identical metric scale across both models; bones are never rescaled.
    d.line([(22,size[1]-24),(22+.03*scale,size[1]-24)],fill=MUTED,width=2)
    d.text((22,size[1]-50),'3 cm',font=font(17),fill=MUTED)
    return image


def overlay(rgb, points):
    image = Image.fromarray(rgb).copy()
    d = ImageDraw.Draw(image)
    if points is None:
        return image
    p = np.asarray(points,dtype=float)
    if p.shape!=(21,2) or not np.isfinite(p).all():
        raise ValueError('Invalid projected points')
    for a,b,color,_ in EDGES:
        d.line([tuple(p[a]),tuple(p[b])],fill=color,width=3)
    for i,(x,y) in enumerate(p):
        color = MUTED if i==0 else COLORS[(i-1)//4]
        d.ellipse((x-4,y-4,x+4,y+4),fill=color)
    return image


def render_case(case,bgr,result,output):
    output = Path(output)
    rgb = bgr[:,:,::-1].copy()
    mp = np.asarray(case['detection']['world'])
    tracked = result['status']=='tracked'
    wp = result['result']['wilor_preds'] if tracked else None
    wi = np.asarray(wp['pred_keypoints_3d'])[0] if tracked else None
    image = Image.new('RGB',(1800,1050),BG)
    d = ImageDraw.Draw(image)
    d.text((26,20),f"저장 장면 {case['number']} · 인식 모델 원본 비교",font=font(32),fill=TEXT)
    titles = ['같은 입력 사진','MediaPipe · 저장된 3D','WiLoR · 새로 추론한 3D']
    for x,title in zip([20,615,1210],titles):
        d.text((x,90),title,font=font(25),fill=TEXT)
    image.paste(Image.fromarray(rgb).resize((580,435)),(20,144))
    for x,world in [(615,mp),(1210,wi)]:
        image.paste(skeleton(world),(x,144))
        d.text((x,551),'사선에서 본 같은 결과',font=font(23),fill=MUTED)
        image.paste(skeleton(world,True),(x,600))
    lines = ['위: 손바닥을 정면으로 정렬','아래: 깊이를 볼 수 있는 사선',
             '길이·관절각·가림 관계 보정 없음','회전·이동만 적용 / 두 모델 동일 축척',
             '정지 사진 비교: 떨림은 아직 미평가']
    for i,line in enumerate(lines):
        d.text((25,610+i*37),line,font=font(22),fill=MUTED)
    for i,name in enumerate(['엄지','검지','중지','약지','소지']):
        d.text((25+(i%3)*165,825+(i//3)*38),name,font=font(23),fill=COLORS[i])
    timing = result.get('median_elapsed_ms',result['elapsed_ms'])
    d.text((25,935),f'WiLoR 처리 {timing:.0f} ms / 장면',font=font(24),fill=TEXT)
    d.text((25,1005),'사진과 두 모델의 결과는 같은 장면입니다. 어느 쪽도 실측 3D 정답은 아닙니다.',font=font(21),fill=MUTED)
    image.save(output/'comparison.png')
    # Projection on the exact same image also exposes detector/handedness failures.
    mp2d = np.asarray(case['detection']['image'],dtype=float)[:,:2]*[rgb.shape[1],rgb.shape[0]]
    wi2d = np.asarray(wp['pred_keypoints_2d'])[0] if tracked else None
    projection = Image.new('RGB',(1800,560),BG)
    pd = ImageDraw.Draw(projection)
    for x,title,points in zip([20,615,1210],['원본','MediaPipe 2D','WiLoR 3D를 사진에 투영'],[None,mp2d,wi2d]):
        pd.text((x,25),title,font=font(25),fill=TEXT)
        projection.paste(overlay(rgb,points).resize((580,435)),(x,80))
    projection.save(output/'projection.png')
