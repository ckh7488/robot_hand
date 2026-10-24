"""Render actual candidate CAD meshes, with the frozen arm as visual context."""
from pathlib import Path
import json, math, argparse
import numpy as np
import mujoco
from PIL import Image, ImageDraw, ImageFont

D=Path(__file__).resolve().parent
OUT=D/'generated'
S=json.loads((OUT/'scene.json').read_text(encoding='utf-8'))
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output-dir', type=Path, default=OUT)
OUTPUT=parser.parse_args().output_dir
OUTPUT.mkdir(parents=True,exist_ok=True)
BG='#eff4fa'
FONT=ImageFont.truetype('C:/Windows/Fonts/malgun.ttf',34)
SMALL=ImageFont.truetype('C:/Windows/Fonts/malgun.ttf',24)
TINY=ImageFont.truetype('C:/Windows/Fonts/malgun.ttf',20)

def model(whole):
    items=S['hand']+(S['arm_reference'] if whole else [])
    assets=[]; geoms=[]
    for i,it in enumerate(items):
        path=Path(it['mesh'])
        if not path.is_absolute():path=D/path
        scale='.001 .001 .001' if it['mesh_units']=='mm' else '1 1 1'
        assets.append(f'<mesh name="part{i}" file="{path.as_posix()}" scale="{scale}" maxhullvert="64"/>')
        color=' '.join(map(str,it['color']))
        geoms.append(f'<geom type="mesh" mesh="part{i}" rgba="{color} 1" contype="0" conaffinity="0"/>')
    xml='<mujoco><visual><global offwidth="1600" offheight="1700"/><quality shadowsize="4096"/><headlight ambient=".6 .6 .6" diffuse=".5 .5 .5" specular=".12 .12 .12"/></visual><asset><texture type="skybox" builtin="flat" rgb1=".94 .96 .98" rgb2=".94 .96 .98" width="256" height="1536"/>'+''.join(assets)+'</asset><worldbody><light pos=".4 .3 .9" diffuse=".6 .6 .6"/>'+''.join(geoms)+'</worldbody></mujoco>'
    m=mujoco.MjModel.from_xml_string(xml);data=mujoco.MjData(m);mujoco.mj_forward(m,data)
    points=[]
    for g in range(m.ngeom):
        j=m.geom_dataid[g];v=m.mesh_vert[m.mesh_vertadr[j]:m.mesh_vertadr[j]+m.mesh_vertnum[j]]
        points.append(v@data.geom_xmat[g].reshape(3,3).T+data.geom_xpos[g])
    points=np.concatenate(points)
    return m,data,points.min(0),points.max(0)

def sheet(whole):
    m,data,lo,hi=model(whole)
    ren=mujoco.Renderer(m,height=1500,width=1000)
    canvas=Image.new('RGB',(2400,1380),BG);dr=ImageDraw.Draw(canvas)
    title='손·손목·전완 — 새 비율의 외형 CAD' if whole else '손바닥을 줄여 다시 잡은 손의 비율'
    dr.text((32,19),title,font=FONT,fill='#203449')
    dr.text((32,70),'손바닥 목표 180 × 150 mm  /  중지 154 mm 유지  /  입체적인 중공 마디와 별도 손끝',font=SMALL,fill='#435870')
    views=[('정면',90,-90),('사선',125,-30),('측면',180,0)]
    for i,(label,az,el) in enumerate(views):
        cam=mujoco.MjvCamera();cam.lookat[:]=(lo+hi)/2;cam.distance=max(hi-lo)*2.5
        cam.azimuth=az;cam.elevation=el
        ren.update_scene(data,cam)
        for eye in ren.scene.camera:
            fw=np.array(eye.forward);up=np.array([0.,1.,0.]);up-=fw*np.dot(up,fw);eye.up[:]=up/np.linalg.norm(up)
        im=Image.fromarray(ren.render())
        ren.enable_segmentation_rendering();seg=ren.render();ren.disable_segmentation_rendering()
        yy,xx=np.where(seg[:,:,0]>=0)
        if len(xx):im=im.crop((max(0,int(xx.min())-26),max(0,int(yy.min())-26),min(im.width,int(xx.max())+27),min(im.height,int(yy.max())+27)))
        k=min(760/im.width,1110/im.height)
        im=im.resize((round(im.width*k),round(im.height*k)),Image.Resampling.LANCZOS)
        canvas.paste(im,(i*800+(800-im.width)//2,155+(1120-im.height)//2))
        dr.text((i*800+32,120),label,font=SMALL,fill='#203449')
    dr.line((32,1304,2368,1304),fill='#d5dfe9',width=2)
    note='새 손은 모터 미배치 · 팔은 V9 참고 형상 · 줄 경로 / 간섭 / 100 ms 성능 검증 전' if whole else '외형 검토 후보 · 손 내부 모터 미배치 · 줄 경로 / 간섭 / 100 ms 추종 성능은 아직 검증 전'
    dr.text((32,1322),note,font=TINY,fill='#435870')
    filename='robot_hand_new_proportions_whole.png' if whole else 'robot_hand_new_proportions_hand.png'
    canvas.save(OUTPUT/filename)
    canvas.save(OUT/filename)
    ren.close()
    print(json.dumps({'image':str(OUTPUT/filename),'whole':whole,'rendered_meshes':m.ngeom,'mesh_bounds_m':[lo.tolist(),hi.tolist()]}),flush=True)

if __name__=='__main__':
    sheet(False)
    sheet(True)
