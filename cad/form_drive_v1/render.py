"""Render the real candidate meshes; cover removal is only a viewing operation."""
from pathlib import Path
import argparse
import hashlib
import json
import numpy as np
import mujoco
from PIL import Image, ImageDraw, ImageFont

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
FONT='C:/Windows/Fonts/malgun.ttf'

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,default=HERE/'generated');a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    form=ROOT/'cad/form_candidate_v1';f=json.loads((form/'generated/scene.json').read_text())
    packdir=HERE/'generated/packaging';pack=json.loads((packdir/'scene.json').read_text())
    rows=pack['parts'] if isinstance(pack,dict) else pack
    sources={Path(__file__).resolve(),form/'generated/scene.json',packdir/'scene.json'}
    sources.update((form/it['mesh']).resolve() for it in f['hand']+f['arm_reference'])
    sources.update((packdir/it['mesh']).resolve() for it in rows)
    hashes=lambda:{x.relative_to(ROOT).as_posix():hashlib.sha256(x.read_bytes()).hexdigest() for x in sorted(sources)}
    before=hashes();outputs=[]
    for cut in [False,True]:
        items=[]
        for it in f['hand']+f['arm_reference']:
            if cut and (it['name'].startswith('Cover_') or it['name'] in ['Palm_shaped_front_cover','Palm_tapered_chassis']):continue
            items.append(dict(it,path=(form/it['mesh']).resolve()))
        for it in rows:
            items.append(dict(it,path=(packdir/it['mesh']).resolve(),mesh_units='mm'))
        assets=[];geoms=[]
        for i,it in enumerate(items):
            scale='.001 .001 .001' if it['mesh_units']=='mm' else '1 1 1'
            assets.append(f'<mesh name="p{i}" file="{it["path"].as_posix()}" scale="{scale}" maxhullvert="64"/>')
            rgb=' '.join(str(v) for v in it['color'][:3]);geoms.append(f'<geom type="mesh" mesh="p{i}" rgba="{rgb} 1" contype="0" conaffinity="0"/>')
        xml='<mujoco><visual><global offwidth="1400" offheight="1600"/><quality shadowsize="2048"/><headlight ambient=".6 .6 .6" diffuse=".5 .5 .5" specular=".12 .12 .12"/></visual><asset><texture type="skybox" builtin="flat" rgb1=".94 .96 .98" rgb2=".94 .96 .98" width="256" height="1536"/>'+''.join(assets)+'</asset><worldbody><light pos=".4 .3 .9" diffuse=".6 .6 .6"/>'+''.join(geoms)+'</worldbody></mujoco>'
        m=mujoco.MjModel.from_xml_string(xml);data=mujoco.MjData(m);mujoco.mj_forward(m,data);ren=mujoco.Renderer(m,height=1400,width=1000)
        canvas=Image.new('RGB',(2400,1380),'#eff4fa');draw=ImageDraw.Draw(canvas)
        title='구동 배치 내부 보기 — 손바닥 6 · 손목 4 · 전완 10' if cut else '새 손 형상을 유지한 20개 모터 배치 후보'
        draw.text((32,20),title,font=ImageFont.truetype(FONT,34),fill='#203449')
        draw.text((32,72),'실제 크기의 모터 · 원위 마디에 모터 없음 · 뒤쪽 랙 추가 · 체결과 줄 경로는 설계 중',font=ImageFont.truetype(FONT,23),fill='#435870')
        views=[('정면',90,-90),('뒤쪽 사선',125,30),('측면',180,0)]
        for i,(label,az,el) in enumerate(views):
            cam=mujoco.MjvCamera();cam.lookat[:]=[0,-.04,-.015];cam.distance=1.7;cam.azimuth=az;cam.elevation=el
            ren.update_scene(data,cam)
            for eye in ren.scene.camera:
                fw=np.array(eye.forward);up=np.array([0.,1.,0.]);up-=fw*np.dot(up,fw);eye.up[:]=up/np.linalg.norm(up)
            pixels=ren.render();im=Image.fromarray(pixels)
            # RGB foreground bounds avoid the renderer's segmentation-ID overflow
            # on this large component assembly; this affects only page cropping.
            border=np.concatenate([pixels[0],pixels[-1],pixels[:,0],pixels[:,-1]])
            background=np.median(border,axis=0)
            yy,xx=np.where(np.max(np.abs(pixels.astype(float)-background),axis=2)>12)
            if len(xx):im=im.crop((max(0,int(xx.min())-20),max(0,int(yy.min())-20),min(im.width,int(xx.max())+21),min(im.height,int(yy.max())+21)))
            k=min(760/im.width,1110/im.height);im=im.resize((round(im.width*k),round(im.height*k)),Image.Resampling.LANCZOS)
            canvas.paste(im,(i*800+(800-im.width)//2,160+(1110-im.height)//2));draw.text((i*800+32,122),label,font=ImageFont.truetype(FONT,24),fill='#203449')
        draw.line((32,1304,2368,1304),fill='#d5dfe9',width=2)
        note='내부 확인을 위해 손바닥 외피를 숨긴 화면 · 제조용 도면 아님' if cut else '배치 검토용 CAD · 전 가동 범위 / 조립성 / 로봇 단독 100ms 성능은 검증 전'
        draw.text((32,1322),note,font=ImageFont.truetype(FONT,22),fill='#435870')
        name='robot_hand_drive001_cutaway.png' if cut else 'robot_hand_drive001_whole.png'
        canvas.save(a.output/name);outputs.append(name);ren.close();print(str(a.output/name),flush=True)
    after=hashes()
    report=dict(status='Actual neutral CAD mesh render; no geometric or physical performance acceptance',
        inputs_unchanged=before==after,source_sha256=before,
        output_sha256={name:hashlib.sha256((a.output/name).read_bytes()).hexdigest() for name in outputs},
        mujoco_version=mujoco.__version__,font_file=FONT)
    (a.output/'render_review.json').write_text(json.dumps(report,indent=2)+'\n')
    if before!=after:raise RuntimeError('CAD meshes changed during render')

if __name__=='__main__':main()
