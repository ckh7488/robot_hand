"""Four existing verified hand animations, together and individually.

This packages saved CAD renderings without inventing motion, adding hardware,
or rerunning collision studies. Playback duration is explanatory, not speed.
"""
import hashlib
import json
from pathlib import Path
import shutil
import sys

from PIL import Image, ImageDraw, ImageFont
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True
CLIPS=[
    dict(id='01_index',title='1 · 검지만 펴기',file='cad/functional_motion_v3/findings/count_one_motion.gif',
         audit='cad/functional_motion_v3/reviews/motion_fix.json',samples=79,note='엄지 자세와 접는 순서를 교정한 왕복 경로'),
    dict(id='02_cross',title='2 · 검지·중지 교차',file='cad/posture_correction_v1/images/cross_motion.gif',
         audit='cad/posture_correction_v1/reviews/audit.json',samples=37,note='중지가 앞에 오는 방향의 교차와 풀기'),
    dict(id='03_ball',title='3 · 고정된 공에 손끝 접근·풀기',file='cad/object_pinch_v2/images/approach_release.gif',
         audit='cad/object_pinch_v2/reviews/audit.json',samples=33,note='지지된25mm 공의 기하 접촉,파지력/들어 올리기 미검증'),
    dict(id='04_combined',title='4 · 손끝 접근 + 중지 펴기',file='cad/combined_pose_v1/images/example.gif',
         audit='cad/combined_pose_v1/reviews/audit.json',samples=45,note='약지·새끼 접기를 함께 조합한 동작')]


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def read(path):return json.loads(path.read_text(encoding='utf-8'))


def main():
    out=HERE/'previews'
    if out.exists():raise FileExistsError('Preserve earlier motion previews')
    manifest=read(ROOT/'checkpoints/ASSEMBLY012.manifest.json')
    source={HERE.relative_to(ROOT).as_posix()+'/motion_previews.py':sha(HERE/'motion_previews.py')}
    source['checkpoints/ASSEMBLY012.manifest.json']=sha(ROOT/'checkpoints/ASSEMBLY012.manifest.json')
    out.mkdir()
    clips=[];framesets=[]
    for clip in CLIPS:
        for rel in [clip['file'],clip['audit']]:
            digest=sha(ROOT/rel)
            if manifest['files'][rel]['sha256']!=digest:raise ValueError('Changed old motion evidence')
            source[rel]=digest
        audit=read(ROOT/clip['audit'])
        if not audit['completed'] or not audit['input_hash_guard'] or audit['geometry_changed']:
            raise ValueError('Incomplete or changed hand-motion evidence')
        target=out/(clip['id']+'.gif');shutil.copy2(ROOT/clip['file'],target)
        frames=[]
        with Image.open(target) as image:
            count=image.n_frames
            for i in range(count):
                image.seek(i);frames.append(image.convert('RGB').copy())
        frames[0].save(out/(clip['id']+'.png'))
        framesets.append(frames)
        clips.append(dict(clip,output=target.relative_to(ROOT).as_posix(),sha256=sha(target),
                          rendered_frames=len(frames),native_collision_reexecuted=False,hand_parts=364))
    fontpath=Path('C:/Windows/Fonts/malgun.ttf')
    if not fontpath.is_file():raise FileNotFoundError('Korean font')
    titlefont=ImageFont.truetype(str(fontpath),30)
    labelfont=ImageFont.truetype(str(fontpath),23)
    smallfont=ImageFont.truetype(str(fontpath),18)
    width,height=1280,1640
    duration=140
    count=max(len(frames) for frames in framesets)
    rendered=[]
    choices=[]
    for i in range(count):
        canvas=Image.new('RGB',(width,height),(243,246,250));draw=ImageDraw.Draw(canvas)
        draw.text((22,12),'같은 손 CAD · 네 가지 움직임',font=titlefont,fill=(24,39,58))
        draw.text((22,55),'손364부품의 기존 검사 동작 / 배선·모터의 실제 동작은 미검증 / 재생속도는 설명용',font=smallfont,fill=(66,80,98))
        indices=[]
        for k,(clip,frames) in enumerate(zip(CLIPS,framesets)):
            x=(k%2)*640;y=92+(k//2)*750
            draw.text((x+14,y),clip['title'],font=labelfont,fill=(24,39,58))
            index=round(i*(len(frames)-1)/(count-1));indices.append(index)
            frame=frames[index].copy();frame.thumbnail((620,690),Image.Resampling.LANCZOS)
            canvas.paste(frame,(x+(640-frame.width)//2,y+36+(690-frame.height)//2))
        draw.text((22,1600),'공 동작은 고정된 공의 접근·접촉 경로입니다. 잡아서 유지하거나 들어 올린다는 뜻은 아닙니다.',font=smallfont,fill=(66,80,98))
        rendered.append(canvas);choices.append(indices)
    palette=rendered[0].quantize(colors=256,method=Image.Quantize.MEDIANCUT)
    quantized=[frame.quantize(palette=palette,dither=Image.Dither.NONE) for frame in rendered]
    movie=out/'four_motions.gif'
    quantized[0].save(movie,save_all=True,append_images=quantized[1:],duration=duration,loop=0,disposal=2,optimize=False)
    errors=[]
    with Image.open(movie) as image:
        if image.size!=(width,height) or image.n_frames!=count or image.info.get('loop')!=0:
            raise ValueError('Incorrect montage dimensions/frames/loop')
        for i in range(count):
            image.seek(i)
            error=float(np.abs(np.asarray(image.convert('RGB'),dtype=np.float32)-np.asarray(rendered[i],dtype=np.float32)).mean())
            if error>3.:raise ValueError('Unexpected encoded-frame distortion')
            errors.append(error)
    for index,label in [(0,'start'),(count//2,'middle'),(count-1,'end')]:
        rendered[index].save(out/('qa_'+label+'.png'))
    outputs={path.relative_to(ROOT).as_posix():sha(path) for path in out.iterdir() if path.is_file()}
    report=dict(completed=True,input_sha256=source,clips=clips,montage=dict(path=movie.relative_to(ROOT).as_posix(),
        frames=count,duration_per_frame_ms=duration,source_frame_indices=choices,frame_pixel_MAE=errors,
        source_time_resampled_for_comparison=True,physical_speed_claimed=False),outputs=outputs,
        geometry_changed=False,new_motion_created=False,collision_reexecuted=False,
        scope='Existing hand-only CAD GIFs packaged for viewing. No motor/wiring animation or new collision,grasp force,unsupported hold,electrical or physical approval.')
    if not all(sha(ROOT/rel)==digest for rel,digest in source.items()):raise ValueError('Changed preview inputs')
    (out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    (out/'README.md').write_text('''# 네 가지 손동작

![네 동작 비교](four_motions.gif)

[검지만 펴기](01_index.gif) · [검지·중지 교차](02_cross.gif) · [고정 공 접근·풀기](03_ball.gif) · [복합 손모양](04_combined.gif)

현재와 같은364부품 손에서 이미 검사한 경로의 CAD 그림을 모았다. 개별 원본은 바이트 그대로 보존했고,위 비교본만 재생 주기를 맞췄다. 그려진 프레임 수와 실제 간섭 검사 표본 수는 다르며 상세는 report.json에 있다. 이번에 손동작 간섭 검사를 다시 실행하거나 새로운 동작을 만든 것은 아니다.

모터·배선·케이블이 함께 움직이는 완성품 동작 영상은 아직 아니다. 재생 시간은 실물 속도가 아니다. 공은25mm 고정 지지 물체이며 접촉 기하를 보여준다. 파지력·지지 없는 유지·들어 올리기와 전체 실물 성능은 미검증이다.
''',encoding='utf-8')
    print(json.dumps(dict(completed=True,clips=len(clips),frames=count,bytes=movie.stat().st_size,max_frame_MAE=max(errors))),flush=True)


if __name__=='__main__':main()
