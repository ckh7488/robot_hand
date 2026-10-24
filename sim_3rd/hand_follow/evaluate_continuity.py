"""Reproduce consistency/continuity metrics; private session is optional input.

Replay samples are not human ground truth. Step reduction includes intentional
motion and may reflect holding rejected data. Synthetic latency excludes camera,
recognition, presentation and physical actuators.
"""
from pathlib import Path
import argparse
import hashlib
import json
import time
import numpy as np
from hand_follow.pipeline import PosePipeline
from hand_follow.continuity import ContinuousPoseFilter
from hand_follow.retarget import NAMES
from hand_follow.diagnostics import plain


def evaluate(repo, output, session=None, render=False):
    output.mkdir(parents=True,exist_ok=True)
    design=json.loads((repo/'cad/accepted_form_v1/design.json').read_text())
    ranges=np.array([design['joint_ranges_deg'][n if n.startswith('thumb') else n.split('_',1)[1]] for n in NAMES])
    report={'scope':__doc__,'geometry':'cad/accepted_form_v1','joint_names':NAMES}
    filt=ContinuousPoseFilter(ranges);filt(np.zeros(20),0)
    target=np.zeros(20);target[2]=90
    curve=np.array([filt(target,i/60)[2] for i in range(1,61)])
    report['synthetic_90deg_step']={name:float((np.flatnonzero(curve>=angle)[0]+1)/60*1000)
        for name,angle in [('reach50_ms',45),('reach90_ms',81)]}
    rng=np.random.default_rng(64);filt=ContinuousPoseFilter(ranges);noise=[]
    for i in range(400):
        target[2]=45+rng.normal(0,2);noise.append(filt(target,i/30)[2])
    report['synthetic_still_2deg_noise_output_std_deg']=float(np.std(noise[100:]))
    if session:
        raw_path=session/'landmarks.jsonl'
        rows=[json.loads(line) for line in raw_path.read_text().splitlines()]
        pipe=PosePipeline(design,ranges,output/'unused_calibration.json',retarget_mode='position')
        records=[];timings=[];held=[];rebased=[]
        for row in rows:
            t=row.get('capture_elapsed_s',row['elapsed_s'])
            det=row['detection']
            started=time.perf_counter();pose=pipe.update(det,t,t)
            timings.append((time.perf_counter()-started)*1000)
            held.append(pose.get('observation_quality',{}).get('held_fingers',[]))
            rebased.append(pose.get('observation_quality',{}).get('rebased_fingers',[]))
            records.append(dict(t=t,tracked=pose['tracked'],before=row['actual_deg'],after=pose['command_deg'],
                warning=pose['warning'],held=held[-1],input_present=bool(det)))
        valid=[(i,r) for i,r in enumerate(records) if r['tracked']]
        report.update(replayed=len(rows),tracked=len(valid),input_sha256=hashlib.sha256(raw_path.read_bytes()).hexdigest(),
            replay_identity_scope='Original stored handedness unchanged. Raw classifier votes unavailable: new tracker identity selection not replayed.',
            detail_sample_scope='Old recording subsampled about 10 Hz; missing processed frames cannot be reconstructed',
            held_group_samples=[sum(g in h for h in held) for g in range(5)],rebased_group_samples=[sum(g in h for h in rebased) for g in range(5)],
            sample_with_hold=sum(bool(h) for h in held),replay_ms_p50_p95=np.percentile(timings,[50,95]).tolist())
        report['tracked_map_ms_p50_p95']=np.percentile([timings[i] for i,_ in valid],[50,95]).tolist()
        for key in ('before','after'):
            steps=[];thumb=[];reacq=[]
            for (i,a),(j,b) in zip(valid,valid[1:]):
                delta=np.abs(np.array(b[key])-a[key])
                if j==i+1 and b['t']-a['t']<=.3:
                    steps.append(delta.max());thumb.append(delta[16:].max())
                elif j>i+1:
                    reacq.append(float(np.max(np.abs(np.array(b[key])-records[j-1][key]))))
            values=np.array([r[key] for _,r in valid])
            report[key]=dict(adjacent_max_step_p50_p95_max=np.percentile(steps,[50,95,100]).tolist(),
                adjacent_thumb_step_p50_p95_max=np.percentile(thumb,[50,95,100]).tolist(),
                reacquisition_max_step=max(reacq,default=0),
                joint_p95_minus_p5=(np.percentile(values,95,axis=0)-np.percentile(values,5,axis=0)).tolist())
        # Numeric replay output stays with the private session evaluation, not
        # automatically in a published checkpoint.
        (output/'replay.json').write_text(json.dumps(records,default=plain),encoding='utf-8')
        if render:render_comparison(repo,records,output)
    (output/'report.json').write_text(json.dumps(report,indent=2,default=plain)+'\n',encoding='utf-8',newline='\n')
    return report


def render_comparison(repo,records,output):
    from PIL import Image,ImageDraw,ImageFont
    from hand_follow.form_preview import FormPreviewHand
    robot=FormPreviewHand(repo/'cad/accepted_form_v1')
    font=ImageFont.truetype('C:/Windows/Fonts/malgun.ttf',19)
    frames=[]
    # Fixed diagnostic intervals include the recorded label flip, thumb
    # transition and distal-length anomaly; use the same accepted geometry.
    selected=[r for r in records if any(a<=r['t']<=b for a,b in ((31,34),(43,46),(231,234)))]
    try:
        for row in selected:
            frame=Image.new('RGB',(960,580),(20,26,36));draw=ImageDraw.Draw(frame)
            for index,key in enumerate(('before','after')):
                robot.set(row[key]);image=robot.render(1,width=480,height=480,fast=True)
                frame.paste(Image.fromarray(image),(index*480,55))
                draw.text((index*480+18,12),'이전 실제 출력' if index==0 else '수정 코드 재생',font=font,fill='white')
            label=f"기록 {row['t']:.2f}초 · 같은 형상 / 같은 저장 입력 · 실제 정확도 미확인"
            draw.text((18,542),label,font=font,fill='#abc0d0')
            frames.append(frame)
        frames[0].save(output/'tracking_comparison.gif',save_all=True,append_images=frames[1:],duration=150,loop=0)
        frames[len(frames)//2].save(output/'tracking_comparison.png')
    finally:robot.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--session',type=Path);parser.add_argument('--render',action='store_true')
    args=parser.parse_args();evaluate(Path(__file__).resolve().parents[2],args.output,args.session,args.render)
