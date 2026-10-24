"""Paired preview-only benchmark. Excludes camera, tracking and window display."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import time
import cv2
import numpy as np
from hand_follow.cad_hand import CadHand
from hand_follow.pipeline import PosePipeline
from hand_follow.ui import PreviewUI


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline-ui',type=Path,required=True,help='Historical ui.py, exported separately')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--repo',type=Path,default=Path(__file__).resolve().parents[2])
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    spec=importlib.util.spec_from_file_location('baseline_preview',args.baseline_ui)
    old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
    default_threads=cv2.getNumThreads()
    robot=CadHand(args.repo,cad='webcam_articulation_v1')
    samples={'before':[],'after':[]};blocks=[]
    try:
        with tempfile.TemporaryDirectory() as tmp:
            pipeline=PosePipeline(robot.design,robot.ranges,Path(tmp)/'unused.json')
            q=pipeline.fist.copy();robot.set(q)
            pose=dict(command_deg=q,raw_deg=q,mapped_deg=q,tracked=True,warning='',
                      estimated_joints=[],clipped_joints=[])
            camera=np.zeros((480,640,3),np.uint8)
            for mode in ('before','after','after','before'):
                robot.close()
                cv2.setNumThreads(default_threads if mode=='before' else 1)
                ui=(old.PreviewUI if mode=='before' else PreviewUI)(headless=True)
                dims=(880,760) if mode=='before' else (660,570)
                rows=[]
                for i in range(28):
                    timing=dict(camera_fps=30.,processing_fps=11+(i%9)*.1,track_ms=25+i%13,age_ms=53+i%7)
                    start=time.perf_counter();render=robot.render(1,*dims,fast=mode=='after');mid=time.perf_counter()
                    canvas=ui.compose(camera,None,render,pose,pipeline,timing,'live',1,False);end=time.perf_counter()
                    if i>=8:rows.append([(mid-start)*1000,(end-mid)*1000,(end-start)*1000])
                samples[mode].extend(rows)
                blocks.append(dict(mode=mode,median_ms=np.median(rows,axis=0).tolist()))
                if mode=='after':
                    from PIL import Image
                    Image.fromarray(canvas).save(args.output/'preview_layout.png')
    finally:
        robot.close()
    source=Path(__file__).parent/'hand_follow'
    result=dict(scope=__doc__,columns=['render_ms','compose_ms','total_ms'],blocks=blocks,
        baseline_ui_sha256=hashlib.sha256(args.baseline_ui.read_bytes()).hexdigest(),
        source_sha256={name:hashlib.sha256((source/name).read_bytes()).hexdigest() for name in ('ui.py','cad_hand.py')},
        samples_per_mode=40,default_opencv_threads=default_threads,
        results={label:dict(median_ms=np.median(rows,axis=0).tolist(),p90_ms=np.percentile(rows,90,axis=0).tolist())
                 for label,rows in samples.items()})
    (args.output/'preview_benchmark.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result),flush=True)


if __name__=='__main__':main()
