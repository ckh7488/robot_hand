"""Paired UI composition benchmark; same rendered pixels, no camera/window."""
import argparse
import importlib.util
import json
from pathlib import Path
import tempfile
import time
import numpy as np
from PIL import Image
from hand_follow.form_preview import FormPreviewHand
from hand_follow.pipeline import PosePipeline
from hand_follow.ui import PreviewUI


def run(baseline,output):
    output.mkdir(parents=True,exist_ok=True)
    spec=importlib.util.spec_from_file_location('old_preview',baseline)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    robot=FormPreviewHand(Path(__file__).resolve().parents[2]/'cad/accepted_form_v1')
    samples={'before':[],'after':[]}
    try:
        with tempfile.TemporaryDirectory() as tmp:
            pipe=PosePipeline(robot.design,robot.ranges,Path(tmp)/'unused.json',retarget_mode='position')
            q=pipe.fist;robot.set(q);rendered=robot.render(0,660,570,fast=True)
            pose=dict(command_deg=q,raw_deg=q,mapped_deg=q,tracked=True,warning='',estimated_joints=[],clipped_joints=[])
            for mode in ('before','after','after','before'):
                ui=(module.PreviewUI if mode=='before' else PreviewUI)(headless=True)
                for i in range(35):
                    timing=dict(camera_fps=30.,processing_fps=15+i*.1,track_ms=25+i,age_ms=50+i*.7)
                    start=time.perf_counter()
                    canvas=ui.compose(np.zeros((480,640,3),np.uint8),None,rendered,pose,pipe,timing,'live',0)
                    if i>=5:samples[mode].append((time.perf_counter()-start)*1000)
                if mode=='after':Image.fromarray(canvas).save(output/'preview_layout.png')
    finally:robot.close()
    report=dict(scope=__doc__,samples_per_mode=60,results={k:dict(median_ms=float(np.median(v)),p95_ms=float(np.percentile(v,95))) for k,v in samples.items()})
    (output/'ui_benchmark.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8',newline='\n')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--baseline-ui',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();run(a.baseline_ui,a.output)
