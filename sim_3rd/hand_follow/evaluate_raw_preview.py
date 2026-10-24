"""Render the live comparison layout without a camera; verify its raw provenance."""
import argparse
import json
from pathlib import Path
from types import SimpleNamespace
import time
import numpy as np
from PIL import Image
from hand_follow.form_preview import FormPreviewHand
from hand_follow.raw_preview import display_points, render_raw
from hand_follow.retarget import palm_frame
from hand_follow.synthetic import landmarks_from_angles
from hand_follow.ui import PreviewUI


def evaluate(output):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    repo=Path(__file__).resolve().parents[2]
    robot=FormPreviewHand(repo/'cad/accepted_form_v1')
    ui=PreviewUI(headless=True)
    angles=np.array([0,65,75,40,0,0,0,0,0,0,0,0,0,0,0,0,10,35,40,30],float)
    raw=landmarks_from_angles(angles)
    detection=dict(world=raw,handedness='Right')
    pose=dict(tracked=True,command_deg=angles,raw_deg=angles,mapped_deg=angles,
              warning='',estimated_joints=[],clipped_joints=[])
    pipeline=SimpleNamespace(retarget_mode='shape',design=robot.design,message='',capture_kind=None,filter_on=True)
    timing=dict(camera_fps=30,processing_fps=25,track_ms=17,age_ms=40)
    camera=np.full((480,640,3),[40,34,29],np.uint8)
    camera[80:400,180:460]=[60,52,45]
    robot.set(angles)
    saved=[];worst=0.
    try:
        for view in range(3):
            rendered=robot.render(view,width=426,height=496,fast=True)
            eye=robot.renderer.scene.camera[0]
            forward=np.asarray(eye.forward,float);forward/=np.linalg.norm(forward)
            up=np.asarray(eye.up,float);up/=np.linalg.norm(up)
            right=np.cross(forward,up);right/=np.linalg.norm(right)
            q=(raw-raw[0])@palm_frame(raw)
            expected=q@np.column_stack((right,up,-forward))
            difference=float(np.max(np.abs(display_points(raw,view)-expected)))
            worst=max(worst,difference)
            assert difference<1e-7,difference
            canvas=ui.compose(camera,detection,rendered,pose,pipeline,timing,'live',view)
            np.testing.assert_array_equal(canvas[111:731,566:1098],render_raw(detection,view)[0])
            assert canvas.shape==(900,1680,3)
            # A second compose exercises the static label cache and current view.
            canvas=ui.compose(camera,detection,rendered,pose,pipeline,timing,'live',view)
            np.testing.assert_array_equal(canvas[111:731,566:1098],render_raw(detection,view)[0])
            name=f'view_{view+1}.png';Image.fromarray(canvas).save(output/name);saved.append(name)
        start=time.perf_counter()
        for _ in range(100):render_raw(detection,1)
        drawing_ms=(time.perf_counter()-start)*10
        missing=ui.compose(camera,None,rendered,dict(pose,tracked=False),pipeline,timing,'live',2)
        Image.fromarray(missing).save(output/'missing.png')
        ui.compose(camera,detection,rendered,pose,pipeline,timing,'test',0,debug=True)
        result=dict(passed=True,raw_panel_equals_same_frame=True,camera_basis_max_error_m=worst,
                    raw_drawing_mean_ms=drawing_ms,views=saved,canvas_size=[1680,900],
                    scope='Synthetic data and display integrity, not detector or robot pose accuracy.')
        (output/'validation.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
        return result
    finally:robot.close()


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--output',required=True,type=Path)
    print(json.dumps(evaluate(ap.parse_args().output),indent=2))
