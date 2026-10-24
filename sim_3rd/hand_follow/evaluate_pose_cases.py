"""Exercise the app's displayed-frame capture path without opening a camera/window."""
import argparse
import json
from pathlib import Path
from unittest.mock import patch
import numpy as np
from hand_follow.app import main
from hand_follow.ui import PreviewUI
from review_pose_case import render_case


def evaluate(output):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    displayed=[]
    class TestUI(PreviewUI):
        def __init__(self):super().__init__(headless=True)
        def show(self,*args,**kwargs):
            self.last_canvas=self.compose(*args,**kwargs)
            detection=args[1]
            displayed.append((np.array(detection['world']).copy(),self.last_canvas.copy()))
            # One button release and one keyboard press, with an unmarked frame between.
            if len(displayed)==1:
                import cv2
                self._mouse(cv2.EVENT_LBUTTONUP,200,754,None,None)
                return self.action
            return ord('s') if len(displayed)==3 else ord('q') if len(displayed)==4 else 255
        def close(self):pass
    with patch('hand_follow.ui.PreviewUI',TestUI):
        main(['--source','synthetic','--retarget','shape','--seconds','15',
              '--calibration',str(output/'unused_calibration.json'),'--log-dir',str(output/'sessions')])
    session=Path(json.loads((output/'sessions/latest_session.json').read_text())['directory'])
    cases=sorted((session/'cases').glob('case_????'))
    assert len(cases)==2,len(cases)
    from PIL import Image
    for path,index in zip(cases,[0,2]):
        data=json.loads((path/'case.json').read_text(encoding='utf-8'))
        np.testing.assert_array_equal(data['detection']['world'],displayed[index][0])
        np.testing.assert_array_equal(np.array(Image.open(path/'screen.png')),displayed[index][1])
        assert data['retarget_mode']=='shape'
    assert len(list(session.rglob('*.png')))==4  # screen+robot per explicit capture; synthetic has no camera image.
    events=[json.loads(s) for s in (session/'events.jsonl').read_text(encoding='utf-8').splitlines()]
    assert len([e for e in events if e['event']=='pose_case_saved'])==2
    status=json.loads((session/'status.json').read_text())
    assert status['state']=='stopped' and status['error'] is None
    report=render_case(cases[0],output/'raw_review')
    result=dict(passed=True,displayed_frames=len(displayed),explicit_captures=len(cases),
                exact_raw_coordinates=True,exact_screen_pixels=True,raw_review=report,
                scope='Synthetic app integration; not live hand accuracy or physical camera timing.')
    (output/'validation.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    (output/'preview_layout.png').write_bytes((cases[1]/'screen.png').read_bytes())
    return result


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--output',type=Path,required=True)
    print(json.dumps(evaluate(ap.parse_args().output),indent=2))
