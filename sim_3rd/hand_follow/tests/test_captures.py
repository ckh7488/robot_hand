"""Keep a marked picture and its raw detector values from the same frame."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import numpy as np
from PIL import Image
from hand_follow.captures import PoseCaseRecorder, capture_displayed_frame
from hand_follow.retarget import Calibration
from hand_follow.ui import PreviewUI


class PoseCaptures(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)/'한글 세션'
        self.recorder = PoseCaseRecorder(self.root)
        self.pipeline = SimpleNamespace(retarget_mode='shape',filter_on=True,calibration=Calibration())
        self.raw = np.arange(63,dtype=float).reshape(21,3)/12345.6789
        self.camera = np.full((12,16,3),[7,23,199],np.uint8)
        self.screen = np.full((15,20,3),[80,90,100],np.uint8)
        self.args = dict(seq=314,capture_elapsed_s=4.1,command_elapsed_s=4.14,
            detection=dict(world=self.raw,image=np.zeros((21,2)),normalized=self.raw.copy(),handedness='Right'),
            pose=dict(tracked=True,command_deg=np.arange(20)),actual=np.arange(20)+.25,
            timing=dict(age_ms=40),mode='live',view=1,pipeline=self.pipeline,
            camera_bgr=self.camera,screen_rgb=self.screen,robot_rgb=self.screen[:10])

    def tearDown(self):
        self.temp.cleanup()

    def test_explicit_capture_preserves_exact_input_pixels_and_frame_identity(self):
        self.assertFalse(self.recorder.root.exists())
        path=self.recorder.save(**self.args)
        data=json.loads((path/'case.json').read_text(encoding='utf-8'))
        self.assertEqual(data['seq'],314)
        self.assertEqual(data['capture_elapsed_s'],4.1)
        np.testing.assert_array_equal(data['detection']['world'],self.raw)
        np.testing.assert_array_equal(data['detection']['normalized'],self.raw)
        np.testing.assert_array_equal(np.array(Image.open(path/'camera.png')),self.camera[:,:,::-1])
        np.testing.assert_array_equal(np.array(Image.open(path/'screen.png')),self.screen)
        np.testing.assert_array_equal(data['actual_deg'],self.args['actual'])
        self.raw[:]=0;self.camera[:]=0
        self.assertNotEqual(data['detection']['world'][1][1],0)
        self.assertEqual(data['retarget_mode'],'shape')

    def test_repeated_and_missing_detections_never_overwrite_or_reuse_landmarks(self):
        first=self.recorder.save(**self.args)
        args=dict(self.args,detection=None,mode='test',seq=315)
        second=PoseCaseRecorder(self.root).save(**args)
        self.assertNotEqual(first,second)
        data=json.loads((second/'case.json').read_text(encoding='utf-8'))
        self.assertIsNone(data['detection'])
        self.assertEqual(data['mode'],'test')
        self.assertEqual(data['seq'],315)
        self.assertTrue((second/'camera.png').exists())

    def test_failed_write_does_not_publish_success_or_end_tracking(self):
        log=SimpleNamespace(started=1,event=Mock())
        ui=SimpleNamespace(last_canvas=self.screen,notify=Mock(),capture_count=0)
        with patch.object(Image.Image,'save',side_effect=OSError('disk full')):
            result=capture_displayed_frame(self.recorder,log,ui,self.pipeline,
                seq=4,captured=2,command_time=2.1,detection=self.args['detection'],pose={},actual=np.zeros(20),
                timing={},mode='live',view=0,camera_bgr=self.camera,robot_rgb=self.screen)
        self.assertIsNone(result)
        self.assertEqual(log.event.call_args.args[0],'pose_case_failed')
        self.assertEqual(self.recorder.saved,0)
        self.assertFalse((self.recorder.root/'case_0001').exists())
        self.assertEqual(PoseCaseRecorder(self.root).next_number,2)

    def test_new_button_uses_same_s_action_and_notice_as_keyboard(self):
        import cv2
        ui=PreviewUI(headless=True)
        button=next(b for b in ui.buttons if b[4]=='S')
        x1,y1,x2,y2,_,_=button
        ui._mouse(cv2.EVENT_LBUTTONUP,(x1+x2)//2,(y1+y2)//2,None,None)
        self.assertEqual(ui.action,ord('s'))
        log=SimpleNamespace(started=1,event=Mock())
        ui.last_canvas=self.screen
        result=capture_displayed_frame(self.recorder,log,ui,self.pipeline,
            seq=4,captured=2,command_time=2.1,detection=None,pose={},actual=np.zeros(20),
            timing={},mode='live',view=0,camera_bgr=self.camera,robot_rgb=self.screen)
        self.assertEqual(result.name,'case_0001')
        self.assertEqual(log.event.call_args.args[0],'pose_case_saved')
        self.assertEqual(ui.capture_count,1)
        self.assertIn('#001',ui.notice)
        self.assertIn('인식 없음',ui.notice)
