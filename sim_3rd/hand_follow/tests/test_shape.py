"""Pose preservation regressions: a small tip error must not hide wrong bends."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from hand_follow.shape import ShapeRetargeter, thumb_directions, geometry_signature
from hand_follow.retarget import Calibration, raw_angles, NAMES
from hand_follow.synthetic import landmarks_from_angles
from hand_follow.fingers import observations
from hand_follow.pipeline import PosePipeline

REPO = Path(__file__).resolve().parents[3]
DESIGN = json.loads((REPO/'cad/accepted_form_v1/design.json').read_text())
RANGES = np.array([DESIGN['joint_ranges_deg'][n if n.startswith('thumb') else n.split('_',1)[1]] for n in NAMES])
OPEN = np.zeros(20)
FIST = np.array([0,65,80,55]*4+[15,45,45,55],float)


def calibration():
    cal = Calibration()
    for name, pose in [('open',OPEN),('fist',FIST)]:
        p = landmarks_from_angles(pose)
        cal.position_profile[name+'_tips'] = observations(p,'Right')['tips'].tolist()
        cal.position_profile['thumb_'+name+'_directions'] = thumb_directions(p,'Right').tolist()
    cal.position_profile.update(schema=1,cad_signature=geometry_signature(DESIGN))
    flex = np.array([i for i in range(16) if i%4])
    cal.gain[flex] = np.array(DESIGN['fist_deg'])[flex]/FIST[flex]
    return cal


class ShapeMapping(unittest.TestCase):
    def setUp(self):
        self.fit = ShapeRetargeter(DESIGN,RANGES)
        self.cal = calibration()

    def test_calibrated_open_and_fist_reach_authored_endpoints(self):
        for pose, expected in [(OPEN,OPEN),(FIST,DESIGN['fist_deg'])]:
            p = landmarks_from_angles(pose)
            fingers = self.fit.fingers(raw_angles(p),self.cal)
            thumb, info = self.fit.thumb(p,'Right',self.cal)
            np.testing.assert_allclose(np.r_[fingers,thumb],expected,atol=1e-7)
            self.assertTrue(info['calibrated'])

    def test_mixed_pose_keeps_root_bend_and_distal_extension(self):
        # Previously a tip fit could replace this with root ~0, DIP ~90.
        pose = np.array([0,65,50,0, 8,12,0,0, 0,60,80,0, -8,0,0,0]+[0]*4,float)
        result = self.fit.fingers(raw_angles(landmarks_from_angles(pose)),Calibration())
        np.testing.assert_allclose(result[[1,2,3,5,6,7,9,10,11,13,14,15]],
                                   pose[[1,2,3,5,6,7,9,10,11,13,14,15]],atol=1e-7)
        changed = pose.copy(); changed[3]=40
        after = self.fit.fingers(raw_angles(landmarks_from_angles(changed)),Calibration())
        np.testing.assert_allclose(after[np.arange(16)!=3],result[np.arange(16)!=3],atol=1e-7)
        self.assertAlmostEqual(after[3]-result[3],40)

    def test_hook_and_root_only_flexion_keep_different_bend_patterns(self):
        for bends in ([0,80,40],[75,0,0]):
            pose = np.array(([0]+bends)*4+[0]*4,float)
            actual = self.fit.fingers(raw_angles(landmarks_from_angles(pose)),Calibration())
            np.testing.assert_allclose(actual,pose[:16],atol=1e-7)

    def test_other_four_fingers_cannot_move_thumb(self):
        a = FIST*.6
        b = a.copy(); b[:16]=[0,65,80,55, 0,0,0,0, 0,65,80,55, 0,0,0,0]
        before,_=self.fit.thumb(landmarks_from_angles(a),'Right',self.cal)
        after,_=self.fit.thumb(landmarks_from_angles(b),'Right',self.cal)
        np.testing.assert_allclose(before,after,atol=1e-8)

    def test_thumb_has_four_local_degrees_of_freedom(self):
        base = FIST*.5
        initial,_=self.fit.thumb(landmarks_from_angles(base),'Right',self.cal)
        columns=[]
        for joint in range(16,20):
            moved=base.copy();moved[joint]+=.01
            q,_=self.fit.thumb(landmarks_from_angles(moved),'Right',self.cal)
            columns.append((q-initial)/.01)
        self.assertEqual(np.linalg.matrix_rank(np.array(columns),tol=1e-4),4)

    def test_mirror_and_camera_rotation_preserve_thumb(self):
        pose=FIST*.6
        expected,_=self.fit.thumb(landmarks_from_angles(pose),'Right',self.cal)
        rotation=Rotation.from_euler('xyz',[32,-57,78],degrees=True).as_matrix()
        for hand in ('Left','Right'):
            actual,_=self.fit.thumb(landmarks_from_angles(pose,hand,rotation=rotation),hand,self.cal)
            np.testing.assert_allclose(actual,expected,atol=1e-7)

    def test_insufficient_thumb_calibration_is_visible_and_finite(self):
        self.cal.position_profile['thumb_fist_directions']=self.cal.position_profile['thumb_open_directions']
        result,info=self.fit.thumb(landmarks_from_angles(FIST),'Right',self.cal)
        self.assertFalse(info['calibrated'])
        self.assertTrue(info['warning'])
        self.assertTrue(np.isfinite(result).all())

    def test_invalid_profile_and_changed_thumb_geometry_are_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'cal.json'
            self.cal.position_profile['thumb_open_directions'][0]=[0,0,0]
            self.cal.save(path)
            with self.assertRaises(ValueError):Calibration.load(path)
            calibration().save(path)
            changed=copy.deepcopy(DESIGN);changed['thumb']['base_yaw_deg']+=5
            pipe=PosePipeline(changed,RANGES,path,retarget_mode='shape')
            self.assertFalse(pipe.open_calibrated)

    def test_pipeline_captures_restores_thumb_and_shows_debug_ui(self):
        from hand_follow.ui import PreviewUI
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'cal.json'
            pipe=PosePipeline(DESIGN,RANGES,path,retarget_mode='shape')
            for kind,pose in [('open',OPEN),('fist',FIST)]:
                p=landmarks_from_angles(pose)
                obs=observations(p,'Right');obs['thumb_directions']=thumb_directions(p,'Right')
                pipe.start_calibration(kind);start=pipe.capture_started
                for elapsed in np.linspace(.5,1.9,30):pipe._capture(raw_angles(p),start+elapsed,obs)
            restored=PosePipeline(DESIGN,RANGES,path,retarget_mode='shape')
            self.assertIn('thumb_fist_directions',restored.calibration.position_profile)
            actual=restored.update(dict(world=landmarks_from_angles(FIST),handedness='Right'),1,1)
            self.assertTrue(actual['thumb_fit']['calibrated'])
            np.testing.assert_allclose(actual['mapped_deg'],DESIGN['fist_deg'],atol=1e-7)
            ui=PreviewUI(headless=True)
            image=ui.compose(None,None,np.zeros((570,660,3),np.uint8),actual,restored,
                dict(camera_fps=30,processing_fps=30,track_ms=10,age_ms=20),'live',0,True)
            self.assertEqual(image.shape,(900,1680,3))

    def test_unreliable_thumb_hold_does_not_stop_other_fingers(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'cal.json';self.cal.save(path)
            pipe=PosePipeline(DESIGN,RANGES,path,retarget_mode='shape')
            for i in range(10):before=pipe.update(dict(world=landmarks_from_angles(OPEN),handedness='Right'),i/30,i/30)
            p=landmarks_from_angles(FIST);p[4]=p[3]+2*(p[4]-p[3])
            after=pipe.update(dict(world=p,handedness='Right'),10/30,10/30)
            self.assertIn(4,after['observation_quality']['held_fingers'])
            np.testing.assert_allclose(after['mapped_deg'][16:],before['mapped_deg'][16:])
            self.assertGreater(after['mapped_deg'][2],80)
