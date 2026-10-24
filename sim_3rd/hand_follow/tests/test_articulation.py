"""Regression checks for the actual articulated CAD, separate from tracking."""
from pathlib import Path
import tempfile
import unittest
import numpy as np
import test_cad
import test_thumb
from hand_follow.cad_hand import CadHand
from hand_follow.pipeline import PosePipeline, FLEX
from hand_follow.retarget import Calibration


class ArticulatedCad(test_cad.CadMapping, test_thumb.ThumbGeometry):
    @classmethod
    def setUpClass(cls):
        cls.robot = CadHand(test_cad.REPO, cad='webcam_articulation_v1')

    def test_root_has_two_perpendicular_axes_and_distal_joints_curl_inward(self):
        robot = self.robot
        robot.set(np.zeros(20))
        model, data = robot.model, robot.data
        sweep = model.joint('thumb_base_sweep').id
        fold = model.joint('thumb_opposition').id
        self.assertAlmostEqual(float(data.xaxis[sweep] @ data.xaxis[fold]), 0, places=10)
        self.assertAlmostEqual(np.linalg.norm(data.xanchor[sweep]-data.xanchor[fold]), .021, places=10)
        neutral = data.site_xpos[model.site('thumb_ip_tip').id].copy()
        for index in (18, 19):
            pose = np.zeros(20)
            pose[index] = 35
            robot.set(pose)
            tip = data.site_xpos[model.site('thumb_ip_tip').id]
            self.assertLess(tip[0], neutral[0]-.01, 'Distal thumb must curl toward palm centre')

    def test_complete_fist_reaches_palm_and_bends_both_thumb_joints(self):
        fist = np.array(self.robot.design['fist_deg'])
        self.robot.set(fist)
        np.testing.assert_allclose(self.robot.angles, fist, atol=1e-9)
        for finger in ('index', 'middle', 'ring', 'little'):
            tip = self.robot.data.site_xpos[self.robot.model.site(finger+'_dip_tip').id]
            self.assertAlmostEqual(tip[2]*1000, 16, places=6)
        self.assertTrue(np.all(fist[18:] > 25))
        tip = self.robot.data.site_xpos[self.robot.model.site('thumb_ip_tip').id]*1000
        self.assertLess(tip[0], self.robot.design['fingers'][0]['root'][0])
        self.assertGreater(tip[2], 85)

    def test_calibration_uses_new_fist_and_persists_occluded_tip_ratio(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'calibration.json'
            pipeline = PosePipeline(self.robot.design, self.robot.ranges, path)
            pipeline.open_calibrated = True
            measured = np.array([0,50,60,40]*4+[0]*4, float)
            measured[3] = 2
            pipeline.start_calibration('fist')
            start = pipeline.capture_started
            for elapsed in np.linspace(.5, 1.9, 45):
                pipeline._capture(measured, start+elapsed)
            self.assertTrue(pipeline.fist_calibrated)
            loaded = Calibration.load(path)
            np.testing.assert_allclose(loaded.apply(measured)[FLEX], pipeline.fist[FLEX])
            self.assertAlmostEqual(loaded.dip_ratios[3], pipeline.fist[3]/pipeline.fist[2])
