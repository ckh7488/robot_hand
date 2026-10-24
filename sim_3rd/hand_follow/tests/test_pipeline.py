import json
from pathlib import Path
import tempfile
import time
import unittest
import numpy as np
from hand_follow.pipeline import PosePipeline, FIST, FLEX
from hand_follow.retarget import raw_angles, joint_ranges
from hand_follow.synthetic import landmarks_from_angles
DESIGN = {'thumb': {'base_yaw_deg': -40, 'second_yaw_deg': 25}}

class RealLandmarks(unittest.TestCase):

    def test_actual_mediapipe_fists_and_mirrors_have_positive_curl(self):
        data = json.loads((Path(__file__).parent / 'data/mediapipe_poses.json').read_text(encoding='utf-8-sig'))
        for sample in data['samples']:
            angles = raw_angles(sample['world'], sample['hand'])
            for offset in range(0, 16, 4):
                self.assertGreater(angles[offset + 1], 20, sample['file'])
                self.assertGreater(angles[offset + 2], 45, sample['file'])
                self.assertGreater(angles[offset + 3], 30, sample['file'])

    def test_deep_flexion_does_not_reverse_distal_joints(self):
        for hand in ('Left', 'Right'):
            for flex in (85, 89, 91, 95, 105):
                pose = np.array([5, flex, 90, 65] * 4 + [0] * 4, float)
                result = raw_angles(landmarks_from_angles(pose, hand), hand)
                np.testing.assert_allclose(result[:16], pose[:16], atol=1e-08)

    def test_collapsed_palm_is_rejected(self):
        with self.assertRaises(ValueError):
            raw_angles(np.zeros((21, 3)))

class Pipeline(unittest.TestCase):

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.pipeline = PosePipeline(DESIGN, joint_ranges(), Path(self.temp.name) / 'calibration.json')

    def tearDown(self):
        self.temp.cleanup()

    def detection(self, angles):
        return dict(world=landmarks_from_angles(angles), handedness='Right', score=0.99)

    def capture(self, kind, angles):
        self.pipeline.start_calibration(kind)
        start = self.pipeline.capture_started
        for i in range(60):
            now = start + i / 30
            self.pipeline.update(self.detection(angles), now, now)

    def test_open_then_fist_calibration_reaches_robot_range(self):
        offset = np.array([4, 5, 4, 3] * 4 + [0] * 4, float)
        self.capture('open', offset)
        self.assertTrue(self.pipeline.open_calibrated)
        measured = offset + np.array([0, 50, 65, 45] * 4 + [0] * 4, float)
        self.capture('fist', measured)
        self.assertTrue(self.pipeline.fist_calibrated)
        self.pipeline.filter.reset()
        now = time.perf_counter() + 10
        result = self.pipeline.update(self.detection(measured), now, now)
        np.testing.assert_allclose(result['command_deg'][FLEX], FIST[FLEX], atol=1e-06)

    def test_invalid_fist_does_not_create_large_gain(self):
        self.capture('open', np.zeros(20))
        self.capture('fist', np.zeros(20))
        np.testing.assert_array_equal(self.pipeline.calibration.gain, np.ones(20))
        self.assertFalse(self.pipeline.fist_calibrated)

    def test_one_occluded_tip_does_not_discard_other_finger_calibration(self):
        offset = np.zeros(20)
        offset[3] = 36
        self.capture('open', offset)
        measured = offset + np.array([0, 50, 65, 45] * 4 + [0] * 4, float)
        measured[3] = 38  # Depth inference cannot distinguish this tip's poses.
        self.capture('fist', measured)
        self.assertTrue(self.pipeline.fist_calibrated)
        self.assertEqual(self.pipeline.calibration.coupled_dips, [3])
        self.pipeline.filter.reset()
        now = time.perf_counter() + 10
        result = self.pipeline.update(self.detection(measured), now, now)
        np.testing.assert_allclose(result['command_deg'][FLEX], FIST[FLEX], atol=1e-6)
        self.assertEqual(result['estimated_joints'], [3])
        loaded = PosePipeline(DESIGN, joint_ranges(), self.pipeline.calibration_path)
        self.assertEqual(loaded.calibration.coupled_dips, [3])

    def test_tracking_loss_holds_pose_and_recovers(self):
        now = time.perf_counter()
        result = self.pipeline.update(self.detection(FIST), now, now)
        held = self.pipeline.update(None, now + 0.5, now + 0.5)
        np.testing.assert_equal(held['command_deg'], result['command_deg'])
        self.assertFalse(held['tracked'])
        recovered = self.pipeline.update(self.detection(np.zeros(20)), now + 0.6, now + 0.6)
        np.testing.assert_allclose(recovered['command_deg'], np.zeros(20), atol=1e-08)
