from pathlib import Path
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from hand_follow.cad_hand import CadHand
from hand_follow.pipeline import FIST
from hand_follow.retarget import raw_angles
from hand_follow.synthetic import landmarks_from_angles
from hand_follow.thumb import ThumbRetargeter, chain

REPO = Path(__file__).resolve().parents[3]


class ThumbGeometry(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = CadHand(REPO)

    @classmethod
    def tearDownClass(cls):
        cls.robot.close()

    def test_optimizer_chain_agrees_with_loaded_cad_joints(self):
        rng = np.random.default_rng(83)
        for _ in range(20):
            pose = rng.uniform(*self.robot.ranges.T)
            self.robot.set(pose)
            for fi, config in enumerate([*self.robot.design['fingers'], self.robot.design['thumb']]):
                name = config.get('name', 'thumb')
                names = ('opposition', 'mcp', 'ip') if name == 'thumb' else ('mcp', 'pip', 'dip')
                actual = [self.robot.data.site_xpos[self.robot.model.site(f'{name}_{joint}_tip').id] * 1000 for joint in names]
                np.testing.assert_allclose(chain(config, pose[fi * 4:fi * 4 + 4])[1:], actual, atol=1e-7)

    def wrapped_human(self):
        points = landmarks_from_angles(FIST)
        # A stylized thumb rolls across the palm; it is not generated from the
        # CAD's coplanar hinge axes and catches the old angle-projection failure.
        points[1:5] = [[.025, .020, .005], [.040, .050, .020],
                       [.020, .080, .040], [-.015, .090, .040]]
        return points

    def test_crossing_thumb_moves_above_folded_fingers(self):
        points = self.wrapped_human()
        fit = ThumbRetargeter(self.robot.design, self.robot.ranges)
        angles, info = fit.solve(points, 'Right', FIST, raw_angles(points)[16:])
        tip = chain(self.robot.design['thumb'], angles)[-1]
        self.assertGreater(info['wrap_weight'], .99)
        self.assertLess(tip[0], 15)
        self.assertGreater(tip[2], 85)
        # This stylized human has a wider span than the CAD can reach. The
        # residual must remain visible; do not extend joints to erase it.
        self.assertLess(info['tip_error_mm'], 30)
        self.assertTrue(np.all(angles >= self.robot.ranges[16:, 0]))
        self.assertTrue(np.all(angles <= self.robot.ranges[16:, 1]))

    def test_mirror_and_camera_rotation_preserve_thumb_target(self):
        points = self.wrapped_human()
        values = []
        for hand, rotation in [('Right', np.eye(3)), ('Left', Rotation.from_euler('xyz', [38, -19, 72], degrees=True).as_matrix())]:
            p = points.copy()
            if hand == 'Left':
                p[:, 0] *= -1
            p = p @ rotation.T
            fit = ThumbRetargeter(self.robot.design, self.robot.ranges)
            angles, info = fit.solve(p, hand, FIST, raw_angles(p, hand)[16:])
            values.append((angles, info['target_mm']))
        np.testing.assert_allclose(values[0][0], values[1][0], atol=.001)
        np.testing.assert_allclose(values[0][1], values[1][1], atol=1e-8)

    def test_open_hand_does_not_trigger_fist_wrap(self):
        points = landmarks_from_angles(np.zeros(20))
        fit = ThumbRetargeter(self.robot.design, self.robot.ranges)
        angles, info = fit.solve(points, 'Right', np.zeros(20), np.zeros(4))
        self.assertEqual(info['wrap_weight'], 0)
        self.assertLess(np.max(np.abs(angles)), .1)
