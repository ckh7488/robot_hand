"""Round-trip and invariance tests for the landmark -> joint angle conversion (no camera needed)."""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from hand_follow.retarget import raw_angles, NAMES, Calibration
from hand_follow.synthetic import landmarks_from_angles, demo_angles
from hand_follow.filters import OneEuro

RNG = np.random.default_rng(1)


def random_pose():
    a = np.zeros(20)
    for f in range(4):
        a[4*f:4*f+4] = [RNG.uniform(-15, 15), RNG.uniform(-15, 85), RNG.uniform(-10, 100), RNG.uniform(-10, 80)]
    a[16:20] = [RNG.uniform(-40, 40), RNG.uniform(-10, 80), RNG.uniform(-15, 80), RNG.uniform(-10, 80)]
    return a


class Retarget(unittest.TestCase):
    def test_round_trip_right_hand(self):
        worst = 0
        for _ in range(300):
            a = random_pose()
            got = raw_angles(landmarks_from_angles(a), "Right")
            worst = max(worst, np.abs(got - a).max())
        self.assertLess(worst, 1e-6)

    def test_any_camera_orientation(self):
        for _ in range(100):
            a = random_pose()
            R = Rotation.random(random_state=int(RNG.integers(1e9))).as_matrix()
            got = raw_angles(landmarks_from_angles(a, rotation=R), "Right")
            self.assertLess(np.abs(got - a).max(), 1e-6)

    def test_left_hand_gives_same_anatomical_angles(self):
        for _ in range(100):
            a = random_pose()
            got = raw_angles(landmarks_from_angles(a, "Left"), "Left")
            self.assertLess(np.abs(got - a).max(), 1e-6)

    def test_wrong_handedness_only_flips_out_of_plane_signs(self):
        a = random_pose()
        got = raw_angles(landmarks_from_angles(a, "Left"), "Right")
        flip = [1, 2, 3, 5, 6, 7, 9, 10, 11, 13, 14, 15, 17, 18, 19]
        self.assertLess(np.abs(got[flip] + a[flip]).max(), 1e-6)

    def test_noise_sensitivity_reported(self):
        # 2 mm landmark noise (MediaPipe-like) -> report typical angle noise per joint type
        a = demo_angles(0.0)
        errs = np.array([raw_angles(landmarks_from_angles(a, noise_m=0.002, rng=RNG), "Right") - a
                         for _ in range(400)])
        sd = errs.std(axis=0)
        print("\n2 mm landmark noise -> angle std (deg):",
              {n: round(float(s), 1) for n, s in zip(NAMES, sd) if n.startswith(("index", "thumb"))})
        self.assertTrue(np.all(sd < 20))

    def test_no_bias_at_straight_joints(self):
        a = np.zeros(20)
        errs = np.array([raw_angles(landmarks_from_angles(a, noise_m=0.002, rng=RNG), "Right")
                         for _ in range(400)])
        bias = errs.mean(axis=0)
        self.assertLess(np.abs(bias[[2, 3, 6, 7, 10, 11, 14, 15, 18, 19]]).max(), 1.5)

    def test_calibration_zeroes_open_hand(self):
        a = np.zeros(20); a[[0, 4, 8, 12]] = [4, 1, -2, -5]
        c = Calibration(); c.capture_open_hand(raw_angles(landmarks_from_angles(a), "Right"))
        self.assertLess(np.abs(c.apply(raw_angles(landmarks_from_angles(a), "Right"))).max(), 1e-6)


class Filter(unittest.TestCase):
    def test_one_euro_tracks_step_and_smooths_noise(self):
        f = OneEuro(1, min_cutoff_hz=1.5, beta=0.02)
        t = np.arange(0, 2, 1/120)
        x = np.where(t > 1, 90.0, 0.0) + RNG.normal(0, 1, len(t))
        y = np.array([f(np.array([xi]), ti)[0] for xi, ti in zip(x, t)])
        still = y[(t > 0.5) & (t < 1.0)]
        self.assertLess(still.std(), 0.5)
        reach = t[np.argmax(y > 72)] - 1.0
        print(f"\nOne Euro: 0->90 deg step reaches 72 deg after {reach*1000:.0f} ms at 120 fps")
        self.assertLess(reach, 0.06)


if __name__ == "__main__":
    unittest.main(verbosity=2)
