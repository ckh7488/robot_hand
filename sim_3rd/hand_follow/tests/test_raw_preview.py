"""The diagnostic view must not manufacture a plausible replacement hand."""
import unittest
import numpy as np
from hand_follow.raw_preview import display_points, render_raw, BACKGROUND
from hand_follow.synthetic import landmarks_from_angles


class RawPreview(unittest.TestCase):
    def test_every_view_preserves_all_pairwise_distances_and_chirality(self):
        angles=np.array([5,45,60,25]*4+[15,30,45,20],float)
        for hand in ('Right','Left'):
            raw=landmarks_from_angles(angles,handedness=hand)
            original=raw.copy()
            distances=np.linalg.norm(raw[:,None]-raw[None,:],axis=2)
            ids=[0,5,9,4]
            volume=np.linalg.det(raw[ids[1:]]-raw[ids[0]])
            for view in range(3):
                shown=display_points(raw,view)
                np.testing.assert_allclose(np.linalg.norm(shown[:,None]-shown[None,:],axis=2),distances,atol=1e-14)
                self.assertAlmostEqual(np.linalg.det(shown[ids[1:]]-shown[ids[0]]),volume,places=14)
            np.testing.assert_array_equal(raw,original)

    def test_lost_or_bad_detection_clears_the_previous_skeleton(self):
        detection={'world':landmarks_from_angles(np.zeros(20))}
        drawn,state=render_raw(detection)
        self.assertEqual(state,'tracked')
        blank,state=render_raw(None)
        self.assertEqual(state,'missing')
        self.assertTrue(np.all(blank==BACKGROUND))
        self.assertFalse(np.array_equal(drawn,blank))
        for bad in (np.zeros((21,3)),np.full((21,3),np.nan),np.zeros((20,3))):
            result,state=render_raw({'world':bad})
            self.assertEqual(state,'invalid')
            np.testing.assert_array_equal(result,blank)
