"""Check that the diagnostic drawing cannot change the inferred hand geometry."""
import unittest
import numpy as np
from render_comparison import rigid_view, skeleton
from render_mesh import mesh_view


class DisplayTests(unittest.TestCase):
    def setUp(self):
        rng = np.random.default_rng(746)
        self.points = rng.normal(size=(21,3))*.035
        self.points[0] = [0,0,0]
        self.points[9] = [0,.08,0]
        self.points[5] = [.035,.07,0]
        self.points[17] = [-.035,.07,0]

    def test_rigid_views_preserve_distances_and_chirality(self):
        for handedness in [1,-1]:
            p = self.points*[handedness,1,1]
            original = p.copy()
            distances = np.linalg.norm(p[:,None]-p[None,:],axis=-1)
            volume = np.linalg.det((p[[1,5,9]]-p[0]).T)
            for side in [False,True]:
                q,r = rigid_view(p,side)
                np.testing.assert_allclose(r.T@r,np.eye(3),atol=1e-12)
                self.assertAlmostEqual(np.linalg.det(r),1)
                np.testing.assert_allclose(np.linalg.norm(q[:,None]-q[None,:],axis=-1),distances,atol=1e-12)
                self.assertAlmostEqual(np.linalg.det((q[[1,5,9]]-q[0]).T),volume)
                skeleton(p,side)
                np.testing.assert_array_equal(p,original)

    def test_invalid_geometry_is_not_invented(self):
        for bad in [np.zeros((21,3)), np.ones((20,3)), np.full((21,3),np.nan)]:
            with self.assertRaises(ValueError):
                rigid_view(bad)
        image = skeleton(None)
        self.assertEqual(image.size,(570,380))

    def test_surface_occlusion_is_independent_of_triangle_order(self):
        far=np.array([[-.025,.045,0],[.025,.045,0],[0,.09,.01]])
        near=far.copy();near[:,2]=.03
        both=np.vstack([far,near])
        faces=np.array([[0,1,2],[3,4,5]])
        expected=np.asarray(mesh_view(self.points,near,[[0,1,2]],size=(120,160)))
        actual=np.asarray(mesh_view(self.points,both,faces,size=(120,160)))
        reverse=np.asarray(mesh_view(self.points,both,faces[::-1],size=(120,160)))
        np.testing.assert_array_equal(actual,expected)
        np.testing.assert_array_equal(actual,reverse)
        self.assertFalse(np.array_equal(actual,np.asarray(mesh_view(self.points,far,[[0,1,2]],size=(120,160)))))


if __name__=='__main__':
    unittest.main()
