import unittest
import numpy as np
from metrics import mesh_crossings,match_and_measure,bend_angles,inter_mesh_crossings
from audit_regions import detail


class Checks(unittest.TestCase):
    def test_disjoint_triangles(self):
        v=np.array([[0,0,0],[1,0,0],[0,1,0],[0,0,2],[1,0,2],[0,1,2]],float)
        count,_=mesh_crossings(v,np.array([[0,1,2],[3,4,5]]))
        self.assertEqual(count,0)

    def test_surface_crossing(self):
        v=np.array([[0,0,0],[1,0,0],[0,1,0],[.2,.2,-1],[.3,.2,1],[.2,.3,1]],float)
        count,hit=mesh_crossings(v,np.array([[0,1,2],[3,4,5]]))
        self.assertEqual(count,1);self.assertEqual(len(hit),2)

    def test_contact_is_not_crossing(self):
        v=np.array([[0,0,0],[1,0,0],[0,1,0],[.2,.2,0],[.3,.2,1],[.2,.3,1]],float)
        count,_=mesh_crossings(v,np.array([[0,1,2],[3,4,5]]));self.assertEqual(count,0)

    def test_adjacent_faces_are_excluded(self):
        v=np.array([[0,0,0],[1,0,0],[0,1,0],[.2,.2,1]],float)
        count,_=mesh_crossings(v,np.array([[0,1,2],[0,1,3]]));self.assertEqual(count,0)

    def hand(self):
        rng=np.random.default_rng(2);xyz=rng.normal(size=(21,3))*.03
        return dict(xyz=xyz,uv=xyz[:,:2]*1000+100,valid=np.ones(21,bool),match_mask=np.ones(21,bool),side=1)

    def test_root_translation_only(self):
        gt=self.hand();pred=dict(xyz=gt['xyz']+np.array([1,2,3]),uv=gt['uv'],side=1)
        m=match_and_measure([pred],[gt]);self.assertLess(m['matches'][0]['mpjpe_mm'],1e-8)
        pred['xyz']=gt['xyz']*2
        self.assertGreater(match_and_measure([pred],[gt])['matches'][0]['mpjpe_mm'],10)

    def test_missing_hands_are_not_zero_error(self):
        m=match_and_measure([],[self.hand()]);self.assertEqual(m['missed_hands'],1);self.assertEqual(m['matches'],[])

    def test_different_finger_regions_exclude_palm_and_same_finger(self):
        v=np.array([[0,0,0],[1,0,0],[0,1,0],[.2,.2,-1],[.3,.2,1],[.2,.3,1]],float)
        f=np.array([[0,1,2],[3,4,5]]);c=np.array([0,1])
        self.assertEqual(detail(v,f,c,np.array([1,2]))[0].sum(),1)
        self.assertEqual(detail(v,f,c,np.array([1,1]))[0].sum(),0)
        self.assertEqual(detail(v,f,c,np.array([0,1]))[0].sum(),0)

    def test_two_meshes_cross_or_are_separated(self):
        a=np.array([[0,0,0],[1,0,0],[0,1,0]],float)
        b=np.array([[.2,.2,-1],[.3,.2,1],[.2,.3,1]],float);f=np.array([[0,1,2]])
        self.assertEqual(inter_mesh_crossings(a,b,f)[0],1)
        self.assertEqual(inter_mesh_crossings(a,b+10,f)[0],0)


if __name__=='__main__':unittest.main()
