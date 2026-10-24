import os
from pathlib import Path
import unittest
import numpy as np
from hand_follow.cad_hand import CadHand
from hand_follow.pipeline import FIST
from hand_follow.retarget import NAMES
REPO = Path(os.environ.get('HAND_FOLLOW_REPO', Path(__file__).resolve().parents[3]))

class CadMapping(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.robot = CadHand(REPO)

    @classmethod
    def tearDownClass(cls):
        cls.robot.close()

    def setUp(self):
        self.robot.set(np.zeros(20))

    def test_every_cad_part_is_present(self):
        self.assertEqual(len(self.robot.items), 120)
        self.assertEqual(self.robot.model.ngeom, 120)
        self.assertEqual(self.robot.model.njnt, 20)

    def test_neutral_meshes_match_original_stl_bounds(self):
        dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
        m, d = (self.robot.model, self.robot.data)
        for item in self.robot.items:
            path = self.robot.source / item['mesh']
            original = np.frombuffer(path.read_bytes()[84:], dtype=dtype)['vertices'].reshape(-1, 3) * 0.001
            gid = m.geom(item['name']).id
            mesh = m.geom_dataid[gid]
            local = m.mesh_vert[m.mesh_vertadr[mesh]:m.mesh_vertadr[mesh] + m.mesh_vertnum[mesh]]
            world = local @ d.geom_xmat[gid].reshape(3, 3).T + d.geom_xpos[gid]
            np.testing.assert_allclose([world.min(0), world.max(0)], [original.min(0), original.max(0)], atol=2e-07)

    def test_joint_moves_only_correct_finger_and_keeps_palm_fixed(self):
        m, d = (self.robot.model, self.robot.data)
        before = d.geom_xpos.copy()
        pose = np.zeros(20)
        pose[2] = 70
        self.robot.set(pose)
        for item in self.robot.items:
            gid = m.geom(item['name']).id
            if not item['name'].startswith('index_'):
                np.testing.assert_allclose(d.geom_xpos[gid], before[gid], atol=1e-12)
        tip = m.site('index_dip_tip').id
        self.assertGreater(d.site_xpos[tip, 2], 0.05)

    def test_fist_reaches_named_commands_and_tips_curl_back(self):
        m, d = (self.robot.model, self.robot.data)
        before = {finger: d.site_xpos[m.site(finger + '_dip_tip').id].copy() for finger in ('index', 'middle', 'ring', 'little')}
        self.robot.set(FIST)
        np.testing.assert_allclose(self.robot.angles, FIST, atol=1e-10)
        for finger in before:
            tip = d.site_xpos[m.site(finger + '_dip_tip').id]
            self.assertLess(tip[1], before[finger][1] - 0.1)
            self.assertGreater(tip[2], 0.015)
