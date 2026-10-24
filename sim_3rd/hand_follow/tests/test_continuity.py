"""Behavioral regressions for unstable observations and tracking transitions."""
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from hand_follow.observation import ObservationGuard
from hand_follow.continuity import ContinuousPoseFilter
from hand_follow.synthetic import landmarks_from_angles
from hand_follow.retarget import joint_ranges, NAMES
from hand_follow.pipeline import PosePipeline


class ObservationChecks(unittest.TestCase):
    def test_changing_bone_length_holds_only_affected_finger(self):
        guard=ObservationGuard()
        opened=landmarks_from_angles(np.zeros(20))
        guard.update(opened,'Right',0)
        pose=np.zeros(20);pose[5:8]=[35,65,25]
        moved=landmarks_from_angles(pose)
        moved[8]=moved[7]+1.6*(moved[8]-moved[7])
        accepted,info=guard.update(moved,'Right',.033)
        self.assertEqual(info['held_fingers'],[0])
        np.testing.assert_allclose(accepted[5:9],opened[5:9],atol=1e-10)
        np.testing.assert_allclose(accepted[9:13],moved[9:13],atol=1e-10)

    def test_fast_curl_rotation_and_translation_do_not_count_as_bad_bones(self):
        guard=ObservationGuard()
        guard.update(landmarks_from_angles(np.zeros(20)),'Right',0)
        pose=np.array([10,85,95,70]*4+[15,60,40,50])
        rotation=Rotation.from_euler('xyz',[40,65,20],degrees=True).as_matrix()
        moved=landmarks_from_angles(pose,rotation=rotation)+[.2,-.3,.8]
        accepted,info=guard.update(moved,'Right',.033)
        self.assertEqual(info['held_fingers'],[])
        np.testing.assert_allclose(accepted,moved,atol=1e-10)

    def test_new_hand_after_loss_can_have_different_proportions(self):
        guard=ObservationGuard()
        p=landmarks_from_angles(np.zeros(20));guard.update(p,'Right',0)
        p[8]=p[7]+1.6*(p[8]-p[7])
        _,info=guard.update(p,'Right',2)
        self.assertEqual(info['held_fingers'],[])

    def test_initial_bad_proportion_does_not_freeze_hand_forever(self):
        guard=ObservationGuard()
        actual=landmarks_from_angles(np.zeros(20));bad=actual.copy()
        bad[8]=bad[7]+1.6*(bad[8]-bad[7])
        guard.update(bad,'Right',0)
        for i in range(1,5):accepted,info=guard.update(actual,'Right',i/30)
        self.assertEqual(info['held_fingers'],[])
        np.testing.assert_allclose(accepted,actual,atol=1e-10)

    def test_sudden_length_jump_is_detected_even_when_near_long_term_median(self):
        guard=ObservationGuard();p=landmarks_from_angles(np.zeros(20))
        for i in range(10):guard.update(p,'Right',i/30)
        short=p.copy();short[8]=short[7]+.75*(short[8]-short[7])
        guard.update(short,'Right',10/30)
        long=p.copy();long[8]=long[7]+1.2*(long[8]-long[7])
        _,info=guard.update(long,'Right',11/30)
        self.assertIn(0,info['held_fingers'])


class OutputContinuity(unittest.TestCase):
    def test_single_frame_angle_spike_does_not_move_other_fingers_or_spike_joint(self):
        filt=ContinuousPoseFilter(joint_ranges())
        zero=np.zeros(20)
        for i in range(20):filt(zero,i/30)
        spike=zero.copy();spike[2]=90
        actual=filt(spike,20/30)
        np.testing.assert_allclose(actual,zero,atol=1e-10)
        np.testing.assert_allclose(filt(zero,21/30),zero,atol=1e-10)

    def test_sustained_fast_motion_passes_without_367ms_slow_filter(self):
        filt=ContinuousPoseFilter(joint_ranges());filt(np.zeros(20),0)
        target=np.zeros(20);target[2]=90
        curve=np.array([filt(target,i/60)[2] for i in range(1,61)])
        self.assertLessEqual((np.flatnonzero(curve>=81)[0]+1)/60,.2)
        self.assertLess(curve.max(),90.001)
        self.assertGreater(curve[-1],89)

    def test_reacquisition_starts_from_last_displayed_pose(self):
        filt=ContinuousPoseFilter(joint_ranges());target=np.array([0,70,90,60]*4+[0]*4)
        for i in range(60):filt(target,i/30)
        before=filt.position.copy();filt.hold(4)
        after=filt(np.zeros(20),5)
        self.assertLessEqual(np.max(np.abs(after-before)),24.001)
        for i in range(1,31):after=filt(np.zeros(20),5+i/30)
        self.assertLess(np.max(np.abs(after)),.2)

    def test_still_noise_and_ramp_tracking_are_both_bounded(self):
        rng=np.random.default_rng(64);filt=ContinuousPoseFilter(joint_ranges())
        values=[]
        for i in range(400):
            q=np.zeros(20);q[2]=45+rng.normal(0,2)
            values.append(filt(q,i/30)[2])
        self.assertLess(np.std(values[100:]),1.2)
        errors=[]
        for i in range(1,61):
            q[2]=45+i*.6
            errors.append(abs(filt(q,(399+i)/30)[2]-q[2]))
        self.assertLess(np.percentile(errors,95),4)


class RobustPipeline(unittest.TestCase):
    def test_bad_distal_length_does_not_change_held_finger_command_target(self):
        repo=Path(__file__).resolve().parents[3]
        design=json.loads((repo/'cad/accepted_form_v1/design.json').read_text())
        ranges=np.array([design['joint_ranges_deg'][n if n.startswith('thumb') else n.split('_',1)[1]] for n in NAMES])
        with tempfile.TemporaryDirectory() as tmp:
            pipe=PosePipeline(design,ranges,Path(tmp)/'cal.json',retarget_mode='position')
            p=landmarks_from_angles(np.zeros(20))
            for i in range(20):before=pipe.update(dict(world=p,handedness='Right'),i/30,i/30)
            pose=np.zeros(20);pose[5:8]=[35,65,25]
            p=landmarks_from_angles(pose);p[8]=p[7]+1.6*(p[8]-p[7])
            after=pipe.update(dict(world=p,handedness='Right'),20/30,20/30)
            self.assertEqual(after['observation_quality']['held_fingers'],[0])
            np.testing.assert_allclose(after['mapped_deg'][:4],before['mapped_deg'][:4],atol=1e-8)
            self.assertGreater(np.max(after['mapped_deg'][4:8]),20)
