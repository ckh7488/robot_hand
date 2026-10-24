import tempfile
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import numpy as np
from hand_follow.motion import MotionStabilizer
from hand_follow.pipeline import PosePipeline
from hand_follow.retarget import joint_ranges
from hand_follow.synthetic import landmarks_from_angles
from hand_follow.tracker import HandTracker


class ContinuousMotion(unittest.TestCase):
    def test_single_frame_outlier_is_rejected_and_other_fingers_stay_still(self):
        motion=MotionStabilizer(joint_ranges())
        target=np.zeros(20);target[2]=50
        for i in range(150):out=motion(target,i/30)
        baseline=out.copy()
        spike=target.copy();spike[2]=95;spike[6]=80
        out=motion(spike,150/30)
        np.testing.assert_allclose(out,baseline,atol=.1)
        for i in range(151,165):out=motion(target,i/30)
        self.assertLess(abs(out[6]),.01)

    def test_step_reaches_target_without_large_overshoot_or_exceeding_speed(self):
        motion=MotionStabilizer(joint_ranges())
        motion(np.zeros(20),0)
        target=np.zeros(20);target[2]=90;target[18]=70
        previous=np.zeros(20)
        outputs=[]
        for i in range(1,46):
            out=motion(target,i/60)
            self.assertTrue(np.all(np.abs(out-previous)<=motion.speed/60+1e-6))
            previous=out;outputs.append(out)
        self.assertGreater(outputs[35][2],88)
        self.assertLess(np.max(np.array(outputs)[:,2]),90.1)

    def test_tracking_loss_and_reacquisition_preserve_continuity(self):
        design={'thumb':{'base_yaw_deg':-40,'second_yaw_deg':25}}
        with tempfile.TemporaryDirectory() as directory:
            pipe=PosePipeline(design,joint_ranges(),Path(directory)/'cal.json',stabilize=True)
            closed=np.array([0,75,90,65]*4+[0]*4,float)
            detection=lambda a:dict(world=landmarks_from_angles(a),handedness='Right',score=.99)
            for i in range(100):old=pipe.update(detection(closed),i/30,i/30)['command_deg']
            held=pipe.update(None,4.,4.)['command_deg']
            np.testing.assert_equal(held,old)
            new=pipe.update(detection(np.zeros(20)),4.033,4.033)['command_deg']
            self.assertLess(np.max(np.abs(new-old)),2)
            for i in range(1,60):new=pipe.update(detection(np.zeros(20)),4.033+i/30,4.033+i/30)['command_deg']
            self.assertLess(np.max(np.abs(new[:16])),1)

    def test_small_stationary_noise_is_suppressed(self):
        rng=np.random.default_rng(34)
        motion=MotionStabilizer(joint_ranges())
        out=[]
        for i in range(500):
            target=np.zeros(20);target[2]=45+rng.normal(0,2)
            out.append(motion(target,i/30)[2])
        self.assertLess(np.std(out[150:]),.8)
        self.assertLess(abs(np.mean(out[150:])-45),1)


class HandIdentity(unittest.TestCase):
    def test_continuous_hand_does_not_flip_identity_after_five_bad_labels(self):
        tracker=HandTracker.__new__(HandTracker)
        tracker.mp=NS(Image=lambda **kwargs:None,ImageFormat=NS(SRGB=0))
        tracker.last_ts=-1;tracker.wrist=None;tracker.label=None
        tracker.pending_label=None;tracker.pending_count=0;tracker.last_seen_ms=-10000
        points=[NS(x=float(i%5)*.01,y=float(i//5)*.01,z=0.) for i in range(21)]
        current=['Right']
        tracker.det=NS(detect_for_video=lambda img,t:NS(hand_landmarks=[points],
            hand_world_landmarks=[points],handedness=[[NS(category_name=current[0],score=.98)]]))
        self.assertIsNone(tracker.process(None,0))
        self.assertIsNone(tracker.process(None,33))
        self.assertEqual(tracker.process(None,66)['handedness'],'Right')
        current[0]='Left'
        for t in range(99,450,33):
            self.assertEqual(tracker.process(None,t)['handedness'],'Right')
        self.assertIsNone(tracker.process(None,1500))
        self.assertIsNone(tracker.process(None,1533))
        self.assertEqual(tracker.process(None,1566)['handedness'],'Left')


if __name__=='__main__':unittest.main()
