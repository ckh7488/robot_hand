"""Cross-proportion fitting, invariance and high-gain regression checks."""
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from hand_follow.fingers import FingerRetargeter, observations, geometry_signature, planar_chain
from hand_follow.thumb import chain, chain_jacobian
from hand_follow.retarget import Calibration, raw_angles, NAMES
from hand_follow.pipeline import PosePipeline
from hand_follow.synthetic import landmarks_from_angles

REPO = Path(__file__).resolve().parents[3]
DESIGN = json.loads((REPO/'cad/webcam_articulation_v1/design.json').read_text())
RANGES = np.array([DESIGN['joint_ranges_deg'][n if n.startswith('thumb') else n.split('_',1)[1]] for n in NAMES])
FIST = np.array([0,70,90,70]*4+[0]*4, float)


def profile(opened=None, fist=None):
    opened = np.zeros(20) if opened is None else opened
    fist = FIST if fist is None else fist
    cal = Calibration()
    cal.offset = opened.copy()
    cal.position_profile = dict(schema=1, cad_signature=geometry_signature(DESIGN),
        open_tips=observations(landmarks_from_angles(opened),'Right')['tips'].tolist(),
        fist_tips=observations(landmarks_from_angles(fist),'Right')['tips'].tolist(),
        fist_flex_deg=fist[:16].reshape(4,4)[:,1:].tolist())
    return cal


def mapped(points, cal, hand='Right'):
    return FingerRetargeter(DESIGN,RANGES).solve(observations(points,hand),raw_angles(points,hand),cal)


class PositionMapping(unittest.TestCase):
    def test_open_is_straight_and_fist_tip_targets_remain_near_palm(self):
        cal=profile()
        result,_=mapped(landmarks_from_angles(np.zeros(20)),cal)
        np.testing.assert_allclose(result,np.zeros(16),atol=1e-6)
        result,info=mapped(landmarks_from_angles(FIST),cal)
        self.assertTrue(info['calibrated'])
        for row in info['fingers']:
            self.assertLess(row['tip_error_mm'],8)
            self.assertGreater(row['tip_yz_mm'][1],row['fingertip_guard_mm']-.2)

    def test_individual_fingers_and_distal_motion_are_not_one_fist_animation(self):
        cal=profile()
        pose=np.zeros(20);pose[5:8]=[25,45,20]
        middle,_=mapped(landmarks_from_angles(pose),cal)
        np.testing.assert_allclose(middle[np.r_[0:4,8:16]],0,atol=1e-6)
        self.assertGreater(np.linalg.norm(middle[5:8]),20)
        changed=pose.copy();changed[7]+=30
        distal,_=mapped(landmarks_from_angles(changed),cal)
        self.assertGreater(abs(distal[7]-middle[7]),5)
        self.assertGreater(np.linalg.norm(distal[5:8]-middle[5:8]),8)

    def test_camera_rotation_and_left_hand_preserve_result(self):
        pose=FIST*.65;cal=profile()
        expected,_=mapped(landmarks_from_angles(pose),cal)
        rotation=Rotation.from_euler('xyz',[31,-48,62],degrees=True).as_matrix()
        for hand in ('Left','Right'):
            actual,_=mapped(landmarks_from_angles(pose,hand,rotation=rotation),cal,hand)
            np.testing.assert_allclose(actual,expected,atol=1e-6)

    def test_spread_and_root_only_flexion_do_not_turn_into_a_fist(self):
        cal=profile()
        for spread,root in ((20,0),(0,75),(15,40)):
            pose=np.array([spread,root,0,0]*4+[0]*4,float)
            result,info=mapped(landmarks_from_angles(pose),cal)
            np.testing.assert_allclose(result.reshape(4,4),np.tile([spread,root,0,0],(4,1)),atol=1e-5)
            self.assertLess(max(f['progress'] for f in info['fingers']),1e-6)

    def test_hook_without_root_flexion_does_not_activate_palm_contact_correction(self):
        pose=np.array([0,0,75,45]*4+[0]*4,float)
        result,info=mapped(landmarks_from_angles(pose),profile())
        self.assertLess(max(f['progress'] for f in info['fingers']),1e-6)
        self.assertLess(np.max(result.reshape(4,4)[:,1]),30)

    def test_changed_geometry_requires_new_open_calibration(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'cal.json';cal=profile()
            cal.position_profile['cad_signature']='old';cal.save(path)
            pipeline=PosePipeline(DESIGN,RANGES,path,retarget_mode='position')
            pipeline.start_calibration('fist')
            self.assertFalse(pipeline.open_calibrated)
            self.assertIsNone(pipeline.capture_kind)
            self.assertEqual(pipeline.calibration.position_profile,{})

    def test_old_individual_angle_gain_does_not_scale_position_command(self):
        cal=profile();points=landmarks_from_angles(FIST*.6)
        first,_=mapped(points,cal)
        cal.gain[2]=4;cal.gain[1]=.5
        second,_=mapped(points,cal)
        np.testing.assert_allclose(first,second,atol=1e-9)

    def test_unreliable_distal_landmark_has_less_influence_than_347_percent_gain(self):
        opened=np.zeros(20);opened[3]=42.5
        fist=FIST.copy();fist[3]=60
        cal=profile(opened,fist);cal.gain[3]=3.47
        reference=landmarks_from_angles(fist*.7+opened*.3)
        base,info=mapped(reference,cal)
        self.assertIn(3,info['estimated_joints'])
        old=cal.apply(raw_angles(reference))[3]
        new_change=[];old_change=[]
        for delta in (-.002,-.001,.001,.002):
            points=reference.copy();points[7,2]+=delta # tip stays put; uncertain middle landmark
            candidate,_=mapped(points,cal)
            new_change.append(abs(candidate[3]-base[3]))
            old_change.append(abs(cal.apply(raw_angles(points))[3]-old))
        self.assertLess(np.mean(new_change),np.mean(old_change)*.6)

    def test_previously_unobservable_dip_is_declared_estimated(self):
        cal=profile();cal.coupled_dips=[7]
        _,info=mapped(landmarks_from_angles(FIST),cal)
        self.assertIn(7,info['estimated_joints'])

    def test_position_calibration_capture_persists_geometry_profile(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'cal.json'
            pipeline=PosePipeline(DESIGN,RANGES,path,retarget_mode='position')
            for kind,pose in [('open',np.zeros(20)),('fist',FIST)]:
                points=landmarks_from_angles(pose);raw=raw_angles(points)
                observed=observations(points,'Right')
                pipeline.start_calibration(kind);start=pipeline.capture_started
                for elapsed in np.linspace(.5,1.9,30):pipeline._capture(raw,start+elapsed,observed)
            restored=Calibration.load(path)
            self.assertIn('fist_tips',restored.position_profile)
            self.assertEqual(restored.position_profile['cad_signature'],geometry_signature(DESIGN))

    def test_profile_rejects_invalid_coordinates(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'cal.json';cal=profile()
            cal.position_profile['open_tips'][0][0]=float('nan');cal.save(path)
            with self.assertRaises(ValueError):Calibration.load(path)


class Derivatives(unittest.TestCase):
    def test_planar_and_thumb_analytic_derivatives_match_central_difference(self):
        lengths=np.array([65,50,32]);q=np.radians([45,70,30]);_,jac=planar_chain(lengths,q)
        for i in range(3):
            step=np.zeros(3);step[i]=1e-6
            measured=(planar_chain(lengths,q+step)[0][-1]-planar_chain(lengths,q-step)[0][-1])/(2e-6)
            np.testing.assert_allclose(jac[:,i],measured,atol=1e-6)
        for pose in ([5,30,25,45],[-35,70,65,15],[20,0,0,0]):
            points,jac=chain_jacobian(DESIGN['thumb'],np.array(pose,float))
            np.testing.assert_allclose(points,chain(DESIGN['thumb'],pose),atol=1e-9)
            for i in range(4):
                step=np.zeros(4);step[i]=1e-4
                measured=(chain(DESIGN['thumb'],np.array(pose)+step)[1:]-chain(DESIGN['thumb'],np.array(pose)-step)[1:])/(2e-4)
                np.testing.assert_allclose(jac[:,:,i],measured,atol=1e-7)
