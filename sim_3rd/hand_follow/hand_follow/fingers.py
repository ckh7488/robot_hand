"""Independent fingertip/curvature fitting across different finger proportions.

Open/fist calibration adjusts a fingertip workspace, not twenty angle gains.
The measured tip still moves within that workspace: intermediate gestures are
not a replay of the authored fist. This is a bounded kinematic preview only.
"""
import hashlib
import json
import numpy as np
from .retarget import FINGERS, palm_frame


def geometry_signature(design):
    payload = {key: design.get(key) for key in ('fingers', 'fist_deg', 'joint_ranges_deg')}
    return hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()


def observations(world, handedness):
    p = np.asarray(world, float)
    q = (p - p[0]) @ palm_frame(p, handedness != 'Left')
    tips, lengths = [], []
    for indices in FINGERS.values():
        a, b, c, d = indices
        forward = np.array([q[a, 0], q[a, 1], 0.])
        norm = np.linalg.norm(forward)
        if norm < .001:
            raise ValueError('Degenerate finger base direction')
        forward /= norm
        side = np.array([forward[1], -forward[0], 0.])
        proximal = q[b] - q[a]
        spread = np.degrees(np.arctan2(proximal @ side, proximal @ forward))
        spread = np.radians(np.clip((spread + 90) % 180 - 90, -60, 60))
        local_forward = np.cos(spread)*forward + np.sin(spread)*side
        local_side = np.cos(spread)*side - np.sin(spread)*forward
        spans = np.linalg.norm(np.diff(q[indices], axis=0), axis=1)
        if np.any(spans < .001):
            raise ValueError('Degenerate finger landmarks')
        v = q[d] - q[a]
        tips.append(np.array([v @ local_side, v @ local_forward, v[2]]) / spans.sum())
        lengths.append(spans * 1000)
    return dict(tips=np.array(tips), lengths_mm=np.array(lengths))


def planar_chain(lengths, radians):
    angles = np.cumsum(radians)
    vectors = np.column_stack((np.cos(angles), np.sin(angles))) * lengths[:, None]
    points = np.cumsum(vectors, axis=0)
    derivatives = np.column_stack((-vectors[:, 1], vectors[:, 0]))
    jacobian = np.cumsum(derivatives[::-1], axis=0)[::-1].T
    return points, jacobian


def fit_finger(lengths, target, prior, limits, previous=None, weak_tip=False, floor=None, continuous=False):
    """Small bounded damped least-squares solve, with an analytic Jacobian."""
    low, high = np.radians(limits).T
    prior = np.clip(np.radians(prior), low, high)
    weights = np.array([.20, .35, .08 if weak_tip else .18])
    regularizer = np.eye(3) * .015
    prior_matrix = np.diag(weights)
    previous_q = np.radians(previous) if previous is not None else None

    def evaluate(q):
        points, jac = planar_chain(lengths, q)
        residual = np.r_[(points[-1] - target) / 20, (q - prior) * weights]
        matrix = np.vstack((jac / 20, prior_matrix))
        if continuous and previous_q is not None:
            # A tip has many inverse-kinematic solutions. Resolve the null
            # space toward the previous configuration, not a new seed branch.
            residual = np.r_[residual, (q-previous_q)*.35]
            matrix = np.vstack((matrix, np.eye(3)*.35))
        # A fingertip point guard does not certify shell or inter-finger clearance.
        if floor is not None and points[-1, 1] < floor:
            residual = np.r_[residual, (points[-1, 1] - floor) / 2]
            matrix = np.vstack((matrix, jac[1] / 2))
        return residual, matrix, points

    seeds = [prior]
    if previous is not None:
        seeds.append(np.clip(np.radians(previous), low, high))
    evaluated = [(seed, evaluate(seed)) for seed in seeds]
    q, (residual, jacobian, points) = min(evaluated, key=lambda pair: pair[1][0] @ pair[1][0])
    q = q.copy()
    for _ in range(10):
        step = np.linalg.solve(jacobian.T @ jacobian + regularizer,
                               -jacobian.T @ residual)
        step = np.clip(step, -.35, .35)
        cost = residual @ residual
        accepted = False
        for fraction in (1., .5, .25):
            candidate = np.clip(q + step * fraction, low, high)
            r, matrix, candidate_points = evaluate(candidate)
            if r @ r < cost - 1e-12:
                change = np.max(np.abs(candidate - q))
                q, accepted = candidate, True
                residual, jacobian, points = r, matrix, candidate_points
                break
        if not accepted or change < 1e-5:
            break
    return np.degrees(q), float(np.linalg.norm(points[-1] - target)), points


class FingerRetargeter:
    def __init__(self, design, ranges):
        self.design = design
        self.ranges = np.asarray(ranges)
        self.signature = geometry_signature(design)
        self.fist = np.asarray(design.get('fist_deg', [0,80,95,70]*4), float)
        self.previous = None

    def reset(self):
        self.previous = None

    def solve(self, observed, raw, calibration, continuous=False):
        profile = calibration.position_profile
        calibrated = bool(profile and profile.get('cad_signature') == self.signature)
        closed_calibrated = calibrated and 'fist_tips' in profile
        output = np.asarray(raw[:16], float).copy()
        information = []
        for fi, config in enumerate(self.design['fingers']):
            start = fi * 4
            lengths = np.array(config['links'], float)
            total = lengths.sum()
            tip = observed['tips'][fi]
            opened = np.array(profile['open_tips'][fi]) if calibrated else np.array([0., 1., 0.])
            progress = 0.
            target = np.array([0., total, 0.]) + (tip - opened) * total
            if closed_calibrated:
                closed = np.array(profile['fist_tips'][fi])
                curl = float(np.clip((np.linalg.norm(opened)-np.linalg.norm(tip)) /
                    max(np.linalg.norm(opened)-np.linalg.norm(closed), .1), 0, 1))
                fist_root = (profile['fist_flex_deg'][fi][0] if 'fist_flex_deg' in profile else
                    self.fist[start+1]/calibration.gain[start+1]+calibration.offset[start+1])
                root_fold = float(np.clip((raw[start+1]-calibration.offset[start+1]) /
                    max(fist_root-calibration.offset[start+1], 15), 0, 1))
                progress = min(curl, root_fold)
                contact_blend = progress * progress * (3 - 2 * progress)
                blend = curl * curl * (3 - 2 * curl)
                anchor = planar_chain(lengths, np.radians(self.fist[start+1:start+4]))[0][-1]
                projected = np.array([0., total, 0.]) + (closed - opened) * total
                target += contact_blend * (np.r_[0., anchor] - projected)
            else:
                progress = float(np.clip((raw[start+2]+raw[start+3]-20)/145, 0, 1))
                blend = progress * progress * (3 - 2 * progress)
                contact_blend = blend
            prior = raw[start+1:start+4] - calibration.offset[start+1:start+4] * (1-blend)
            weak_tip = bool(start+3 in calibration.coupled_dips or
                abs(calibration.offset[start+3]) > 30 or calibration.gain[start+3] > 2)
            spread = (raw[start] - calibration.offset[start]) * (1-contact_blend)
            if continuous:
                # Near 90-degree MCP flexion, planar projection has no stable
                # azimuth. Its sign flip must not become a spread command.
                spread *= 1-np.clip((abs(raw[start+1])-55)/25, 0, 1)
            spread = float(np.clip(spread, *self.ranges[start]))
            planar_target = target[1:].copy()
            # Preserve signed root-relative Y, including a tip curled behind MCP.
            # Keep a curled tip point on the palm side of the cover surface.
            floor = None
            if planar_target[0] < -5 and progress > .5 and not continuous:
                floor = self.design.get('palm_cover_front_z_mm', 14) + 2 - config['root'][2]
                planar_target[1] = max(planar_target[1], floor)
            previous = self.previous[start+1:start+4] if self.previous is not None else None
            angles, error, points = fit_finger(lengths, planar_target, prior,
                self.ranges[start+1:start+4], previous, weak_tip, floor, continuous)
            output[start:start+4] = np.r_[spread, angles]
            information.append(dict(finger=config['name'], progress=progress,
                target_yz_mm=planar_target, tip_error_mm=error, weak_dip=weak_tip,
                tip_yz_mm=points[-1], fingertip_guard_mm=floor,
                active_limits=[start+1+i for i,value in enumerate(angles)
                    if np.min(np.abs(self.ranges[start+1+i]-value)) < .25]))
        self.previous = output.copy()
        return output, dict(calibrated=closed_calibrated, fingers=information,
            estimated_joints=[4*i+3 for i,v in enumerate(information) if v['weak_dip']],
            active_limits=[j for row in information for j in row['active_limits']])
