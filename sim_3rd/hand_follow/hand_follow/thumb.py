"""Fit the CAD thumb to observed positions, including an outside-fist wrap.

Human thumbs roll about axes this CAD does not have. A projected hinge angle
therefore cannot represent their motion. This bounded fit follows the thumb tip
and intermediate point instead. The outside-fist clearance is a geometric
preview heuristic, not a collision solver or a change to the source CAD.
"""
import numpy as np
from scipy.optimize import least_squares
from .retarget import palm_frame


def rz(degrees):
    c, s = np.cos(np.radians(degrees)), np.sin(np.radians(degrees))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rx(degrees):
    c, s = np.cos(np.radians(degrees)), np.sin(np.radians(degrees))
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def chain(config, angles):
    """Four joint/tip positions in CAD millimetres; same frames as CadHand."""
    points = [np.array(config['root'], float)]
    rotation = rz(config.get('base_yaw_deg', 0) - angles[0]) @ rx(angles[1])
    for i, length in enumerate(config['links']):
        if i:
            if i == 1:
                rotation = rotation @ rz(config.get('second_yaw_deg', 0))
            rotation = rotation @ rx(angles[i + 1])
        points.append(points[-1] + rotation @ [0, length, 0])
    return np.array(points)


def smoothstep(value):
    t = np.clip(value, 0, 1)
    return t * t * (3 - 2 * t)


class ThumbRetargeter:
    def __init__(self, design, ranges):
        self.design = design
        self.limits = np.asarray(ranges)[16:]
        self.previous = None

    def reset(self):
        self.previous = None

    def solve(self, world, handedness, finger_angles, initial):
        p = np.asarray(world, float)
        q = (p - p[0]) @ palm_frame(p, handedness != 'Left') * 1000
        thumb = self.design['thumb']
        directions = np.diff(q[1:5], axis=0)
        lengths = np.linalg.norm(directions, axis=1)
        if np.any(lengths < 1):
            raise ValueError('Degenerate thumb landmarks')
        target = np.cumsum(directions / lengths[:, None] * np.array(thumb['links'])[:, None], axis=0)
        target += thumb['root']

        # Preserve relative contact location as the thumb crosses folded fingers.
        # Bone directions alone put a human-size thumb inside this wider CAD palm.
        width = max(np.linalg.norm(q[5] - q[17]), 20)
        fingers = self.design['fingers']
        robot_width = abs(fingers[0]['root'][0] - fingers[3]['root'][0])
        scale = np.array([robot_width / width,
                          fingers[1]['root'][1] / max(q[9, 1], 30),
                          fingers[1]['root'][1] / max(q[9, 1], 30)])
        chains = [chain(config, finger_angles[i * 4:i * 4 + 4]) for i, config in enumerate(fingers)]
        closure = smoothstep((np.median(finger_angles[[2, 6, 10, 14]]) - 35) / 45)
        across = smoothstep((q[5, 0] - q[4, 0]) / (width * .5))
        wrap = float(closure * across)
        relative = chains[0][1] + (q[2:5] - q[6]) * scale
        target = (1 - wrap) * target + wrap * relative
        # Approximate outer surface: folded index/middle hinges plus 22 mm for
        # both visible shells. Only active for an observed crossing thumb.
        top = max(np.max(points[1:3, 2]) for points in chains[:2]) + 22
        target[-1, 2] += wrap * max(0, top - target[-1, 2])

        lower, upper = self.limits.T
        seed = np.clip(initial if self.previous is None else self.previous, lower + 1e-6, upper - 1e-6)
        weights = np.array([.10, .25, 2.0])[:, None]

        def residual(angles):
            positional = ((chain(thumb, angles)[1:] - target) * weights / 50).ravel()
            # Small regularizer resolves redundant solutions without hiding tip error.
            return np.r_[positional, (angles - seed) / 180 * .03]

        fit = least_squares(residual, seed, bounds=(lower, upper), max_nfev=16,
                            ftol=2e-4, xtol=2e-4, gtol=2e-4)
        if not np.isfinite(fit.x).all():
            raise ValueError('Thumb fit produced invalid angles')
        self.previous = fit.x.copy()
        error = float(np.linalg.norm(chain(thumb, fit.x)[-1] - target[-1]))
        return fit.x, dict(target_mm=target[-1], tip_error_mm=error, wrap_weight=wrap)
