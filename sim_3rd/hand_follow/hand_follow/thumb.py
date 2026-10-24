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


def ry(degrees):
    c, s = np.cos(np.radians(degrees)), np.sin(np.radians(degrees))
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def chain(config, angles):
    """Four joint/tip positions in CAD millimetres; same frames as CadHand."""
    base = rz(config.get('base_yaw_deg', 0) - angles[0])
    points = [np.array(config['root'], float) + base @ config.get('flexion_offset', [0, 0, 0])]
    rotation = rz(config.get('base_yaw_deg', 0) - angles[0]) @ rx(angles[1])
    for i, length in enumerate(config['links']):
        if i:
            if i == 1:
                rotation = rotation @ rz(config.get('second_yaw_deg', 0)) @ ry(config.get('second_roll_deg', 0))
            rotation = rotation @ rx(angles[i + 1])
        points.append(points[-1] + rotation @ [0, length, 0])
    return np.array(points)


def smoothstep(value):
    t = np.clip(value, 0, 1)
    return t * t * (3 - 2 * t)


def chain_jacobian(config, angles):
    """Joint points and exact per-degree position derivatives of the four axes."""
    base = rz(config.get('base_yaw_deg', 0) - angles[0])
    root = np.array(config['root'], float)
    points = [root + base @ config.get('flexion_offset', [0, 0, 0])]
    origins = [root, points[0]]
    axes = [np.array([0., 0., -1.]), base[:, 0]]
    rotation = base @ rx(angles[1])
    for i, length in enumerate(config['links']):
        if i:
            if i == 1:
                rotation = rotation @ rz(config.get('second_yaw_deg', 0)) @ ry(config.get('second_roll_deg', 0))
            origins.append(points[-1])
            axes.append(rotation[:, 0])
            rotation = rotation @ rx(angles[i + 1])
        points.append(points[-1] + rotation @ [0, length, 0])
    points = np.array(points)
    jacobian = np.zeros((3, 3, 4))
    for point in range(1, 4):
        active = point + 1
        jacobian[point-1, :, :active] = np.cross(
            np.array(axes[:active]), points[point]-np.array(origins[:active])).T * (np.pi/180)
    return points, jacobian


class ThumbRetargeter:
    def __init__(self, design, ranges):
        self.design = design
        self.limits = np.asarray(ranges)[16:]
        self.previous = None
        self.previous_wrap = None

    def reset(self):
        self.previous = None
        self.previous_wrap = None

    def solve(self, world, handedness, finger_angles, initial, dt=None):
        p = np.asarray(world, float)
        q = (p - p[0]) @ palm_frame(p, handedness != 'Left') * 1000
        thumb = self.design['thumb']
        directions = np.diff(q[1:5], axis=0)
        lengths = np.linalg.norm(directions, axis=1)
        if np.any(lengths < 1):
            raise ValueError('Degenerate thumb landmarks')
        target = np.cumsum(directions / lengths[:, None] * np.array(thumb['links'])[:, None], axis=0)
        target += chain(thumb, np.zeros(4))[0]

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
        if dt is not None and self.previous_wrap is not None:
            wrap = self.previous_wrap + dt/(.045+dt)*(wrap-self.previous_wrap)
        self.previous_wrap = wrap
        relative = chains[0][1] + (q[2:5] - q[6]) * scale
        target = (1 - wrap) * target + wrap * relative
        # Approximate outer surface: folded index/middle hinges plus 22 mm for
        # both visible shells. Only active for an observed crossing thumb.
        top = max(np.max(points[1:3, 2]) for points in chains[:2]) + 22
        target[-1, 2] += wrap * max(0, top - target[-1, 2])

        lower, upper = self.limits.T
        seed = np.clip(initial if self.previous is None else self.previous, lower + 1e-6, upper - 1e-6)
        continuity = .03/180
        if dt is not None and self.previous is not None:
            # Retain the current inverse-kinematics branch when the tip has
            # multiple solutions, instead of jumping to an equivalent pose.
            step = np.array([480,480,600,600])*np.clip(dt,.015,.1)
            lower, upper = np.maximum(lower,seed-step), np.minimum(upper,seed+step)
            continuity = .18/30
        weights = np.array([.35, .65, 2.0] if dt is not None else [.10, .25, 2.0])[:, None]

        cache = [None, None, None]

        def evaluate(angles):
            if cache[0] is None or not np.array_equal(cache[0], angles):
                points, jacobian = chain_jacobian(thumb, angles)
                cache[:] = [angles.copy(), points, jacobian]
            return cache[1], cache[2]

        def residual(angles):
            points, _ = evaluate(angles)
            positional = ((points[1:] - target) * weights / 50).ravel()
            # Small regularizer resolves redundant solutions without hiding tip error.
            return np.r_[positional, (angles - seed) * continuity]

        def jacobian(angles):
            _, derivative = evaluate(angles)
            return np.vstack(((derivative * weights[:, :, None] / 50).reshape(9, 4),
                              np.eye(4) * continuity))

        fit = least_squares(residual, seed, jac=jacobian, bounds=(lower, upper), max_nfev=16,
                            ftol=2e-4, xtol=2e-4, gtol=2e-4)
        if not np.isfinite(fit.x).all():
            raise ValueError('Thumb fit produced invalid angles')
        self.previous = fit.x.copy()
        error = float(np.linalg.norm(chain(thumb, fit.x)[-1] - target[-1]))
        return fit.x, dict(target_mm=target[-1], tip_error_mm=error, wrap_weight=wrap)
