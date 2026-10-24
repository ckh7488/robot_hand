"""Map observed bends to robot posture without a fingertip-only objective.

Different thumb root geometry needs a calibrated posture correction. Four
observed feature residuals remain independent; no other finger drives the thumb.
This is gesture retargeting, not anatomical reconstruction or collision control.
"""
import hashlib
import json
import numpy as np
from .retarget import palm_frame


def geometry_signature(design):
    payload = {key: design.get(key) for key in ('fingers', 'thumb', 'fist_deg', 'joint_ranges_deg')}
    return hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()


def unit(value):
    a = np.asarray(value, float)
    norm = np.linalg.norm(a, axis=-1, keepdims=True)
    if not np.isfinite(a).all() or np.any(norm < 1e-9):
        raise ValueError('Degenerate bone direction')
    return a / norm


def thumb_directions(world, handedness):
    p = np.asarray(world, float)
    return unit(np.diff(p[1:5], axis=0) @ palm_frame(p, handedness != 'Left'))


def thumb_features(directions):
    """Palm-relative root azimuth/elevation and two internal bend magnitudes."""
    d = unit(directions)
    return np.array([np.degrees(np.arctan2(d[0, 0], d[0, 1])),
        np.degrees(np.arctan2(d[0, 2], np.hypot(d[0, 0], d[0, 1]))),
        np.degrees(np.arccos(np.clip(d[0] @ d[1], -1, 1))),
        np.degrees(np.arccos(np.clip(d[1] @ d[2], -1, 1)))])


def feature_difference(a, b):
    difference = np.asarray(a) - b
    difference[0] = (difference[0] + 180) % 360 - 180
    return difference


class ShapeRetargeter:
    def __init__(self, design, ranges):
        self.design = design
        self.ranges = np.asarray(ranges, float)
        self.fist = np.asarray(design['fist_deg'], float)
        self.previous = None
        self.finger_limits = []

    def reset(self):
        self.previous = None

    def fingers(self, raw, calibration):
        # A tip target must not exchange MCP bend for a saturated DIP. Every
        # joint keeps its calibrated bend, except explicitly unobserved DIPs.
        mapped = calibration.apply(raw)[:16]
        for start in range(0, 16, 4):
            closure = np.clip((max(abs(raw[start+1]), mapped[start+2])-35)/45, 0, 1)
            closure = closure * closure * (3-2*closure)
            mapped[start] *= 1-closure
        self.finger_limits = np.flatnonzero((mapped < self.ranges[:16, 0]) |
                                             (mapped > self.ranges[:16, 1])).tolist()
        return np.clip(mapped, *self.ranges[:16].T)

    def thumb(self, world, handedness, calibration):
        directions = thumb_directions(world, handedness)
        observed = thumb_features(directions)
        profile = calibration.position_profile
        calibrated = all(k in profile for k in ('thumb_open_directions', 'thumb_fist_directions'))
        warning = ''
        progress = None
        residual = None
        if calibrated:
            opened = thumb_features(profile['thumb_open_directions'])
            closed = thumb_features(profile['thumb_fist_directions'])
            stroke = feature_difference(closed, opened)
            span = stroke[2:].sum()
            if span < 15:
                calibrated = False
                warning = '엄지 보정 범위 부족 · 엄지를 펴고 C / 감싸고 V'
            else:
                # Thumb bend supplies a posture correction for different CAD
                # geometry. Root yaw/pitch and MCP/IP residuals remain 1:1.
                # A 7-degree root stroke is never amplified to 56 degrees.
                progress = float(np.clip((observed-opened)[2:].sum()/span, 0, 1))
                expected = opened + progress * stroke
                residual = feature_difference(observed, expected)
                angles = progress * self.fist[16:] + residual
        if not calibrated:
            angles = observed.copy()
            angles[0] += self.design['thumb'].get('base_yaw_deg', 0)
            warning = warning or '엄지까지 맞추려면 C 편 손 → V 주먹 보정'
        weak_sweep = np.linalg.norm(directions[0, :2]) < .2
        if weak_sweep and self.previous is not None:
            angles[0] = self.previous[0]
        limited = np.flatnonzero((angles < self.ranges[16:, 0]) | (angles > self.ranges[16:, 1])).tolist()
        angles = np.clip(angles, *self.ranges[16:].T)
        self.previous = angles.copy()
        return angles, dict(calibrated=calibrated, thumb_progress=progress,
            observed_features_deg=observed, independent_residual_deg=residual,
            estimated_root_posture=calibrated, weak_base_sweep=weak_sweep,
            active_limits=[16+i for i in limited], warning=warning,
            scope='Calibrated thumb posture with independent feature residuals; no collision certification')
