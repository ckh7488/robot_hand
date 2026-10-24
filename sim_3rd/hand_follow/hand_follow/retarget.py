"""MediaPipe hand world landmarks -> 20 robot joint angles (degrees).

Joint order is the repository's 20-axis command convention. Thumb geometry
defaults match cad/form_candidate_v1 and is read from that CAD by the application:
  index/middle/ring/little: spread, mcp, pip, dip
  thumb: base_sweep, opposition, mcp, ip
Palm frame (same as the robot): x = transverse toward the thumb, y = distal (wrist -> middle MCP),
z = palmar. Positive spread = toward the thumb. Positive flexion = toward the palm.

Angles are anatomical, so the same numbers drive the robot whether the person shows a left or a
right hand (mirror imitation). Only the palm normal depends on handedness.
"""
from dataclasses import dataclass, field
import json
from pathlib import Path
import numpy as np

NAMES = [f"{f}_{j}" for f in ["index", "middle", "ring", "little"]
         for j in ["spread", "mcp", "pip", "dip"]] + [
             "thumb_base_sweep", "thumb_opposition", "thumb_mcp", "thumb_ip"]

# MediaPipe hand landmark indices
WRIST = 0
THUMB = [1, 2, 3, 4]          # CMC, MCP, IP, TIP
FINGERS = {"index": [5, 6, 7, 8], "middle": [9, 10, 11, 12],
           "ring": [13, 14, 15, 16], "little": [17, 18, 19, 20]}

# Defaults match cad/form_candidate_v1; the application passes the CAD values.
THUMB_BASE_YAW_DEG = -40.0
THUMB_SECOND_YAW_DEG = 25.0

# Repo design.json joint_ranges_deg (used when the repo is not available)
DEFAULT_RANGES = {"spread": [-22, 22], "mcp": [-20, 85], "pip": [0, 100], "dip": [0, 80],
                  "thumb_base_sweep": [-60, 60], "thumb_opposition": [-20, 100],
                  "thumb_mcp": [-20, 105], "thumb_ip": [0, 90]}


def _unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else v * 0.0


def palm_frame(p, right_handed=True):
    """Return R (3x3, columns x,y,z in camera/world coords) of the palm frame."""
    y = _unit(p[9] - p[WRIST])
    xr = p[5] - p[17]
    x = _unit(xr - (xr @ y) * y)
    z = np.cross(x, y)
    if not right_handed:
        z = -z
    return np.column_stack([x, y, z])


def _signed_angle(a, b, axis):
    """Angle from bone a to bone b about the flexion axis (+ = toward the palm). Signed, so
    landmark noise around a straight joint averages to zero instead of biasing it positive."""
    a, b = _unit(a), _unit(b)
    cross = np.cross(a, b)
    projected = np.arctan2(axis @ cross, a @ b)
    magnitude = np.arctan2(np.linalg.norm(cross), np.clip(a @ b, -1, 1))
    signed = magnitude * (1 if axis @ cross >= 0 else -1)
    # Near straight, transverse landmark noise is not real flexion. For a
    # clearly bent joint use its full bend, even when the estimated axis twists.
    weight = np.clip((abs(np.degrees(projected)) - 25) / 25, 0, 1)
    return float(np.degrees((1 - weight) * projected + weight * signed))


def raw_angles(world_landmarks, handedness="Right", thumb_base_yaw=THUMB_BASE_YAW_DEG,
               thumb_second_yaw=THUMB_SECOND_YAW_DEG):
    """21x3 array in metres -> 20 uncalibrated angles in NAMES order, degrees."""
    p = np.asarray(world_landmarks, dtype=float)
    if p.shape != (21, 3) or not np.isfinite(p).all():
        raise ValueError('Expected 21 finite three-dimensional landmarks')
    if np.linalg.norm(p[9] - p[0]) < 0.01 or np.linalg.norm(np.cross(p[5] - p[17], p[9] - p[0])) < 0.0001:
        raise ValueError('Degenerate palm landmarks')
    R = palm_frame(p, right_handed=(handedness != "Left"))
    q = (p - p[WRIST]) @ R          # landmarks in palm frame
    out = {}
    for f, (a, b, c, d) in FINGERS.items():
        meta = q[a] - q[WRIST]
        ref = _unit(np.array([meta[0], meta[1], 0.0]))     # metacarpal direction in palm plane
        prox = _unit(q[b] - q[a])
        inplane = np.array([prox[0], prox[1], 0.0])
        # spread: signed in-plane angle from the metacarpal direction, + toward thumb (+x)
        side = np.array([ref[1], -ref[0], 0.0])            # ref rotated toward +x
        spread = float(np.degrees(np.arctan2(inplane @ side, inplane @ ref)))
        # At/over 90 degrees flexion the projected bone reverses. Choosing the
        # forward spread branch prevents both distal flexion signs flipping.
        spread = (spread + 90) % 180 - 90
        spread = float(np.clip(spread, -60, 60))
        s = np.radians(spread)
        forward = np.cos(s) * ref + np.sin(s) * side
        out[f + "_spread"] = spread
        out[f + "_mcp"] = float(np.degrees(np.arctan2(prox[2], prox @ forward)))
        axis = _unit(np.cross(forward, [0.0, 0.0, 1.0]))
        out[f + "_pip"] = _signed_angle(q[b] - q[a], q[c] - q[b], axis)
        out[f + "_dip"] = _signed_angle(q[c] - q[b], q[d] - q[c], axis)
    c1, c2, c3, c4 = THUMB
    meta = _unit(q[c2] - q[c1])
    yaw = np.radians(thumb_base_yaw)
    xb = np.array([np.cos(yaw), np.sin(yaw), 0.0])      # base frame x' in palm coords
    yb = np.array([-np.sin(yaw), np.cos(yaw), 0.0])     # base frame y' (thumb zero direction)
    out["thumb_base_sweep"] = float(np.degrees(np.arctan2(meta @ xb, meta @ yb)))
    out["thumb_opposition"] = float(np.degrees(np.arctan2(meta[2], np.hypot(meta @ xb, meta @ yb))))
    # The CAD has an oblique second joint (25 degrees), not four coplanar bones.
    a = yaw - np.radians(out['thumb_base_sweep'])
    o = np.radians(out['thumb_opposition'])
    b = np.radians(thumb_second_yaw)
    rz = lambda t: np.array([[np.cos(t), -np.sin(t), 0], [np.sin(t), np.cos(t), 0], [0, 0, 1]])
    rx = np.array([[1, 0, 0], [0, np.cos(o), -np.sin(o)], [0, np.sin(o), np.cos(o)]])
    frame = rz(a) @ rx @ rz(b)
    distal = (q[c3] - q[c2]) @ frame
    out['thumb_mcp'] = float(np.degrees(np.arctan2(distal[2], distal[1])))
    out['thumb_ip'] = _signed_angle(q[c3] - q[c2], q[c4] - q[c3], frame[:, 0])
    return np.array([out[n] for n in NAMES])


def joint_ranges(model=None):
    """(20,2) degrees. From a MuJoCo model built by the repo if given, else repo defaults."""
    if model is not None:
        return np.degrees(model.jnt_range[:20]).copy()
    rows = []
    for n in NAMES:
        key = n if n.startswith("thumb") else n.split("_", 1)[1]
        rows.append(DEFAULT_RANGES[key])
    return np.array(rows, dtype=float)


@dataclass
class Calibration:
    """angle_robot = gain * (raw - offset). Offsets are captured with a flat, relaxed open hand."""
    offset: np.ndarray = field(default_factory=lambda: np.zeros(20))
    gain: np.ndarray = field(default_factory=lambda: np.ones(20))
    coupled_dips: list = field(default_factory=list)
    dip_ratios: dict = field(default_factory=dict)
    position_profile: dict = field(default_factory=dict)

    def apply(self, raw):
        mapped = self.gain * (raw - self.offset)
        for joint in self.coupled_dips:
            mapped[joint] = mapped[joint - 1] * self.dip_ratios.get(joint, 70 / 95)
        return mapped

    def capture_open_hand(self, raw, keep_thumb_base=True):
        """Flat open hand defines zero for spread/flexion. Thumb base angles keep the robot's
        own zero unless keep_thumb_base is False (then the person's relaxed thumb becomes zero)."""
        new = raw.copy()
        if keep_thumb_base:
            new[16] = 0.0
            new[17] = 0.0
        self.offset = new

    def save(self, path):
        Path(path).write_text(json.dumps({"names": NAMES, "offset_deg": self.offset.tolist(),
                                          "gain": self.gain.tolist(), "estimated_dip_indices": self.coupled_dips,
                                          "estimated_dip_ratios": self.dip_ratios,
                                          "position_profile": self.position_profile}, indent=1), encoding="utf-8")

    @classmethod
    def load(cls, path):
        d = json.loads(Path(path).read_text(encoding="utf-8"))
        offset, gain = np.array(d['offset_deg'], float), np.array(d['gain'], float)
        if d.get('names') != NAMES or offset.shape != (20,) or gain.shape != (20,) or not np.isfinite([offset, gain]).all():
            raise ValueError('Calibration has invalid joint order or values')
        if np.any(gain <= 0) or np.any(gain > 4):
            raise ValueError('Calibration gain outside safe mapping range')
        estimated = d.get('estimated_dip_indices', [])
        if not isinstance(estimated, list) or any(type(i) is not int or i not in (3, 7, 11, 15) for i in estimated):
            raise ValueError('Invalid estimated DIP indices')
        ratios = {int(i): float(r) for i, r in d.get('estimated_dip_ratios', {}).items()}
        if any(i not in (3, 7, 11, 15) or not np.isfinite(r) or not 0 < r <= 2 for i, r in ratios.items()):
            raise ValueError('Invalid estimated DIP ratio')
        profile = d.get('position_profile', {})
        if profile:
            if not isinstance(profile, dict) or profile.get('schema') != 1 or not isinstance(profile.get('cad_signature'), str):
                raise ValueError('Invalid position calibration profile')
            for key in ('open_tips', 'fist_tips'):
                if key == 'fist_tips' and key not in profile:
                    continue
                values = np.asarray(profile.get(key), float)
                if values.shape != (4, 3) or not np.isfinite(values).all() or np.max(np.abs(values)) > 2:
                    raise ValueError('Invalid normalized fingertip calibration')
            if 'fist_flex_deg' in profile:
                values = np.asarray(profile['fist_flex_deg'], float)
                if values.shape != (4,3) or not np.isfinite(values).all() or np.max(np.abs(values)) > 180:
                    raise ValueError('Invalid fist flexion calibration')
            for key in ('thumb_open_directions','thumb_fist_directions'):
                if key in profile:
                    values=np.asarray(profile[key],float)
                    if values.shape!=(3,3) or not np.isfinite(values).all() or np.max(np.abs(np.linalg.norm(values,axis=1)-1))>.01:
                        raise ValueError('Invalid thumb direction calibration')
        return cls(offset, gain, sorted(set(estimated)), ratios, profile)
