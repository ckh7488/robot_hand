"""Synthetic MediaPipe-style world landmarks from known joint angles.

Used by the tests (round trip: angles -> landmarks -> retarget -> angles) and by the
no-camera demo source. Geometry is roughly a human right hand in metres.
"""
import numpy as np
from .retarget import NAMES, FINGERS, THUMB, THUMB_BASE_YAW_DEG, THUMB_SECOND_YAW_DEG

MCP_POS = {"index": (0.022, 0.085, 0.0), "middle": (0.0, 0.090, 0.0),
           "ring": (-0.020, 0.085, 0.0), "little": (-0.038, 0.075, 0.0)}
LENGTHS = {"index": (0.040, 0.025, 0.020), "middle": (0.045, 0.028, 0.022),
           "ring": (0.042, 0.027, 0.021), "little": (0.033, 0.020, 0.018)}
THUMB_CMC = (0.025, 0.020, 0.0)
THUMB_LEN = (0.045, 0.032, 0.025)
Z = np.array([0.0, 0.0, 1.0])


def _rot(axis, deg):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    t = np.radians(deg)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(t) * K + (1 - np.cos(t)) * K @ K


def landmarks_from_angles(angles_deg, handedness="Right", rotation=None, noise_m=0.0, rng=None):
    q = dict(zip(NAMES, np.asarray(angles_deg, float)))
    p = np.zeros((21, 3))
    for f, idx in FINGERS.items():
        mcp = np.array(MCP_POS[f])
        ref = mcp / np.linalg.norm(mcp)
        ref = np.array([ref[0], ref[1], 0.0]) / np.linalg.norm(ref[:2])
        d0 = _rot(-Z, q[f + "_spread"]) @ ref
        k = np.cross(d0, Z)
        L1, L2, L3 = LENGTHS[f]
        a1 = q[f + "_mcp"]; a2 = a1 + q[f + "_pip"]; a3 = a2 + q[f + "_dip"]
        p[idx[0]] = mcp
        p[idx[1]] = p[idx[0]] + L1 * (_rot(k, a1) @ d0)
        p[idx[2]] = p[idx[1]] + L2 * (_rot(k, a2) @ d0)
        p[idx[3]] = p[idx[2]] + L3 * (_rot(k, a3) @ d0)
    yaw = np.radians(THUMB_BASE_YAW_DEG)
    yb = np.array([-np.sin(yaw), np.cos(yaw), 0.0])
    d1 = _rot(-Z, q["thumb_base_sweep"]) @ yb
    k1 = np.cross(d1, Z)
    o = q["thumb_opposition"]
    c = np.array(THUMB_CMC)
    p[THUMB[0]] = c
    p[THUMB[1]] = c + THUMB_LEN[0] * (_rot(k1, o) @ d1)
    base = _rot(Z, THUMB_BASE_YAW_DEG - q['thumb_base_sweep']) @ _rot([1, 0, 0], o)
    second = base @ _rot(Z, THUMB_SECOND_YAW_DEG)
    p[THUMB[2]] = p[THUMB[1]] + THUMB_LEN[1] * (second @ _rot([1, 0, 0], q['thumb_mcp']) @ [0, 1, 0])
    p[THUMB[3]] = p[THUMB[2]] + THUMB_LEN[2] * (second @ _rot([1, 0, 0], q['thumb_mcp'] + q['thumb_ip']) @ [0, 1, 0])
    if handedness == "Left":
        p[:, 0] *= -1.0            # mirror image of a right hand is a left hand
    if rotation is not None:
        p = p @ np.asarray(rotation).T
    p = p - p[0]
    if noise_m:
        rng = rng or np.random.default_rng(0)
        p = p + rng.normal(0, noise_m, p.shape)
    return p


def demo_angles(t):
    """A repeating sequence: open -> fist -> open -> point -> spread -> thumb opposition."""
    open_ = np.zeros(20)
    fist = np.array([0, 80, 95, 70] * 4 + [-55, 50, 0, 0], float)
    point = fist.copy(); point[0:4] = 0
    splay = np.array([15, 0, 0, 0, 2, 0, 0, 0, -6, 0, 0, 0, -15, 0, 0, 0, -20, 10, 0, 0], float)
    oppose = np.zeros(20); oppose[16:20] = [20, 70, 30, 40]
    seq = [open_, fist, open_, point, open_, splay, open_, oppose]
    seg = 0.8                                   # seconds per transition
    k = int(t // seg) % len(seq)
    s = (t % seg) / seg
    s = min(1.0, s / 0.4)                       # move in 0.32 s, then hold
    s = 10 * s**3 - 15 * s**4 + 6 * s**5
    return (1 - s) * seq[k] + s * seq[(k + 1) % len(seq)]
