"""Cheap live drawing of detector world points, before any retargeting/filter.

Only a rigid camera rotation/translation and one uniform display scale are used.
No handedness correction, joint limits, bone normalization or pose history.
"""
import cv2
import numpy as np

FINGERS = [([1, 2, 3, 4], (53, 199, 193)),
           ([5, 6, 7, 8], (255, 158, 82)),
           ([9, 10, 11, 12], (120, 210, 146)),
           ([13, 14, 15, 16], (235, 142, 187)),
           ([17, 18, 19, 20], (179, 155, 228))]
PALM = [(0, 1), (0, 5), (5, 9), (9, 13), (13, 17), (17, 0)]
BACKGROUND = (20, 26, 36)


def display_points(world, view=0):
    """Rigidly align wrist -> middle MCP upright, then choose a viewing angle.

    A proper rotation for both hands: their chirality is never reflected.
    Malformed or degenerate palms have no invented substitute pose.
    """
    p = np.asarray(world, dtype=float)
    if p.shape != (21, 3) or not np.isfinite(p).all():
        raise ValueError('Expected 21 finite world points')
    y = p[9] - p[0]
    if np.linalg.norm(y) < 1e-6:
        raise ValueError('Degenerate wrist direction')
    y /= np.linalg.norm(y)
    x = p[5] - p[17]
    x -= (x @ y) * y
    if np.linalg.norm(x) < 1e-6:
        raise ValueError('Degenerate palm width')
    x /= np.linalg.norm(x)
    q = (p-p[0]) @ np.column_stack((x, y, np.cross(x, y)))
    # Match the CAD views' viewing directions, with +y vertically upward.
    azimuth, elevation = [(90, -90), (135, -32), (180, -5)][view % 3]
    a, e = np.radians([azimuth, elevation])
    toward_eye = -np.array([np.cos(e)*np.cos(a), np.cos(e)*np.sin(a), np.sin(e)])
    up = np.array([0., 1., 0.])
    up -= (up @ toward_eye)*toward_eye
    up /= np.linalg.norm(up)
    right = np.cross(up, toward_eye)
    # MuJoCo angles describe the direction toward the target; the eye is
    # opposite it. Front is therefore +z with +x toward the screen's right.
    return q @ np.column_stack((right, up, toward_eye))


def render_raw(detection, view=0, width=532, height=620):
    image = np.empty((height, width, 3), dtype=np.uint8)
    image[:] = BACKGROUND
    if detection is None:
        return image, 'missing'
    try:
        xyz = display_points(detection['world'], view)
    except (KeyError, TypeError, ValueError):
        return image, 'invalid'
    # Fixed metres-to-pixels scale, same on all axes; never normalize each bone.
    scale = min(width/.25, height/.29)
    # Center the entire observed hand without changing its internal geometry.
    centered = xyz[:, :2] - (xyz[:, :2].max(0)+xyz[:, :2].min(0))/2
    pts = centered * [scale, -scale] + [width/2, height/2]
    pts = np.rint(np.clip(pts, -1_000_000, 1_000_000)).astype(int)
    segments = [(a, b, (112, 133, 153), 2) for a, b in PALM]
    colors = {0: (164, 183, 201)}
    for ids, color in FINGERS:
        segments.extend((a, b, color, 5) for a, b in zip(ids, ids[1:]))
        colors.update({i: color for i in ids})
    # Farther segments first; crossing lines retain a useful depth cue.
    for a, b, color, thickness in sorted(segments, key=lambda s: (xyz[s[0], 2]+xyz[s[1], 2])/2):
        cv2.line(image, tuple(pts[a]), tuple(pts[b]), BACKGROUND, thickness+4, cv2.LINE_AA)
        cv2.line(image, tuple(pts[a]), tuple(pts[b]), color, thickness, cv2.LINE_AA)
    for i in np.argsort(xyz[:, 2]):
        cv2.circle(image, tuple(pts[i]), 6, colors[i], -1, cv2.LINE_AA)
    return image, 'tracked'
