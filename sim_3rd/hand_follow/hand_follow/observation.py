"""Geometric consistency checks, not a learned landmark confidence score.

Work in a palm-relative frame so rigid hand motion does not trigger a reject.
Only inconsistent fingers are held; the other four remain independently live.
No image/occlusion ground truth is available from this check.
"""
from collections import deque
import numpy as np
from .retarget import FINGERS, palm_frame

GROUPS = list(FINGERS.values()) + [[1, 2, 3, 4]]


class ObservationGuard:
    def __init__(self):
        self.reset()

    def reset(self):
        self.history = [deque(maxlen=31) for _ in GROUPS]
        self.accepted = [None for _ in GROUPS]
        self.candidates = [deque(maxlen=3) for _ in GROUPS]
        self.last_lengths = [None for _ in GROUPS]
        self.last_time = None
        self.hand = None

    def update(self, world, handedness, timestamp):
        p = np.asarray(world, float)
        if p.shape != (21, 3) or not np.isfinite(p).all():
            raise ValueError('손 좌표가 유효하지 않습니다 · 마지막 자세 유지')
        width = np.linalg.norm(p[5]-p[17])
        height = np.linalg.norm(p[9]-p[0])
        area = np.linalg.norm(np.cross(p[5]-p[17], p[9]-p[0]))
        if min(width, height) < .02 or area/max(width*height, 1e-9) < .35:
            raise ValueError('손바닥 방향이 불확실합니다 · 마지막 자세 유지')
        frame = palm_frame(p, handedness != 'Left')
        local = (p-p[0]) @ frame / width
        if self.hand != handedness or (self.last_time is not None and timestamp-self.last_time > 1.):
            self.reset()
        self.hand, self.last_time = handedness, timestamp
        held, deviations, rebased = [], [], []
        for group, indices in enumerate(GROUPS):
            points = local[indices]
            bones = np.diff(points, axis=0)
            lengths = np.linalg.norm(bones, axis=1)
            baseline = np.median(self.history[group], axis=0) if self.history[group] else lengths
            relative = np.abs(lengths / np.maximum(baseline, .015)-1)
            if self.last_lengths[group] is not None:
                relative = np.maximum(relative, np.abs(lengths/self.last_lengths[group]-1))
            deviations.append(float(relative.max()))
            # Physical bones cannot grow/shrink with a finger curl. Tolerances
            # admit ordinary RGB depth noise; do not call accepted data correct.
            impossible = np.any(lengths < .04) or np.any(lengths > 2.)
            invalid = impossible or np.any(relative > .35)
            if invalid and not impossible:
                candidates = self.candidates[group]
                candidates.append(lengths.copy())
                consensus = np.median(candidates, axis=0)
                # Initial RGB scale/shape can be wrong. A consistent new
                # estimate may replace it after three samples; freezing the
                # first estimate forever would masquerade as low jitter.
                if len(candidates) == 3 and np.max(np.abs(np.array(candidates)/consensus-1)) < .12:
                    self.history[group].clear()
                    self.history[group].append(consensus)
                    candidates.clear()
                    invalid = False
                    rebased.append(group)
            if invalid:
                held.append(group)
                if self.accepted[group] is not None:
                    local[indices[1:]] = points[0] + self.accepted[group]
                continue
            self.candidates[group].clear()
            self.last_lengths[group] = lengths.copy()
            self.history[group].append(lengths.copy())
            baseline = np.median(self.history[group], axis=0)
            offsets = np.cumsum(bones / lengths[:, None] * baseline[:, None], axis=0)
            self.accepted[group] = offsets.copy()
            local[indices[1:]] = points[0]+offsets
        # Return to the original camera frame for existing angle/position code.
        # Reflection for a left hand is orthonormal and has the same inverse.
        corrected = local*width @ frame.T + p[0]
        info = dict(held_fingers=held, rebased_fingers=rebased, bone_length_relative_error=deviations,
                    scope='Geometric consistency only; no per-joint visibility confidence')
        return corrected, info
