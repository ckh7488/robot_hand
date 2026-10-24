"""Responsive output filtering with explicit loss/reacquisition behavior."""
import numpy as np
from .filters import OneEuro


class ContinuousPoseFilter:
    def __init__(self, ranges, min_cutoff=2.5, beta=.012):
        self.ranges = np.asarray(ranges, float)
        self.filter = OneEuro(20, min_cutoff_hz=min_cutoff, beta=beta)
        self.position = np.zeros(20)
        self.last_time = None
        self.lost = True
        self.pending = np.full(20, np.nan)
        self.previous_target = None
        self.held_joints = []
        self.was_smooth = True

    def hold(self, timestamp):
        self.last_time = timestamp
        self.lost = True
        self.pending[:] = np.nan

    def __call__(self, target, timestamp, smooth=True):
        target = np.clip(np.asarray(target, float), *self.ranges.T)
        dt = 1/30 if self.last_time is None or self.lost else np.clip(timestamp-self.last_time, .001, .1)
        reacquired = self.lost
        self.last_time, self.lost = timestamp, False
        self.held_joints = []
        if smooth and self.previous_target is not None and not reacquired:
            # Confirm isolated large innovations once. A sustained real change
            # passes on the next sample; it must never be held indefinitely.
            jump = np.abs(target-self.previous_target) > 40
            confirmed = np.isfinite(self.pending) & (np.abs(target-self.pending) < 18)
            reject = jump & ~confirmed
            candidate = target.copy()
            target[reject] = self.previous_target[reject]
            self.pending = np.where(reject, candidate, np.nan)
            self.held_joints = np.flatnonzero(reject).tolist()
        else:
            self.pending[:] = np.nan
        self.previous_target = target.copy()
        if self.filter.x is None or reacquired or smooth != self.was_smooth:
            # Seed from the held command, never the newly observed pose.
            self.filter.x = self.position.copy()
            self.filter.raw_previous = self.position.copy()
            self.filter.dx[:] = 0
        self.was_smooth = smooth
        self.filter.t = timestamp-dt
        filtered = self.filter(target, timestamp) if smooth else target.copy()
        # Even diagnostic F-off reacquisition must not teleport from the held
        # pose; ordinary live frames are not restricted by this ramp.
        if reacquired:
            filtered = np.clip(filtered, self.position-720*dt, self.position+720*dt)
        self.position = np.clip(filtered, *self.ranges.T)
        return self.position.copy()
