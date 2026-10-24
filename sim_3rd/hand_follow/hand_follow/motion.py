"""Continuous preview motion across noisy targets and tracking dropouts.

These bounds are visual motion controls, not motor performance or collision
certification. Each joint remains independent; no authored fist is replayed.
"""
from collections import deque
import numpy as np


class MotionStabilizer:
    def __init__(self, ranges):
        self.ranges = np.asarray(ranges, float)
        self.position = np.clip(np.zeros(len(ranges)), *self.ranges.T)
        self.velocity = np.zeros(len(ranges))
        self.speed = np.array([180, 300, 300, 300] * 4 + [180, 180, 240, 240], float)
        self.acceleration = np.array([1400, 2200, 2200, 2200] * 4 + [1200, 1200, 1600, 1600], float)
        self.history = deque(maxlen=3)
        self.time = None
        self.limited_joints = []

    def hold(self, t):
        self.time = float(t)
        self.velocity[:] = 0
        self.history.clear()

    def __call__(self, target, t, smooth=True):
        target = np.clip(np.asarray(target, float), *self.ranges.T)
        if target.shape != self.position.shape or not np.isfinite(target).all():
            raise ValueError('Invalid motion target')
        if self.time is None:
            self.time = float(t)
        elapsed = float(t) - self.time
        self.time = float(t)
        if elapsed <= 0:
            return self.position.copy()
        if elapsed > .25:
            self.velocity[:] = 0
            self.history.clear()
        # Never consume a tracking gap as a giant integration timestep.
        dt = min(elapsed, .1)
        self.history.append(target.copy())
        if smooth and len(self.history) == 3:
            target = np.median(self.history, axis=0)
        if smooth:
            quiet = (np.abs(target-self.position) < .7) & (np.abs(self.velocity) < 3)
            target[quiet] = self.position[quiet]
        steps = max(1, int(np.ceil(dt / (1/240))))
        h = dt / steps
        limited = np.zeros(len(target), bool)
        for _ in range(steps):
            error = target-self.position
            # Small deviations around a held pose need more damping than a
            # deliberate large move. Do not use a noisy derivative as speed.
            omega = 12.+16.*(1-np.exp(-np.abs(error)/12.)) if smooth else 45.
            acceleration = omega**2*error-2*omega*self.velocity
            # Clipping a critically damped acceleration can otherwise remove
            # its braking phase after a large target jump.
            braking = (self.velocity*error > 0) & (
                self.velocity**2/(2*self.acceleration) > .65*np.abs(error))
            acceleration[braking] = -np.sign(self.velocity[braking])*self.acceleration[braking]
            limited |= np.abs(acceleration) > self.acceleration
            acceleration = np.clip(acceleration, -self.acceleration, self.acceleration)
            velocity = self.velocity + acceleration*h
            limited |= np.abs(velocity) > self.speed
            self.velocity = np.clip(velocity, -self.speed, self.speed)
            candidate = self.position + self.velocity*h
            self.position = np.clip(candidate, *self.ranges.T)
            self.velocity[candidate != self.position] = 0
        self.limited_joints = np.flatnonzero(limited).tolist()
        return self.position.copy()
