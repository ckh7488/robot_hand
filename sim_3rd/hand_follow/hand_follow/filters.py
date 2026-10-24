"""One Euro filter (Casiez et al. 2012): heavy smoothing when still, little lag when moving fast."""
import math
import numpy as np


class OneEuro:
    def __init__(self, n, min_cutoff_hz=1.5, beta=0.02, d_cutoff_hz=1.0):
        self.min_cutoff = min_cutoff_hz
        self.beta = beta              # per (deg/s): cutoff rises with speed
        self.d_cutoff = d_cutoff_hz
        self.x = None
        self.dx = np.zeros(n)
        self.t = None
        self.raw_previous = None

    @staticmethod
    def _alpha(cutoff, dt):
        tau = 1.0 / (2 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def reset(self):
        self.x = None
        self.t = None
        self.raw_previous = None

    def __call__(self, x, t):
        x = np.asarray(x, dtype=float)
        if self.x is None or self.t is None or t <= self.t:
            self.x, self.t = x.copy(), t
            self.dx[:] = 0
            self.raw_previous = x.copy()
            return self.x.copy()
        dt = t - self.t
        dx = (x - self.raw_previous) / dt
        self.raw_previous = x.copy()
        a_d = self._alpha(self.d_cutoff, dt)
        self.dx = a_d * dx + (1 - a_d) * self.dx
        cutoff = self.min_cutoff + self.beta * np.abs(self.dx)
        a = np.array([self._alpha(c, dt) for c in cutoff])
        self.x = a * x + (1 - a) * self.x
        self.t = t
        return self.x.copy()
