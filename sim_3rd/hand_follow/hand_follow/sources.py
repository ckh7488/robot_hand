"""Frame/landmark sources: webcam (latest frame only), video file, synthetic hand (no camera)."""
import threading
import time
import numpy as np


class LatestFrameCamera:
    """Grabs frames on a background thread and keeps only the newest one, so processing never
    works on a stale, buffered frame. t_capture is the host time the frame was handed over by the
    driver (exposure and USB transfer before that are NOT included)."""

    def __init__(self, index=0, width=640, height=480, fps=30, mjpg=True, backend=None):
        import cv2
        self.cv2 = cv2
        api = backend if backend is not None else cv2.CAP_ANY
        self.cap = cv2.VideoCapture(index, api)
        if not self.cap.isOpened():
            raise RuntimeError(f"Camera {index} could not be opened")
        if mjpg:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.reported_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self._lock = threading.Lock()
        self._condition = threading.Condition(self._lock)
        self._frame = None
        self._t = 0.0
        self._seq = 0
        self._run = True
        self._intervals = []
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        last = None
        while self._run:
            ok, frame = self.cap.read()
            t = time.perf_counter()
            if not ok:
                time.sleep(0.001)
                continue
            if last is not None:
                self._intervals = (self._intervals + [t - last])[-120:]
            last = t
            with self._lock:
                self._frame, self._t = frame, t
                self._seq += 1
                self._condition.notify_all()

    def measured_fps(self):
        iv = self._intervals
        return 1.0 / np.mean(iv) if iv else 0.0

    def read(self, last_seq=-1, timeout=1.0):
        """Return (seq, t_capture, frame) of a frame newer than last_seq."""
        with self._condition:
            ready = self._condition.wait_for(lambda: self._frame is not None and self._seq != last_seq, timeout)
            if ready:
                return self._seq, self._t, self._frame
        return last_seq, None, None

    def close(self):
        self._run = False
        self._thread.join(timeout=1)
        self.cap.release()


class VideoFileSource:
    """Plays a video file at its own frame rate (for repeatable tests)."""

    def __init__(self, path):
        import cv2
        self.cap = cv2.VideoCapture(str(path))
        if not self.cap.isOpened():
            raise RuntimeError(f"Video {path} could not be opened")
        self.fps = self.cap.get(cv2.CAP_PROP_FPS) or 30.0
        self.t0 = None
        self.seq = 0

    def measured_fps(self):
        return self.fps

    def read(self, last_seq=-1, timeout=1.0):
        if self.t0 is None:
            self.t0 = time.perf_counter()
        due = self.t0 + self.seq / self.fps
        while time.perf_counter() < due:
            time.sleep(0.0005)
        ok, frame = self.cap.read()
        if not ok:
            return last_seq, None, None
        self.seq += 1
        return self.seq, time.perf_counter(), frame

    def close(self):
        self.cap.release()


class SyntheticSource:
    """No camera: a scripted human-like hand motion produced at a fixed rate.
    Returns landmarks directly (skips MediaPipe), so it tests everything after hand tracking."""

    def __init__(self, fps=120, noise_m=0.0015, handedness="Right", seed=0):
        from .synthetic import landmarks_from_angles, demo_angles
        self._lm, self._ang = landmarks_from_angles, demo_angles
        self.fps, self.noise, self.hand = fps, noise_m, handedness
        self.rng = np.random.default_rng(seed)
        self.t0 = None
        self.seq = 0

    def measured_fps(self):
        return self.fps

    def read(self, last_seq=-1, timeout=1.0):
        if self.t0 is None:
            self.t0 = time.perf_counter()
        due = self.t0 + self.seq / self.fps
        while time.perf_counter() < due:
            time.sleep(0.0002)
        t = time.perf_counter()
        truth = self._ang(t - self.t0)
        lm = self._lm(truth, self.hand, noise_m=self.noise, rng=self.rng)
        self.seq += 1
        return self.seq, t, dict(world=lm, handedness=self.hand, score=1.0, truth=truth)

    def close(self):
        pass
