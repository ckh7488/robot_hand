"""MediaPipe Tasks HandLandmarker in VIDEO mode (synchronous, keeps tracking between frames)."""
from pathlib import Path
import urllib.request

MODEL_URL = ("https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
             "hand_landmarker/float16/latest/hand_landmarker.task")


def ensure_model(path):
    path = Path(path)
    if not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        print(f"Downloading hand landmark model to {path} ...")
        urllib.request.urlretrieve(MODEL_URL, path)
    return path


class HandTracker:
    def __init__(self, model_path, min_detect=0.5, min_presence=0.5, min_track=0.5, gpu=False, max_hands=1):
        import mediapipe as mp
        from mediapipe.tasks import python as mpt
        from mediapipe.tasks.python import vision
        self.mp = mp
        base = mpt.BaseOptions(model_asset_path=str(ensure_model(model_path)),
                               delegate=mpt.BaseOptions.Delegate.GPU if gpu else mpt.BaseOptions.Delegate.CPU)
        opts = vision.HandLandmarkerOptions(base_options=base, running_mode=vision.RunningMode.VIDEO,
                                            num_hands=max_hands, min_hand_detection_confidence=min_detect,
                                            min_hand_presence_confidence=min_presence,
                                            min_tracking_confidence=min_track)
        self.det = vision.HandLandmarker.create_from_options(opts)
        self.last_ts = -1
        self.wrist = None
        self.label = None
        self.pending_label = None
        self.pending_count = 0
        self.last_seen_ms = -10000

    def reset_selection(self):
        self.wrist = None
        self.label = None
        self.pending_count = 0

    def process(self, rgb, t_ms):
        """RGB input; handedness and world coordinates refer to this same image."""
        t_ms = int(t_ms)
        if t_ms <= self.last_ts:
            t_ms = self.last_ts + 1
        self.last_ts = t_ms
        img = self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=rgb)
        res = self.det.detect_for_video(img, t_ms)
        if not res.hand_world_landmarks:
            return None
        import numpy as np
        wrists = [np.array([points[0].x, points[0].y]) for points in res.hand_landmarks]
        continuing = self.wrist is not None and t_ms - self.last_seen_ms < 800
        index = int(np.argmin([np.linalg.norm(w - self.wrist) for w in wrists])) if continuing else int(np.argmax([h[0].score for h in res.handedness]))
        cat = res.handedness[index][0]
        if not continuing:
            self.label = cat.category_name
            self.pending_count = 0
        elif cat.category_name != self.label:
            self.pending_count = self.pending_count + 1 if self.pending_label == cat.category_name else 1
            self.pending_label = cat.category_name
            if self.pending_count >= 5:
                self.label = cat.category_name
                self.pending_count = 0
        else:
            self.pending_count = 0
        self.wrist, self.last_seen_ms = wrists[index], t_ms
        world = np.array([[p.x, p.y, p.z] for p in res.hand_world_landmarks[index]])
        image = np.array([[p.x, p.y] for p in res.hand_landmarks[index]])
        return dict(world=world, image=image, handedness=self.label, detected_handedness=cat.category_name, score=cat.score)

    def close(self):
        self.det.close()
