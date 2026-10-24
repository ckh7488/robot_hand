"""Local WiLoR-mini adapter; preserves upstream pose inference and coordinates.

The upstream directory and weights are supplied explicitly, never downloaded here.
Input is OpenCV BGR: upstream WiLor.forward reverses channels before normalization.
"""
from pathlib import Path
import gc
import time

import numpy as np
import torch
from ultralytics import YOLO
from wilor_mini.models.wilor import WiLor
from wilor_mini.pipelines.wilor_hand_pose3d_estimation_pipeline import WiLorHandPose3dEstimationPipeline


class LocalWiLoR(WiLorHandPose3dEstimationPipeline):
    def init_models(self, **kwargs):
        self.device = torch.device(kwargs.get('device', 'cuda:0'))
        self.dtype = torch.float32
        self.FOCAL_LENGTH = 5000
        self.IMAGE_SIZE = 256
        root = Path(kwargs['weights'])/'pretrained_models'
        self.wilor_model = WiLor(mano_model_path=str(root/'MANO_RIGHT.pkl'),
                                 mano_mean_path=str(root/'mano_mean_params.npz'),
                                 focal_length=self.FOCAL_LENGTH, image_size=self.IMAGE_SIZE)
        # Avoid holding a second entire checkpoint on the 8GB graphics card.
        checkpoint = torch.load(root/'wilor_final.ckpt', map_location='cpu', weights_only=False, mmap=True)
        loaded = self.wilor_model.load_state_dict(checkpoint['state_dict'], strict=False, assign=True)
        self.checkpoint_keys = {'missing':list(loaded.missing_keys), 'unexpected':list(loaded.unexpected_keys)}
        if loaded.missing_keys:
            raise RuntimeError(f'Uninitialized WiLoR model keys: {loaded.missing_keys}')
        del checkpoint
        gc.collect()
        self.wilor_model.eval().to(device=self.device, dtype=self.dtype)
        self.hand_detector = YOLO(str(root/'detector.pt')).to(self.device)

    def synchronize(self):
        if self.device.type == 'cuda':
            torch.cuda.synchronize(self.device)

    @torch.inference_mode()
    def infer_one(self, bgr):
        """Detect independently of MediaPipe; select the largest detected hand.

        All detections are retained for review. No MediaPipe box, handedness,
        joint, calibration, or robot state is used by the new model.
        """
        if bgr.dtype != np.uint8 or bgr.ndim != 3 or bgr.shape[2] != 3:
            raise ValueError('Expected an unmodified HxWx3 BGR uint8 image')
        self.synchronize()
        start = time.perf_counter()
        detected = self.hand_detector(bgr, conf=0.3, verbose=False)[0].boxes
        rows = detected.data.detach().cpu().float().numpy()
        detections = [dict(bbox=r[:4].tolist(), confidence=float(r[4]), is_right=int(r[5])) for r in rows]
        self.synchronize()
        detection_done = time.perf_counter()
        if not detections:
            return {'status':'no_hand', 'detections':[], 'elapsed_ms':(detection_done-start)*1000}
        selected = int(np.argmax((rows[:,2]-rows[:,0])*(rows[:,3]-rows[:,1])))
        result = self.predict_with_bboxes(bgr, rows[selected:selected+1,:4], [int(rows[selected,5])])[0]
        self.synchronize()
        end = time.perf_counter()
        points = result['wilor_preds']['pred_keypoints_3d'][0]
        if points.shape != (21,3) or not np.isfinite(points).all():
            raise RuntimeError('Invalid WiLoR joint output')
        return dict(status='tracked', detections=detections, selected=selected,
                    detection_ms=(detection_done-start)*1000, pose_ms=(end-detection_done)*1000,
                    elapsed_ms=(end-start)*1000, result=result)
