"""Original inference outputs, with explicit detector and coordinate contracts."""
import contextlib
import io
import os
import pathlib
import sys
import time

os.environ.setdefault('PYOPENGL_PLATFORM','win32' if os.name=='nt' else 'egl')
os.environ.setdefault('OMP_NUM_THREADS','1')
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
os.environ.setdefault('YOLO_CONFIG_DIR',r'E:\hand_recog\vision-runtime\yolo')
os.environ.setdefault('MPLCONFIGDIR',r'E:\hand_recog\vision-runtime\mpl')

import numpy as np

RUNTIME=pathlib.Path(r'E:\hand_recog\vision-runtime')
MODELS=RUNTIME/'models'
sys.path[:0]=[str(RUNTIME/'WiLoR-mini'),str(RUNTIME/'hamer'),str(RUNTIME/'chumpy'),str(pathlib.Path(__file__).resolve().parents[1]/'hand_pose_compare')]


class MediaPipe:
    name='mediapipe'
    faces=None

    def __init__(self,device=0):
        import mediapipe as mp
        self.mp=mp
        options=mp.tasks.vision.HandLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=r'E:\hand_recog\hand-lab\dist\assets\hand_landmarker.task'),
            running_mode=mp.tasks.vision.RunningMode.IMAGE,num_hands=2,
            min_hand_detection_confidence=.3,min_hand_presence_confidence=.3,min_tracking_confidence=.5)
        self.model=mp.tasks.vision.HandLandmarker.create_from_options(options)

    def predict(self,bgr):
        import cv2
        rgb=cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)
        img=self.mp.Image(image_format=self.mp.ImageFormat.SRGB,data=rgb)
        t=time.perf_counter();result=self.model.detect(img);elapsed=(time.perf_counter()-t)*1000
        h,w=bgr.shape[:2];hands=[]
        for lm,world,side in zip(result.hand_landmarks,result.hand_world_landmarks,result.handedness):
            hands.append(dict(xyz=np.array([[p.x,p.y,p.z] for p in world],np.float32),
                              uv=np.array([[p.x*w,p.y*h] for p in lm],np.float32),
                              side=int(side[0].category_name=='Right'),confidence=float(side[0].score),
                              raw_side=side[0].category_name))
        return hands,elapsed


class WiLoR:
    name='wilor'

    def __init__(self,device=0):
        import torch
        from wilor_adapter import LocalWiLoR
        torch.set_num_threads(1)
        self.torch=torch;self.device=f'cuda:{device}'
        self.model=LocalWiLoR(weights=MODELS/'wilor',device=self.device,verbose=False)
        self.faces=np.asarray(self.model.wilor_model.mano.faces,dtype=np.int32)
        self.checkpoint_keys=self.model.checkpoint_keys

    def predict(self,bgr):
        self.torch.cuda.synchronize(self.device);t=time.perf_counter()
        with self.torch.inference_mode():
            raw=self.model.predict(bgr,hand_conf=.3)
        self.torch.cuda.synchronize(self.device);elapsed=(time.perf_counter()-t)*1000
        hands=[]
        for r in raw:
            p=r['wilor_preds']
            hands.append(dict(xyz=p['pred_keypoints_3d'][0],uv=p['pred_keypoints_2d'][0],
                              vertices=p['pred_vertices'][0],camera_t=p['pred_cam_t_full'][0],
                              side=int(r['is_right']),bbox=np.array(r['hand_bbox']),
                              focal=float(p['scaled_focal_length'])))
        return hands,elapsed


class HaMeR:
    name='hamer'

    def __init__(self,device=1):
        import torch
        from ultralytics import YOLO
        from hamer.configs import get_config
        from hamer.models import HAMER
        from hamer.datasets.vitdet_dataset import ViTDetDataset
        from hamer.utils.renderer import cam_crop_to_full
        torch.set_num_threads(1)
        self.torch=torch;self.device=f'cuda:{device}';self.Dataset=ViTDetDataset;self.cam_crop_to_full=cam_crop_to_full
        ckpt=next((MODELS/'hamer').rglob('hamer.ckpt'))
        cfg=get_config(str(ckpt.parent.parent/'model_config.yaml'))
        cfg.defrost()
        cfg.MODEL.BBOX_SHAPE=[192,256]
        cfg.MODEL.BACKBONE.pop('PRETRAINED_WEIGHTS',None)
        cfg.MANO.MODEL_PATH=str(MODELS/'wilor/pretrained_models/MANO_RIGHT.pkl')
        cfg.MANO.MEAN_PARAMS=str(MODELS/'wilor/pretrained_models/mano_mean_params.npz')
        cfg.freeze();self.cfg=cfg
        self.model=HAMER(cfg,init_renderer=False)
        state=torch.load(ckpt,map_location='cpu',weights_only=False,mmap=True)
        missing=self.model.load_state_dict(state['state_dict'],strict=False,assign=True)
        self.checkpoint_keys=dict(missing=list(missing.missing_keys),unexpected=list(missing.unexpected_keys))
        if missing.missing_keys:
            raise ValueError('HaMeR missing weights: '+repr(missing.missing_keys))
        del state
        self.model.eval().to(self.device)
        self.faces=np.asarray(self.model.mano.faces,dtype=np.int32)
        # Explicit common detector for WiLoR and HaMeR; no MediaPipe or GT boxes/handedness.
        self.detector=YOLO(str(MODELS/'wilor/pretrained_models/detector.pt')).to(self.device)

    def predict(self,bgr):
        torch=self.torch
        torch.cuda.synchronize(self.device);t=time.perf_counter()
        with torch.inference_mode():
            boxes=self.detector(bgr,conf=.3,verbose=False)[0].boxes.data.cpu().numpy()
            if not len(boxes):
                torch.cuda.synchronize(self.device)
                return [],(time.perf_counter()-t)*1000
            dataset=self.Dataset(self.cfg,bgr,boxes[:,:4],boxes[:,5],rescale_factor=2.0)
            with contextlib.redirect_stdout(io.StringIO()):
                items=[dataset[i] for i in range(len(dataset))]
            batch={k:torch.as_tensor(np.stack([d[k] for d in items]),device=self.device) for k in items[0]}
            out=self.model(batch)
            cam=out['pred_cam'].clone();cam[:,1]*=(2*batch['right']-1)
            focal=self.cfg.EXTRA.FOCAL_LENGTH/self.cfg.MODEL.IMAGE_SIZE*max(bgr.shape[:2])
            trans=self.cam_crop_to_full(cam,batch['box_center'],batch['box_size'],batch['img_size'],focal).cpu().numpy()
            xyz=out['pred_keypoints_3d'].cpu().numpy();verts=out['pred_vertices'].cpu().numpy()
            sides=boxes[:,5].astype(int);xyz[:,:,0]*=(2*sides[:,None]-1);verts[:,:,0]*=(2*sides[:,None]-1)
            camera_xyz=xyz+trans[:,None,:]
            uv=camera_xyz[:,:,:2]/camera_xyz[:,:,2:]*focal+np.array([bgr.shape[1],bgr.shape[0]])/2
        torch.cuda.synchronize(self.device);elapsed=(time.perf_counter()-t)*1000
        return [dict(xyz=xyz[i],uv=uv[i],vertices=verts[i],camera_t=trans[i],side=int(sides[i]),
                     confidence=float(boxes[i,4]),bbox=boxes[i,:4],focal=float(focal)) for i in range(len(boxes))],elapsed


FACTORIES={'mediapipe':MediaPipe,'wilor':WiLoR,'hamer':HaMeR}
