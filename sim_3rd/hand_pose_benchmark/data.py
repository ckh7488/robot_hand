"""Read existing image/GT archives without unpacking or modifying originals."""
import io
import json
import pathlib
import pickle
import zipfile

import numpy as np

DATA = pathlib.Path(r'E:\hand_recog\datasets')


class Samples:
    def __init__(self, phase='eval'):
        self.phase = phase
        self.zips = {}
        self.annotations = {}
        self.rows = []
        self.inventory = []
        self._freihand()
        self._rhd()
        self._inspection()

    def zip(self, path):
        path = str(path)
        if path not in self.zips:
            self.zips[path] = zipfile.ZipFile(path)
        return self.zips[path]

    def _freihand(self):
        image_path = DATA / 'raw/freihand/FreiHAND_pub_v2.zip'
        label_path = DATA / 'raw/freihand/FreiHAND_pub_v2_eval.zip'
        image_zip = self.zip(image_path)
        for split in (['evaluation'] if self.phase == 'eval' else ['training']):
            label_zip = self.zip(label_path) if split == 'evaluation' else image_zip
            xyz = np.array(json.loads(label_zip.read(split + '_xyz.json')), dtype=np.float32)
            K = np.array(json.loads(label_zip.read(split + '_K.json')), dtype=np.float32)
            self.annotations['freihand_' + split] = (xyz, K)
            names = sorted(n for n in image_zip.namelist()
                           if n.startswith(split + '/rgb/') and n.endswith('.jpg'))
            self.inventory.append(dict(dataset='FreiHAND', split=split, images=len(names),
                                       distinct_pose_labels=len(xyz), status='available'))
            for n in names:
                i = int(pathlib.PurePosixPath(n).stem)
                self.rows.append(dict(id=f'freihand/{split}/{i:08d}', dataset='FreiHAND', split=split,
                                      archive=str(image_path), member=n, label_index=i % len(xyz),
                                      family=f'freihand_hanco/{split}/{i % len(xyz)}'))

    def _rhd(self):
        path = DATA / 'raw/rhd/RHD_v1-1.zip'
        archive = self.zip(path)
        for split in (['evaluation'] if self.phase == 'eval' else ['training']):
            prefix = f'RHD_published_v2/{split}'
            # Official locally acquired RHD annotation pickle, never execute untrusted uploads.
            ann = pickle.loads(archive.read(f'{prefix}/anno_{split}.pickle'))
            self.annotations['rhd_' + split] = ann
            self.inventory.append(dict(dataset='RHD', split=split, images=len(ann), status='available'))
            for i in sorted(ann):
                self.rows.append(dict(id=f'rhd/{split}/{i:05d}', dataset='RHD', split=split,
                                      archive=str(path), member=f'{prefix}/color/{i:05d}.png',
                                      label_index=i, family=f'rhd/{split}/{i}'))

    def _inspection(self):
        if self.phase != 'eval':
            return
        folder = DATA / 'inspection/quality-review-20261014/hanco-complete-samples'
        for p in sorted(folder.glob('*-rgb.jpg')):
            labels = p.with_name(p.name.replace('-rgb.jpg', '-labels.json'))
            if labels.exists():
                self.rows.append(dict(id='hanco/inspection/' + p.stem, dataset='HanCo sample',
                                      split='inspection_only', file=str(p), labels=str(labels),
                                      family='freihand_hanco/inspection/' + p.stem))
        folder = DATA / 'inspection/quality-review-20261015/interhand-train-H-partbi'
        for p in sorted(folder.glob('*-rgb.jpg')):
            labels=p.with_name(p.name.replace('-rgb.jpg','-labels.json'))
            if labels.exists():
                self.rows.append(dict(id='interhand/inspection/'+p.stem,dataset='InterHand sample',
                                      split='train_inspection_only',file=str(p),labels=str(labels),family='interhand/Capture5'))
        folder=DATA/'inspection/quality-review-20261015/reinterhand-participant0-prefix'
        for p in sorted(folder.glob('*-RGB.png')):
            self.rows.append(dict(id='reinterhand/inspection/'+p.stem,dataset='ReInterHand sample',
                                  split='inspection_no_mapped_gt',file=str(p),family='reinterhand/participant0'))

    def __len__(self):
        return len(self.rows)

    def read(self, i):
        import cv2
        row = self.rows[i]
        payload = self.zip(row['archive']).read(row['member']) if 'archive' in row else pathlib.Path(row['file']).read_bytes()
        image = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError('Image decoding failed: ' + row['id'])
        h, w = image.shape[:2]
        hands = []
        if row['dataset'] == 'FreiHAND':
            all_xyz, all_K = self.annotations['freihand_' + row['split']]
            xyz, K = all_xyz[row['label_index']], all_K[row['label_index']]
            p = xyz @ K.T
            uv = p[:, :2] / p[:, 2:]
            hands.append(dict(xyz=xyz, uv=uv, valid=np.ones(21, bool), visible=None, side=1))
        elif row['dataset'] == 'RHD':
            ann = self.annotations['rhd_' + row['split']][row['label_index']]
            xyz = np.asarray(ann['xyz'], dtype=np.float32)
            uv_vis = np.asarray(ann['uv_vis'], dtype=np.float32)
            # RHD native order: wrist, tip->base for each finger. Canonical: wrist, base->tip.
            order = np.array([0,4,3,2,1,8,7,6,5,12,11,10,9,16,15,14,13,20,19,18,17])
            for j, side in ((0, 0), (21, 1)):
                uv = uv_vis[j:j+21, :2][order]
                visible = uv_vis[j:j+21, 2][order] > 0.5
                if visible.sum() < 3:
                    continue
                hands.append(dict(xyz=xyz[j:j+21][order], uv=uv,
                                  valid=np.ones(21, bool), visible=visible, side=side))
        elif row['dataset'] == 'HanCo sample':
            d = json.loads(pathlib.Path(row['labels']).read_text(encoding='utf8'))
            hands.append(dict(xyz=np.array(d['xyz_camera_m']), uv=np.array(d['derived_uv']),
                              valid=np.ones(21, bool), visible=None, side=None))
        elif row['dataset'] == 'InterHand sample':
            d=json.loads(pathlib.Path(row['labels']).read_text(encoding='utf8'))
            xyz=np.asarray(d['xyz_camera_mm'])/1000
            uv=np.asarray(d['projected_xy_px'])
            valid=np.asarray(d['joint_valid_wrist_gated']).reshape(-1)>0
            order=np.array([20,3,2,1,0,7,6,5,4,11,10,9,8,15,14,13,12,19,18,17,16])
            for start,side in [(0,1),(21,0)]:
                ids=order+start
                if valid[ids].sum()>=3 and valid[ids[0]]:
                    hands.append(dict(xyz=xyz[ids],uv=uv[ids],valid=valid[ids],visible=None,side=side))
        for hand in hands:
            uv = hand['uv']
            inside = (uv[:,0] >= 0) & (uv[:,0] < w) & (uv[:,1] >= 0) & (uv[:,1] < h)
            hand['match_mask'] = inside & hand['valid']
            if hand['visible'] is not None:
                hand['match_mask'] &= hand['visible']
        return row, image, hands

    def order(self):
        """Evenly interleave dataset/pose ranges; full coverage has no duplicate frame IDs."""
        groups = {}
        for i, row in enumerate(self.rows):
            groups.setdefault(row['dataset'], []).append(i)
        rng = np.random.default_rng(20261026)
        for v in groups.values():
            rng.shuffle(v)
        from itertools import zip_longest
        return [i for group in zip_longest(*groups.values()) for i in group if i is not None]
