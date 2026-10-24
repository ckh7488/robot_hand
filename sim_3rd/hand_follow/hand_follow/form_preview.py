"""Load an authored proportion preview without claiming it is fabrication CAD.

The prepared XML contains meshes already expressed in their joint-local frames.
It must not be loaded through CadHand's world-to-joint mesh conversion again.
"""
from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .cad_hand import CadHand
from .retarget import NAMES


class FormPreviewHand(CadHand):
    def __init__(self, directory):
        self.source = Path(directory).resolve()
        design_path = self.source / 'design.json'
        xml_path = self.source / 'preview.xml'
        self.design = json.loads(design_path.read_text(encoding='utf-8'))
        self.design['preview_label'] = '새 손 비율 · 웹캠 시험'
        # Old geometric-check metadata belongs to the source geometry, not
        # the stretched mesh study. It cannot certify this preview.
        self.design.pop('fist_thumb_clearance_guide', None)
        root = ET.fromstring(xml_path.read_text(encoding='utf-8'))
        paths = {design_path, xml_path}
        for asset in root.findall('./asset/mesh'):
            path = (self.source / asset.get('file')).resolve()
            if not path.is_file():
                raise FileNotFoundError(f'Preview mesh missing: {path}')
            paths.add(path)
            asset.set('file', path.as_posix())
        self.hashes = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                       for path in sorted(paths)}
        self.xml = ET.tostring(root, encoding='unicode')
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        if self.model.njnt != len(NAMES) or self.model.nq != len(NAMES):
            raise ValueError('Proportion preview must contain twenty hinge joints')
        self.data = mujoco.MjData(self.model)
        self.addresses = np.array([self.model.jnt_qposadr[self.model.joint(n).id] for n in NAMES])
        self.ranges = np.array([np.degrees(self.model.jnt_range[self.model.joint(n).id]) for n in NAMES])
        self.items = [dict(name=geom.get('name')) for geom in root.findall('.//geom')]
        self.renderer = None
        self.whole = False
        self.camera_views = [(90, -90), (135, -32), (180, -5)]
        self.camera_focus = [.040, .125, .045]
        self.camera_distance = .47
        self.set(np.zeros(20))
