"""Articulate the selected CAD's actual meshes and joint frames, in metres.

This is a kinematic preview, not a motor or collision simulation. The source
STLs are in their assembled neutral world pose. Each geom is transformed into
its joint's neutral frame exactly once; no mesh is regenerated or rescaled to fit.
"""
from pathlib import Path
import hashlib
import json
import re
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from .retarget import NAMES, joint_ranges


def _fmt(values):
    return ' '.join(f'{v:.12g}' for v in np.asarray(values).ravel())


def _quat(matrix):
    return _fmt(Rotation.from_matrix(matrix).as_quat()[[3, 0, 1, 2]])


def _rz(degrees):
    return Rotation.from_euler('z', degrees, degrees=True).as_matrix()


def _ry(degrees):
    return Rotation.from_euler('y', degrees, degrees=True).as_matrix()


class CadHand:
    def __init__(self, repo, whole=False, cad='form_candidate_v1'):
        self.source = Path(repo) / 'cad' / cad
        design_path = self.source / 'design.json'
        scene_path = self.source / 'generated' / 'scene.json'
        self.design = json.loads(design_path.read_text(encoding='utf-8'))
        scene = json.loads(scene_path.read_text(encoding='utf-8'))
        self.items = scene['hand'] + (scene['arm_reference'] if whole else [])
        self.hashes = {str(p.relative_to(Path(repo))): hashlib.sha256(p.read_bytes()).hexdigest()
                       for p in (design_path, scene_path)}
        root = ET.Element('mujoco', model='CAD hand pose preview')
        ET.SubElement(root, 'compiler', angle='radian', autolimits='true')
        ET.SubElement(root, 'option', gravity='0 0 0')
        visual = ET.SubElement(root, 'visual')
        ET.SubElement(visual, 'global', offwidth='1000', offheight='900')
        ET.SubElement(visual, 'headlight', ambient='.5 .5 .5', diffuse='.6 .6 .6', specular='.15 .15 .15')
        ET.SubElement(visual, 'quality', shadowsize='2048')
        assets = ET.SubElement(root, 'asset')
        ET.SubElement(assets, 'texture', type='skybox', builtin='flat', rgb1='.075 .10 .14',
                      rgb2='.075 .10 .14', width='256', height='1536')
        world = ET.SubElement(root, 'worldbody')
        ET.SubElement(world, 'light', pos='.3 .4 .8', diffuse='.6 .6 .6')
        self.neutral_frames = {'world': (np.zeros(3), np.eye(3))}
        bodies = {'world': world}
        limits = joint_ranges()
        if 'joint_ranges_deg' in self.design:
            limits = np.array([self.design['joint_ranges_deg'][n if n.startswith('thumb') else n.split('_', 1)[1]] for n in NAMES], float)
        configs = [*self.design['fingers'], dict(self.design['thumb'], name='thumb')]
        for fi, config in enumerate(configs):
            name = config['name']
            origin = np.asarray(config['root'], float) / 1000
            rotation = _rz(config.get('base_yaw_deg', 0))
            parent = world
            for ji in range(4):
                joint = NAMES[4 * fi + ji]
                relative_pos = origin if ji == 0 else np.zeros(3)
                relative_rotation = rotation if ji == 0 else np.eye(3)
                if ji == 1:
                    relative_pos = np.asarray(config.get('flexion_offset', [0, 0, 0]), float) / 1000
                    origin += rotation @ relative_pos
                if ji >= 2:
                    relative_pos = np.array([0, config['links'][ji - 2] / 1000, 0])
                    origin = origin + rotation @ relative_pos
                    if ji == 2:
                        relative_rotation = _rz(config.get('second_yaw_deg', 0)) @ _ry(config.get('second_roll_deg', 0))
                        rotation = rotation @ relative_rotation
                body = ET.SubElement(parent, 'body', name=joint,
                                     pos=_fmt(relative_pos), quat=_quat(relative_rotation))
                ET.SubElement(body, 'inertial', pos='0 0 0', mass='.01', diaginertia='.00001 .00001 .00001')
                ET.SubElement(body, 'joint', name=joint, type='hinge',
                              axis='0 0 -1' if ji == 0 else '1 0 0',
                              range=_fmt(np.radians(limits[4 * fi + ji])))
                if ji > 0:
                    ET.SubElement(body, 'site', name=joint + '_tip',
                                  pos=_fmt([0, config['links'][ji - 1] / 1000, 0]), size='.001', rgba='0 0 0 0')
                bodies[joint] = body
                self.neutral_frames[joint] = (origin.copy(), rotation.copy())
                parent = body
        self.part_joint = {}
        for index, item in enumerate(self.items):
            mesh_path = (self.source / item['mesh']).resolve()
            digest = hashlib.sha256(mesh_path.read_bytes()).hexdigest()
            if digest != item['sha256_stl']:
                raise ValueError(f'CAD mesh differs from scene manifest: {item["name"]}')
            self.hashes[str(mesh_path.relative_to(Path(repo)))] = digest
            ET.SubElement(assets, 'mesh', name=f'part{index}', file=mesh_path.as_posix(),
                          scale='.001 .001 .001' if item['mesh_units'] == 'mm' else '1 1 1', maxhullvert='64')
            parent_name = self._parent(item['name'])
            self.part_joint[item['name']] = parent_name
            origin, rotation = self.neutral_frames[parent_name]
            ET.SubElement(bodies[parent_name], 'geom', name=item['name'], type='mesh', mesh=f'part{index}',
                          pos=_fmt(-rotation.T @ origin), quat=_quat(rotation.T),
                          rgba=_fmt([*item['color'], 1]), contype='0', conaffinity='0', group='0')
        self.xml = ET.tostring(root, encoding='unicode')
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        self.addresses = np.array([self.model.jnt_qposadr[self.model.joint(n).id] for n in NAMES])
        self.ranges = np.array([np.degrees(self.model.jnt_range[self.model.joint(n).id]) for n in NAMES])
        self.renderer = None
        self.whole = whole
        self.set(np.zeros(20))

    @staticmethod
    def _parent(name):
        match = re.match(r'^(index|middle|ring|little|thumb)_(.*)$', name)
        if not match:
            return 'world'
        finger, rest = match.groups()
        offset = ['index', 'middle', 'ring', 'little', 'thumb'].index(finger) * 4
        if rest == 'spread_housing':
            return 'world'
        if rest.startswith(('spread_ring', 'root_fork')):
            return NAMES[offset]
        segment = re.match(r'segment([123])_', rest)
        if segment:
            return NAMES[offset + int(segment[1])]
        pin = re.match(r'joint_pin_([012])$', rest)
        if pin:
            return NAMES[offset + int(pin[1])]
        raise ValueError(f'Unassigned CAD part: {name}')

    def set(self, degrees):
        angles = np.asarray(degrees, float)
        if angles.shape != (20,) or not np.isfinite(angles).all():
            raise ValueError('Expected twenty finite joint angles')
        self.data.qpos[self.addresses] = np.radians(np.clip(angles, *self.ranges.T))
        mujoco.mj_forward(self.model, self.data)

    @property
    def angles(self):
        return np.degrees(self.data.qpos[self.addresses]).copy()

    def render(self, view=0, width=880, height=760, axes=False, *, focus=None, distance=None, fast=False):
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.model, height=height, width=width)
        cam = mujoco.MjvCamera()
        cam.lookat[:] = [0.025, .14, .015] if not self.whole else [0, -.07, -.025]
        cam.distance = .53 if not self.whole else 1.05
        if hasattr(self, 'camera_focus'):
            cam.lookat[:] = self.camera_focus
            cam.distance = self.camera_distance
        if focus is not None:
            cam.lookat[:] = focus
        if distance is not None:
            cam.distance = distance
        cam.azimuth, cam.elevation = getattr(self, 'camera_views', [(95, -72), (135, -32), (180, -5)])[view % 3]
        self.renderer.update_scene(self.data, cam)
        self.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = not fast
        if axes:
            colors = [(1., .3, .3, 1), (.3, 1., .3, 1), (.35, .6, 1., 1), (1., .8, .2, 1)]
            for name, color in zip(NAMES[16:], colors):
                jid = self.model.joint(name).id
                scene = self.renderer.scene
                geom = scene.geoms[scene.ngeom]
                mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_ARROW,
                                   np.zeros(3), np.zeros(3), np.eye(3).ravel(), np.array(color))
                origin = self.data.xanchor[jid]
                mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_ARROW, .002,
                                     origin - .055 * self.data.xaxis[jid],
                                     origin + .055 * self.data.xaxis[jid])
                scene.ngeom += 1
        for eye in self.renderer.scene.camera:
            forward = np.asarray(eye.forward)
            up = np.array([0., 1., 0.])
            up -= forward * np.dot(up, forward)
            eye.up[:] = up / np.linalg.norm(up)
        return self.renderer.render().copy()

    def close(self):
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None
