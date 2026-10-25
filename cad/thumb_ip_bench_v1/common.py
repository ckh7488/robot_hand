"""Shared saved-CAD I/O and rigid frames for the isolated thumb IP bench.

Import is inert. Millimetres; IP origin (0,0,0), rotation +X. Source shapes
stay unchanged and are placed using explicit rigid matrices in the scene.
"""
from pathlib import Path, PurePosixPath
import hashlib
import importlib.util
import json
import math

CAD = 'cad/thumb_ip_bench_v1'
SOURCE = 'cad/thumb_ip_drive_v1'
BASE_SHA = 'a35750de1a1075e27ad14e15a89ddef26f767a48'
SOURCE_MANIFEST_SHA = '974d5a8ac3e8f46f57f0d7a5abeb14fbfa301ac0e4b0e7533769a3557603ed4f'
PARENT = 'thumb_segment2_segment_shell'
EXCLUDED = ['thumb_distal1_child_left_bushing', 'thumb_distal1_child_right_bushing']


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1048576), b''):
            h.update(chunk)
    return h.hexdigest()


def safe_path(root, value):
    p = PurePosixPath(str(value).replace('\\', '/'))
    if p.is_absolute() or '..' in p.parts or ':' in str(p) or not p.parts:
        raise ValueError('Unsafe relative input: ' + str(value))
    result = (Path(root) / str(p)).resolve()
    if not result.is_relative_to(Path(root).resolve()):
        raise ValueError('Input escaped its declared root')
    return result


class Inputs:
    def __init__(self, repo, generated=None):
        self.repo = Path(repo).resolve()
        self.generated = Path(generated).resolve() if generated else None
        self.hashes = {}
        self.paths = {}

    def bind(self, path, expected=None):
        p = Path(path).resolve()
        if self.generated and p.is_relative_to(self.generated):
            key = 'generated:' + p.relative_to(self.generated).as_posix()
        elif p.is_relative_to(self.repo):
            key = 'repo:' + p.relative_to(self.repo).as_posix()
        else:
            raise ValueError('Input outside declared roots: ' + str(p))
        value = digest(p)
        if expected is not None and value != expected:
            raise ValueError('Hash mismatch: ' + key)
        if key in self.hashes and self.hashes[key] != value:
            raise ValueError('Input changed: ' + key)
        self.hashes[key] = value
        self.paths[key] = p
        return p

    def read(self, path):
        return json.loads(self.bind(path).read_text(encoding='utf-8-sig'))

    def module(self, name, path):
        spec = importlib.util.spec_from_file_location(name, self.bind(path))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def guard(self):
        after = {k: digest(p) for k, p in self.paths.items()}
        return dict(passed=after == self.hashes, start=dict(self.hashes), end=after)

    def resolve_part(self, row, mesh=False):
        root = self.generated if row['location'] == 'generated' else self.repo
        if root is None or row['location'] not in ('generated', 'repo'):
            raise ValueError('Invalid part location')
        return self.bind(safe_path(root, row['mesh' if mesh else 'step']),
                         row['sha256_stl' if mesh else 'sha256_step'])


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False,
                                    allow_nan=False) + '\n', encoding='utf-8')


def validate_coverage(scene, inherited):
    """Fail closed on missing/duplicate physical parts and assembly/contact rows."""
    from collections import Counter
    jaws = [f'bench_parent_saddle{i}_{side}' for i in (1, 2) for side in ('left', 'right')]
    clamps = [f'bench_parent_saddle{i}_clamp' for i in (1, 2)]
    motor = [f'bench_motor_mount_{i}' for i in range(1, 5)]
    parent = [f'bench_parent_mount_{i}' for i in range(1, 5)]
    feet = [f'bench_table_foot_{i}' for i in range(1, 5)]
    new = set(jaws + ['bench_common_plate', 'drive_bench_baseplate'])
    for prefix in clamps:
        new.update(prefix + suffix for suffix in ('_screw', '_nut'))
    for prefix in motor + parent + feet:
        new.update(prefix + suffix for suffix in ('_screw', '_nut', '_upper_washer'))
        if prefix not in feet:
            new.add(prefix + '_lower_washer')
        if prefix in motor:
            new.add(prefix + '_spacer')
        if prefix in feet:
            new.add(prefix)
    expected = ({r['name'] for r in inherited} - {'drive_bench_baseplate'}) | new
    actual = [r['name'] for r in scene['parts']]
    if len(expected) != 124 or len(actual) != 124 or set(actual) != expected:
        raise ValueError('Physical coverage must be exactly 124 named parts')
    expected_assembly = {(p + '_screw', 'axial_screw') for p in clamps + motor + parent + feet}
    expected_assembly |= {(p + '_nut', 'axial_nut') for p in clamps + motor + parent + feet}
    expected_assembly |= {(p, 'lateral_jaw') for p in jaws}
    declarations = [(r['part'], r['kind']) for r in scene['assembly']]
    if len(declarations) != 32 or set(declarations) != expected_assembly:
        raise ValueError('Expected 14 screw, 14 nut and four jaw access declarations')
    pairs = []
    def pair(a, b):
        pairs.append(tuple(sorted((a, b))))
    for j in jaws:
        pair(j, PARENT); pair(j, PARENT)
    for i, prefix in enumerate(clamps, 1):
        pair(prefix + '_screw', f'bench_parent_saddle{i}_left')
        pair(prefix + '_nut', f'bench_parent_saddle{i}_right')
    for prefix in motor + parent + feet:
        pair(prefix + '_screw', prefix + '_upper_washer')
        if prefix not in feet:
            pair(prefix + '_nut', prefix + '_lower_washer')
    for prefix in motor:
        pair(prefix + '_spacer', 'bench_common_plate')
        pair(prefix + '_spacer', 'drive_bench_baseplate')
        pair(prefix + '_upper_washer', 'drive_bench_baseplate')
        pair(prefix + '_lower_washer', 'bench_common_plate')
    for prefix in feet:
        pair(prefix, 'bench_common_plate')
        pair(prefix + '_upper_washer', 'bench_common_plate')
        pair(prefix + '_nut', prefix)
    for prefix, jaw in zip(parent, jaws):
        pair(jaw, 'bench_common_plate')
        pair(prefix + '_upper_washer', jaw)
        pair(prefix + '_lower_washer', 'bench_common_plate')
    actual_pairs = [tuple(sorted((r['a'], r['b']))) for r in scene['contacts']]
    if len(pairs) != 72 or Counter(pairs) != Counter(actual_pairs):
        raise ValueError('Expected all 72 named seating/contact declarations')
    signatures = set()
    for r in scene['contacts']:
        coordinates = r['plane_origin_mm'] + r['normal']
        if len(coordinates) != 6 or not all(math.isfinite(v) for v in coordinates):
            raise ValueError('Invalid contact plane')
        if not math.isfinite(r['minimum_area_mm2']) or r['minimum_area_mm2'] <= 0:
            raise ValueError('Invalid contact area requirement')
        signature = (tuple(sorted((r['a'], r['b']))), tuple(coordinates),
                     json.dumps(r.get('patch_box_mm', r.get('patch_radius_mm')), sort_keys=True))
        if signature in signatures:
            raise ValueError('Duplicate contact patch')
        signatures.add(signature)
    return dict(physical_parts=124, contacts=72, screws=14, nuts=14, jaws=4)


def translation(xyz):
    import numpy as np
    t = np.eye(4)
    t[:3, 3] = xyz
    return t


def rotation_x(degrees):
    import numpy as np
    a = math.radians(degrees)
    t = np.eye(4)
    t[1:3, 1:3] = [[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]]
    return t


def world_to_ip(frame):
    import numpy as np
    a, b = map(math.radians, [frame['roll'], frame['yaw']])
    ry = np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0],
                   [-math.sin(a), 0, math.cos(a)]])
    rz = np.array([[math.cos(b), -math.sin(b), 0],
                   [math.sin(b), math.cos(b), 0], [0, 0, 1]])
    t = np.eye(4)
    t[:3, :3] = (rz @ ry).T
    t[:3, 3] = -t[:3, :3] @ np.asarray(frame['origin'])
    return t


def placed_matrix(row, q=0.):
    import numpy as np
    neutral = np.asarray(row['neutral_matrix'], dtype=float)
    if row['body'] == 'ip_output':
        return rotation_x(q) @ neutral
    if row['body'] == 'motor_output':
        t = translation([-6., -120., 0.])
        return t @ rotation_x(.77 * q) @ np.linalg.inv(t) @ neutral
    if row['body'] != 'fixed':
        raise ValueError('Unknown body: ' + row['body'])
    return neutral


def rigid(cq, shape, matrix):
    from OCP.gp import gp_Trsf
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    t = gp_Trsf()
    t.SetValues(*[float(matrix[i][j]) for i in range(3) for j in range(4)])
    return cq.Shape.cast(BRepBuilderAPI_Transform(shape.wrapped, t, True).Shape())


def bounds(shape):
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    box = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape.wrapped, box, False, False)
    return list(box.Get())


def overlap(a, b, eps=1e-7):
    return all(a[i] < b[i+3] - eps and b[i] < a[i+3] - eps for i in range(3))


def cord_segments(cq, routing, joint, anchors, q):
    """Nominal finite cord solids in IP coordinates; no tension solution."""
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeTorus
    from OCP.gp import gp_Ax2, gp_Pnt, gp_Dir
    def cylinder(start, end):
        p = cq.Vector(*start)
        d = cq.Vector(*end) - p
        return cq.Solid.makeCylinder(.3, d.Length, p, d.normalized())
    def arc(x, y, radius, aa, bb):
        lo, hi = sorted([aa, bb])
        if not 0 < hi-lo < 2*math.pi:
            raise ValueError('Invalid working wrap')
        axis = gp_Ax2(gp_Pnt(x, y, 0), gp_Dir(1,0,0), gp_Dir(0, math.cos(lo), math.sin(lo)))
        return cq.Shape.cast(BRepPrimAPI_MakeTorus(axis, radius, .3, hi-lo).Shape())
    tails = joint.tendon_geometry(q)
    result = []
    for i, route in enumerate(routing.route(q)):
        tail = cylinder(anchors[i]['tail_start_mm'], anchors[i]['tail_exit_mm']).rotate((0,0,0),(1,0,0), .77*q).translate((-6.,-120.,0.))
        pieces = dict(motor_tail=tail,
            motor_wrap=arc(route['track_x_mm'], -120., 10., route['motor_anchor_angle_rad'], route['contact_angle_rad']),
            free_span=cylinder(route['motor_contact_mm'], route['joint_contact_mm']),
            joint_wrap=arc(route['track_x_mm'], 0., 7.7, route['contact_angle_rad'], route['joint_anchor_angle_rad']),
            joint_tail=cylinder(tails[i]['anchor_mm'], tails[i]['tail_end_mm']))
        result.extend(dict(name=f'cord_{i}_{label}', shape=shape, track=i, segment=label)
                      for label, shape in pieces.items())
    return result


def source_rows(inputs):
    """Select actual detached coupon: 30 parts, plus 33 separate motor parts."""
    import copy
    old = inputs.repo / SOURCE / 'generated'
    manifest_path = inputs.bind(inputs.repo / SOURCE / 'THUMB_DRIVE001_MANIFEST.json', SOURCE_MANIFEST_SHA)
    manifest = inputs.read(manifest_path)
    for name in ('scene.json', 'build_report.json'):
        relative = SOURCE + '/generated/' + name
        inputs.bind(old / name, manifest['files'][relative]['sha256'])
    scene = inputs.read(old / 'scene.json')
    report = inputs.read(old / 'build_report.json')
    if not (report['completed'] and not report['errors'] and report['input_hash_guard']['passed']):
        raise ValueError('Source build is not complete')
    frame = world_to_ip(scene['thumb_ip_frame'])
    selected = [r for r in scene['hand'] if r['body'] in ('thumb_mcp', 'thumb_ip')]
    if len(selected) != 32 or not set(EXCLUDED) <= {r['name'] for r in selected}:
        raise ValueError('Unexpected coupon part membership')
    result = []
    for original in selected + scene['motor_bench']:
        if original['name'] in EXCLUDED:
            continue
        row = copy.deepcopy(original)
        motor = original in scene['motor_bench']
        for key, hashkey in [('step', 'sha256_step'), ('mesh', 'sha256_stl')]:
            source = old if row['location'] == 'generated' else inputs.repo
            path = inputs.bind(safe_path(source, row[key]), row[hashkey])
            row[key] = path.relative_to(inputs.repo).as_posix()
        row.update(location='repo', source_body=original['body'],
            body=('motor_output' if original['body'] == 'bench_motor_output' else 'fixed') if motor
                  else ('ip_output' if original['body'] == 'thumb_ip' else 'fixed'),
            neutral_matrix=(translation([-6., -120., 0.]) if motor else frame).tolist(),
            origin_group='motor' if motor else 'coupon')
        if 'bbox_mm' in row:
            box = row.pop('bbox_mm')
            row['native_bbox_mm'] = box[0] + box[1] if len(box) == 2 else box
        row['bbox_coordinates'] = 'Native STEP coordinates; apply neutral_matrix for IP placement'
        result.append(row)
    if len(result) != 63:
        raise ValueError('Expected 30 coupon and 33 motor rows')
    return result, report
