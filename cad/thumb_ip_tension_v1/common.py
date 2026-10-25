"""Saved-CAD interfaces for an isolated, adjustable thumb IP test fixture.

Millimetres. Existing bench inputs are read and hash-bound, never overwritten.
No claim of powered operation, measured preload, or full-hand integration.
"""
from pathlib import Path, PurePosixPath
import copy
import hashlib
import importlib.util
import json
import math

CAD = 'cad/thumb_ip_tension_v1'
BENCH = 'cad/thumb_ip_bench_v1'
SOURCE = 'cad/thumb_ip_drive_v1'
BASE_SHA = '004f33f8f2e343d74636681ce473a8c586da8d9f'
BENCH_MANIFEST_SHA = 'd62a9dc91b8eb08953afb4c5d9c1651f29cc20117b1c21737b76466d91634dc2'
DRIVE_MANIFEST_SHA = '974d5a8ac3e8f46f57f0d7a5abeb14fbfa301ac0e4b0e7533769a3557603ed4f'
POSES = [-20., -15., 0., 15., 30., 45., 60., 75., 85.]
DISTANCES = [118., 120., 122.]

def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(1048576), b''): h.update(chunk)
    return h.hexdigest()

def safe_path(root, value):
    p = PurePosixPath(str(value).replace('\\', '/'))
    if p.is_absolute() or '..' in p.parts or ':' in str(p) or not p.parts:
        raise ValueError('Unsafe relative input')
    result = (Path(root) / str(p)).resolve()
    if not result.is_relative_to(Path(root).resolve()): raise ValueError('Escaped input root')
    return result

class Inputs:
    def __init__(self, repo, generated=None):
        self.repo = Path(repo).resolve()
        self.generated = Path(generated).resolve() if generated else None
        self.hashes, self.paths = {}, {}
    def bind(self, path, expected=None):
        p = Path(path).resolve()
        if self.generated and p.is_relative_to(self.generated):
            key = 'generated:' + p.relative_to(self.generated).as_posix()
        elif p.is_relative_to(self.repo): key = 'repo:' + p.relative_to(self.repo).as_posix()
        else: raise ValueError('Input outside declared roots')
        value = digest(p)
        if expected is not None and value != expected: raise ValueError('Hash mismatch: '+key)
        if key in self.hashes and self.hashes[key] != value: raise ValueError('Input changed: '+key)
        self.hashes[key], self.paths[key] = value, p
        return p
    def read(self, path): return json.loads(self.bind(path).read_text(encoding='utf-8-sig'))
    def module(self, name, path):
        spec = importlib.util.spec_from_file_location(name, self.bind(path))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    def guard(self):
        after = {k:digest(p) for k,p in self.paths.items()}
        return dict(passed=after == self.hashes, start=dict(self.hashes), end=after)
    def resolve_part(self, row, mesh=False):
        if row['location'] not in ('generated','repo'): raise ValueError('Invalid location')
        root = self.generated if row['location']=='generated' else self.repo
        if root is None: raise ValueError('Missing generated root')
        return self.bind(safe_path(root,row['mesh' if mesh else 'step']),row['sha256_stl' if mesh else 'sha256_step'])

def write_json(path, value):
    Path(path).write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf-8')

def translation(xyz):
    import numpy as np
    t=np.eye(4); t[:3,3]=xyz
    return t

def rotation_x(degrees):
    import numpy as np
    a=math.radians(degrees); t=np.eye(4)
    t[1:3,1:3]=[[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]]
    return t

def placed_matrix(row,q=0.,distance=120.):
    import numpy as np
    neutral=np.asarray(row['neutral_matrix'],dtype=float)
    body=row['body']; s=120.-float(distance)
    if body=='ip_output': return rotation_x(q) @ neutral
    if body=='motor_output':
        t=translation([-6.,-120.,0.])
        return translation([0,s,0]) @ t @ rotation_x(.77*q) @ np.linalg.inv(t) @ neutral
    if body=='carriage': return translation([0,s,0]) @ neutral
    if body=='adjuster_rotor':
        axis=row['adjuster_axis']
        if len(axis)!=3 or not np.isfinite(axis).all() or not math.isfinite(row['travel_mm_per_revolution']) or row['travel_mm_per_revolution']==0:
            raise ValueError('Invalid adjuster frame/pitch')
        # Explicit rigid rotation about the declared Y-axis; signed pitch is
        # supplied by the geometry module. No helical threads are simulated.
        a=math.radians(s/row['travel_mm_per_revolution']*360.)
        ry=np.eye(4); ry[:3,:3]=[[math.cos(a),0,math.sin(a)],[0,1,0],[-math.sin(a),0,math.cos(a)]]
        t=translation(axis)
        return t @ ry @ np.linalg.inv(t) @ neutral
    if body!='fixed': raise ValueError('Unknown rigid body: '+body)
    return neutral

def rigid(cq, shape, matrix):
    from OCP.gp import gp_Trsf
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    t=gp_Trsf(); t.SetValues(*[float(matrix[i][j]) for i in range(3) for j in range(4)])
    return cq.Shape.cast(BRepBuilderAPI_Transform(shape.wrapped,t,True).Shape())

def bounds(shape):
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    b=Bnd_Box(); BRepBndLib.AddOptimal_s(shape.wrapped,b,False,False)
    return list(b.Get())

def overlap(a,b,eps=1e-7):
    return all(a[i]<b[i+3]-eps and b[i]<a[i+3]-eps for i in range(3))

def source_rows(inputs):
    manifest=inputs.read(inputs.bind(inputs.repo/BENCH/'THUMB_BENCH001_MANIFEST.json',BENCH_MANIFEST_SHA))
    root=inputs.repo/BENCH/'generated'
    for name in ('scene.json','build_report.json'):
        inputs.bind(root/name,manifest['files'][BENCH+'/generated/'+name]['sha256'])
    scene=inputs.read(root/'scene.json'); report=inputs.read(root/'build_report.json')
    if not (report['completed'] and not report['errors'] and report['input_hash_guard']['passed']):
        raise ValueError('Incomplete bench source')
    result=[]
    for old in scene['parts']:
        row=copy.deepcopy(old)
        for key,h in [('step','sha256_step'),('mesh','sha256_stl')]:
            source=root if row['location']=='generated' else inputs.repo
            p=inputs.bind(safe_path(source,row[key]),row[h])
            row[key]=p.relative_to(inputs.repo).as_posix()
        row['location']='repo'
        if row['origin_group']=='motor' and row['body']=='fixed': row['body']='carriage'
        if row['name']=='drive_bench_baseplate': row['body']='carriage'
        result.append(row)
    if len(result)!=124 or len({r['name'] for r in result})!=124: raise ValueError('Bench coverage changed')
    return result,scene,report

def inherited_path(inputs,relative):
    """Bind executable source/anchor facts to the frozen bench dependency set."""
    manifest=inputs.read(inputs.bind(inputs.repo/BENCH/'THUMB_BENCH001_MANIFEST.json',BENCH_MANIFEST_SHA))
    if relative not in manifest['files'] and relative.startswith(SOURCE+'/source_docs/'):
        manifest=inputs.read(inputs.bind(inputs.repo/SOURCE/'THUMB_DRIVE001_MANIFEST.json',DRIVE_MANIFEST_SHA))
    if relative not in manifest['files']: raise ValueError('Unbound inherited dependency '+relative)
    return inputs.bind(safe_path(inputs.repo,relative),manifest['files'][relative]['sha256'])

def cord_segments(cq,routing,joint,anchors,q,distance=120.):
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeTorus
    from OCP.gp import gp_Ax2,gp_Pnt,gp_Dir
    def cylinder(start,end):
        p=cq.Vector(*start); d=cq.Vector(*end)-p
        return cq.Solid.makeCylinder(.3,d.Length,p,d.normalized())
    def arc(x,y,radius,aa,bb):
        lo,hi=sorted([aa,bb])
        if not 0<hi-lo<2*math.pi: raise ValueError('Invalid wrap')
        axis=gp_Ax2(gp_Pnt(x,y,0),gp_Dir(1,0,0),gp_Dir(0,math.cos(lo),math.sin(lo)))
        return cq.Shape.cast(BRepPrimAPI_MakeTorus(axis,radius,.3,hi-lo).Shape())
    tails=joint.tendon_geometry(q); result=[]
    for i,route in enumerate(routing.route(q,distance_mm=distance)):
        tail=cylinder(anchors[i]['tail_start_mm'],anchors[i]['tail_exit_mm']).rotate((0,0,0),(1,0,0),.77*q).translate((-6.,-distance,0.))
        pieces=dict(motor_tail=tail,motor_wrap=arc(route['track_x_mm'],-distance,10.,route['motor_anchor_angle_rad'],route['contact_angle_rad']),
            free_span=cylinder(route['motor_contact_mm'],route['joint_contact_mm']),
            joint_wrap=arc(route['track_x_mm'],0.,7.7,route['contact_angle_rad'],route['joint_anchor_angle_rad']),
            joint_tail=cylinder(tails[i]['anchor_mm'],tails[i]['tail_end_mm']))
        result.extend(dict(name=f'cord_{i}_{k}',shape=v,track=i,segment=k) for k,v in pieces.items())
    return result
