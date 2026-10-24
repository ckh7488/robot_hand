"""New proportion and hollow exterior CAD. No motor fit or dynamics approval.

Run in the hidden job runner, with CadQuery available. All outputs stay on D:.
The reused arm is context from frozen V9, whose STL files already use metres.
New part STL and STEP use millimetres; render.py converts only the new meshes.
"""
from pathlib import Path
import json, math, hashlib
import cadquery as cq
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from lightweight_digits import build_segment
from lightweight_thumb import build_thumb_metacarpal

D = Path(__file__).resolve().parent
BASE = D.parent / 'modular_roots_v9'
C = json.loads((D / 'design.json').read_text(encoding='utf-8'))
OUT = D / 'generated'
(OUT / 'meshes').mkdir(parents=True, exist_ok=True)
BLUE = (0.22, 0.38, 0.56)
DARK = (0.27, 0.31, 0.37)
TEAL = (0.22, 0.58, 0.64)
WHITE = (0.89, 0.90, 0.88)
METAL = (0.52, 0.54, 0.58)
rows, solids = [], []
assembly = cq.Assembly(name='Form_candidate_v1_hand')

def poly(points, z, height):
    return cq.Workplane('XY').workplane(offset=z).polyline(points).close().extrude(height).val()

def cylinder(r, height, p, axis=(0, 0, 1)):
    return cq.Solid.makeCylinder(r, height, cq.Vector(*p), cq.Vector(*axis))

def box(x, y, z, p):
    return cq.Workplane('XY').box(x, y, z).translate(p).val()

def add(name, shape, color, role, pos=(0, 0, 0), yaw=0):
    if yaw:
        shape = shape.rotate((0, 0, 0), (0, 0, 1), yaw)
    shape = shape.translate(pos)
    if not shape.isValid() or shape.Volume() <= 0:
        raise ValueError('Invalid CAD solid: ' + name)
    path = OUT / 'meshes' / (name + '.stl')
    cq.exporters.export(shape, str(path), tolerance=0.12, angularTolerance=0.12)
    # Do not compare tessellation-inflated cached bounds with exact STEP bounds.
    b=Bnd_Box(); BRepBndLib.AddOptimal_s(shape.wrapped,b,False,False)
    ext=b.Get()
    rows.append({'name': name, 'asset': name, 'mesh': path.relative_to(D).as_posix(),
        'mesh_units': 'mm', 'color': list(color), 'role': role, 'volume_mm3': shape.Volume(),
        'valid': True, 'solid_count': len(shape.Solids()),
        'bbox_mm': [list(ext[:3]),list(ext[3:])],
        'pos_mm': [0,0,0], 'R': [[1,0,0],[0,1,0],[0,0,1]],
        'sha256_stl': hashlib.sha256(path.read_bytes()).hexdigest()})
    solids.append((name, shape))
    assembly.add(shape, name=name, color=cq.Color(*color))

# The visible palm narrows toward the wrist. The old rectangular rear chassis
# and external thumb motor bar are not retained, rescaled, or hidden in the CAD.
outline = [(-43,0),(43,0),(69,24),(89,64),(90,96),(81,121),
           (45,137),(20,150),(-22,143),(-64,126),(-90,101),(-85,43),(-63,12)]
inner = [(x*0.94, 72+(y-72)*0.94) for x,y in outline]
outer = poly(outline, -44, 48)
chassis = outer.cut(poly(inner, -40.5, 49))
# Open thumb sweep region is a proportion provision, not a swept-volume proof.
chassis = chassis.cut(box(42, 62, 45, (91,45,-9)))
add('Palm_tapered_chassis', chassis, BLUE, 'new_palm')

cover_outline = [(-43,0),(34,0),(38,19),(29,31),(47,56),(62,76),(78,92),
                 (70,108),(24,119),(-20,116),(-58,102),(-78,79),(-76,43),(-58,14)]
cover = poly(cover_outline, 10.5, 3.5)
for x,y in [(-44,33),(16,29),(-54,78),(46,88),(-18,104),(15,10)]:
    cover = cover.cut(cylinder(1.8,5,(x,y,10)))
    head = cylinder(3.3,1.4,(x,y,14))
    head = head.cut(box(3.8,0.8,1,(x,y,15)))
    add('Cover_screw_%s_%s'%(x,y),head,METAL,'provisional_fastener')
add('Palm_shaped_front_cover',cover,WHITE,'new_palm_cover')
# Edge rail and diagonal shoulder webs give an actual hollow side structure.
for x in [-71,71]:
    add('Palm_side_web_'+str(x),box(3,55,48,(x,72,-16)),BLUE,'new_palm')
for x,y in [(-44,33),(16,29),(-54,78),(46,88),(-18,104),(15,10)]:
    post=cylinder(4.8,51,(x,y,-40.5)).cut(cylinder(1.8,52,(x,y,-40.5)))
    add('Cover_post_%s_%s'%(x,y),post,BLUE,'new_palm')

# Preserve the four V9 wrist saddle hole datums with a new bridge. This is an
# interface provision only; bolt selection/load transfer remain to be checked.
bridge=box(54,34,4,(-12,18,-61))
for x,y in [(-32,8),(-32,28),(8,8),(8,28)]:
    bridge=bridge.cut(cylinder(1.7,5,(x,y,-63.5)))
    post=cylinder(5.5,15,(x,y,-59)).cut(cylinder(1.7,16,(x,y,-59)))
    add('Wrist_bridge_post_%s_%s'%(x,y),post,BLUE,'new_wrist_interface')
add('Wrist_bridge_plate',bridge,BLUE,'new_wrist_interface')

def root_joint(name, root, yaw=0, thumb=False):
    x,y,z=root
    col=TEAL if thumb else BLUE
    # Two axes are represented as a spread pivot and a flexion eye; these are
    # new passive housings, not reused XL330 motor mounts or a certified joint.
    base=cylinder(15.5,5,(0,0,-10)).cut(cylinder(4,6,(0,0,-10.5)))
    add(name+'_spread_housing',base,col,'new_joint',root,yaw)
    ring=cylinder(10.5,3,(0,0,-5)).cut(cylinder(4,4,(0,0,-5.5)))
    add(name+'_spread_ring',ring,DARK,'new_joint',root,yaw)
    for xx in [-13,13]:
        cheek=cylinder(8.5,3,(xx-1.5,0,0),(1,0,0)).cut(cylinder(2.15,4,(xx-2,0,0),(1,0,0)))
        web=box(3,10,11,(xx,-2,-5.5))
        add(name+'_root_fork_'+str(xx),cheek.fuse(web),col,'new_joint',root,yaw)

def pin(name, pos, yaw=0):
    shape=cylinder(2,31,(-15.5,0,0),(1,0,0))
    shape=shape.fuse(cylinder(3.4,1.1,(-16.6,0,0),(1,0,0)))
    add(name,shape,METAL,'provisional_joint_pin',pos,yaw)

for f in C['fingers']:
    root_joint(f['name'],f['root'])
    x,y,z=f['root']
    for j,L in enumerate(f['links']):
        for part in build_segment(L,tip=j==2):
            add(f['name']+'_segment%d_'%(j+1)+part['name'],part['shape'],part['color'],'new_finger',(x,y,z))
        pin(f['name']+'_joint_pin_'+str(j),(x,y,z))
        y += L

t=C['thumb']; p=list(t['root']); yaw=t['base_yaw_deg']
root_joint('thumb',p,yaw,True)
for j,L in enumerate(t['links']):
    thumb_parts=build_thumb_metacarpal(L,t['second_yaw_deg']) if j==0 else build_segment(L,tip=j==2)
    for part in thumb_parts:
        col=part['color']
        if 'shell' in part['name'] or 'structure' in part['name'] or 'body' in part['name']:
            # Keep the dark inset dark; structural blue becomes reference teal.
            if col[2]>col[0]+0.12:
                col=TEAL
        add('thumb_segment%d_'%(j+1)+part['name'],part['shape'],col,'new_thumb',p,yaw)
    pin('thumb_joint_pin_'+str(j),p,yaw)
    a=math.radians(yaw);p=[p[0]-math.sin(a)*L,p[1]+math.cos(a)*L,p[2]]
    if j==0:yaw += t['second_yaw_deg']

print('Exporting hand STEP',len(rows),'parts',flush=True)
assembly.save(str(OUT/'hand_form_candidate.step'))
scene=json.loads((BASE/'scene.json').read_text(encoding='utf-8'))['Open']
arm=[]
for it in scene:
    if it['role'].startswith('arm_'):
        path=BASE/'meshes'/(it['asset']+'.stl')
        arm.append({**it,'mesh':'../modular_roots_v9/meshes/'+it['asset']+'.stl','mesh_units':'m',
                    'sha256_stl':hashlib.sha256(path.read_bytes()).hexdigest(),
                    'reference_only':True})
(OUT/'scene.json').write_text(json.dumps({'hand':rows,'arm_reference':arm},indent=2),encoding='utf-8')
report={'status':'CAD exterior study only','new_hand_part_count':len(rows),
        'all_exported_shapes_valid':all(r['valid'] for r in rows),
        'new_hand_volume_mm3':sum(r['volume_mm3'] for r in rows),
        'palm_width_reduction_percent':(1-180/232)*100,
        'palm_length_reduction_percent':(1-150/195)*100,
        'middle_link_sum_mm':154,
        'middle_to_palm_length_ratio':154/150,
        'old_middle_to_palm_length_ratio':154/195,
        'actuators_installed':0,'root_motor_packaging_checked':False,
        'continuous_clearance_checked':False,'cable_routes_checked':False,'dynamics_tested':False,
        'limitations':['No full motor arrangement behind palm covers. Ten root motors and ten forearm motors still need placement.',
          'Joint bores and passive exterior housings are represented; complete transmission, bearings, tolerances and fastening are unfinished.',
          'V9 arm is a reused reference, with nominal arm motor envelopes and unverified new hand interface.',
          'Hollow exterior solid volumes are not whole-robot mass or inertia results.',
          'No previous latency number is transferred to this geometry.'],
        'input_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [D/'design.json',D/'build.py',D/'lightweight_digits.py',D/'lightweight_thumb.py']}}
(OUT/'geometry_review.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report),flush=True)
