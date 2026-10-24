"""Separate motion-study CAD derived from form_candidate_v1. No motor hardware.

The source exterior stays untouched. This candidate turns both distal thumb
hinges toward the palm, raises its orthogonal root gimbal, and cuts conservative
child-link sweep relief from parent shells. Sampled clearance is not a
continuous-motion or manufacturing certificate.
"""
import hashlib
import json
import math
from pathlib import Path
import sys
import os
import cadquery as cq
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from digits import build_segment
from thumb_shell import build_thumb_metacarpal

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parent / 'form_candidate_v1'
OUT = HERE / 'generated'
(OUT / 'meshes').mkdir(parents=True, exist_ok=True)
design = json.loads((SOURCE / 'design.json').read_text())
design['revision'] = 'webcam_articulation_v1'
design['status'] = 'Articulated motion-study candidate; no fabrication or drive approval'
design['thumb'].update(second_roll_deg=-65, flexion_offset=[0, 0, 21])
design['joint_ranges_deg'] = dict(spread=[-22,22], mcp=[-20,90], pip=[0,135], dip=[0,90],
    thumb_base_sweep=[-65,55], thumb_opposition=[0,85], thumb_mcp=[0,85], thumb_ip=[0,85])
fist = []
for finger in design['fingers']:
    l1,l2,l3 = finger['links']
    total = 180 + math.degrees(math.asin((l1*math.sin(math.radians(80))-l3-8)/l2))
    fist.extend([0,80,total-80,270-total])
fist.extend([4.36319691,41.77009943,34.46484045,51.76072322])
design['fist_deg'] = fist
thumb_lifted=[0]*20;thumb_lifted[16:18]=[35,fist[17]]
finger_closed=fist[:16]+thumb_lifted[16:]
design['fist_path']=[dict(fraction=t,angles_deg=a) for t,a in
    ((0,[0]*20),(.20,thumb_lifted),(.75,finger_closed),(1,fist))]
(HERE/'design.json').write_text(json.dumps(design,indent=2))
scene = json.loads((SOURCE/'generated/scene.json').read_text())
original = cq.importers.importStep(str(SOURCE/'generated/hand_form_candidate.step')).val().Solids()
parts = {}
for row, solid in zip(scene['hand'],original):
    if abs(solid.Volume()-row['volume_mm3'])>1e-4:
        raise ValueError('Source STEP/scene part order mismatch')
    parts[row['name']] = dict(shape=solid, color=row['color'], role=row['role'])
report = {'source':'cad/form_candidate_v1',
          'relief_method':'Filled child-link envelope, polygonal arcs at <=3 degrees; verified separately in sampled poses',
          'changes':['Thumb root gimbal height21 mm; distal frame roll-65 degrees',
                     'Narrow proximal necks and hinge sweep relief',
                     'Palm cover root clearances'], 'continuous_clearance_verified':False}
CACHE=Path(os.environ.get('HAND_CAD_CACHE','D:/Codex-work/robot-hand-workspace/work/hand_follow_webcam/cad_relief_cache'))
CACHE.mkdir(parents=True,exist_ok=True)

def child_frame(shape, length, yaw=0, roll=0, flex=0):
    return shape.rotate((0,0,0),(1,0,0),flex).rotate((0,0,0),(0,1,0),roll).rotate((0,0,0),(0,0,1),yaw).translate((0,length,0))

def relief(parent, child, length, max_angle, yaw=0, roll=0):
    signature=json.dumps(['envelope_v3',length,max_angle,yaw,roll,
        [hashlib.sha256((HERE/p).read_bytes()).hexdigest() for p in ('digits.py','thumb_shell.py')],
        [(p['name'],round(p['shape'].Volume(),8),round(p['shape'].Area(),8)) for p in parent+child]])
    key=hashlib.sha256(signature.encode()).hexdigest()[:20]
    cached=CACHE/f'{key}.step'
    if cached.exists():
        solids=cq.importers.importStep(str(cached)).val().Solids()
        if len(solids)==len(parent):
            print('Relief cache',key,flush=True)
            return [dict(part,shape=shape) for part,shape in zip(parent,solids)]
    # A conservative, filled envelope avoids fragile Boolean unions of dozens
    # of hollow meshes. Keep the concentric hinge eye/fork outside this cut.
    # Analytic swept envelope of y>=8.7, |z|<=12.85, rotated from 0 to
    # max_angle. Radial clipping at 250 mm is far beyond every parent shell.
    # The inner arc preserves the 8 mm hinge ring and its dorsal support.
    a,d,radius=8.7,12.85,250.0
    outer_y=math.sqrt(radius*radius-d*d)
    theta=math.radians(max_angle);beta=math.asin(d/radius)
    points=[(a,0),(a,-d),(outer_y,-d)]
    steps=max(2,math.ceil((theta+2*beta)/math.radians(3)))
    points.extend((radius*math.cos(-beta+(theta+2*beta)*i/steps),
                   radius*math.sin(-beta+(theta+2*beta)*i/steps)) for i in range(1,steps+1))
    c,s=math.cos(theta),math.sin(theta)
    points.extend([(c*a-s*d,s*a+c*d),(c*a,s*a)])
    steps=max(2,math.ceil(max_angle/2))
    points.extend((a*math.cos(theta*(1-i/steps)),a*math.sin(theta*(1-i/steps))) for i in range(1,steps))
    wire=cq.Wire.makePolygon([cq.Vector(-14.5,y,z) for y,z in points],close=True)
    cutter=cq.Solid.extrudeLinear(wire,[],cq.Vector(29,0,0))
    cutter=child_frame(cutter,length,yaw,roll)
    result=[]
    for part in parent:
        shape=part['shape']
        if part['name'] in ('segment_shell','thenar_shell','dark_faceplate'):
            shape=shape.cut(cutter, tol=1e-5)
            shape=shape.clean()
            if not shape.isValid() or len(shape.Solids())!=1:
                print('Disconnected components',[(s.Volume(),s.Center().toTuple()) for s in shape.Solids()],flush=True)
                cq.exporters.export(shape,str(CACHE/f'failed_{key}_{part["name"]}.step'))
                raise ValueError(f'Relief disconnected {part["name"]}: {len(shape.Solids())} solids')
        result.append(dict(part,shape=shape))
    cq.exporters.export(cq.Compound.makeCompound([p['shape'] for p in result]),str(cached))
    return result

for config in [*design['fingers'],dict(design['thumb'],name='thumb')]:
    name=config['name'];lengths=config['links'];thumb=name=='thumb'
    local=[build_thumb_metacarpal(lengths[0],config['second_yaw_deg']) if thumb else build_segment(lengths[0]),
           build_segment(lengths[1]),build_segment(lengths[2],tip=True)]
    print('Relief',name,'distal',flush=True)
    local[1]=relief(local[1],local[2],lengths[1],90)
    print('Relief',name,'middle',flush=True)
    local[0]=relief(local[0],local[1],lengths[0],85 if thumb else 135,
                    config.get('second_yaw_deg',0),config.get('second_roll_deg',0))
    root=config['root'];offset=config.get('flexion_offset',[0,0,0]);yaw=config.get('base_yaw_deg',0)
    origin=[root[i]+offset[i] for i in range(3)]
    for segment,group in enumerate(local):
        for part in group:
            shape=part['shape']
            if segment:
                shape=shape.rotate((0,0,0),(0,1,0),config.get('second_roll_deg',0))
                shape=shape.rotate((0,0,0),(0,0,1),config.get('second_yaw_deg',0))
                shape=shape.translate((0,lengths[0],0))
            if segment==2:
                # The extra middle length uses the child direction, unchanged
                # by its longitudinal roll.
                a=math.radians(config.get('second_yaw_deg',0))
                shape=shape.translate((-math.sin(a)*lengths[1],math.cos(a)*lengths[1],0))
            shape=shape.rotate((0,0,0),(0,0,1),yaw).translate(origin)
            key=f'{name}_segment{segment+1}_{part["name"]}'
            old=parts[key]
            parts[key]=dict(old,shape=shape)
        # Replace pin directions so physical pins and the new rolled bores agree.
        pin=cq.Solid.makeCylinder(2,31,cq.Vector(-15.5,0,0),cq.Vector(1,0,0))
        pin=pin.fuse(cq.Solid.makeCylinder(3.4,1.1,cq.Vector(-16.6,0,0),cq.Vector(1,0,0)))
        if segment:
            pin=pin.rotate((0,0,0),(0,1,0),config.get('second_roll_deg',0)).rotate((0,0,0),(0,0,1),config.get('second_yaw_deg',0))
            a=math.radians(config.get('second_yaw_deg',0))
            pin=pin.translate((-math.sin(a)*(lengths[1] if segment==2 else 0),lengths[0]+math.cos(a)*(lengths[1] if segment==2 else 0),0))
        parts[f'{name}_joint_pin_{segment}']['shape']=pin.rotate((0,0,0),(0,0,1),yaw).translate(origin)
    if thumb:
        for side in (-13,13):
            key=f'thumb_root_fork_{side}'
            fork=parts[key]['shape'].translate(offset)
            web=cq.Workplane('XY').box(3,9,16).translate((side,-2,5)).val()
            web=web.rotate((0,0,0),(0,0,1),yaw).translate(root)
            parts[key]['shape']=fork.fuse(web).clean()
        platform=cq.Solid.makeCylinder(15,3,cq.Vector(0,0,-4),cq.Vector(0,0,1)).cut(cq.Solid.makeCylinder(4,4,cq.Vector(0,0,-4.5),cq.Vector(0,0,1)))
        parts['thumb_spread_ring']['shape']=platform.translate(root)
    print('Completed',name,flush=True)

# Clear the palm cover around each rotating root support. The finger pads land
# farther toward the wrist, outside these root openings.
cover=parts['Palm_shaped_front_cover']['shape']
for config in [*design['fingers'],design['thumb']]:
    x,y,z=config['root']
    cutter=cq.Solid.makeCylinder(19,30,cq.Vector(x,y,-6),cq.Vector(0,0,1))
    cover=cover.cut(cutter,tol=1e-6)
parts['Palm_shaped_front_cover']['shape']=cover.clean()

assembly=cq.Assembly(name='webcam_articulation_candidate')
rows=[]
for name,part in parts.items():
    shape=part['shape']
    if not shape.isValid() or len(shape.Solids())!=1:
        raise ValueError(f'Invalid/disconnected output: {name} {len(shape.Solids())}')
    path=OUT/'meshes'/f'{name}.stl'
    cq.exporters.export(shape,str(path),tolerance=.1,angularTolerance=.1)
    bounds=Bnd_Box();BRepBndLib.AddOptimal_s(shape.wrapped,bounds,False,False);ext=bounds.Get()
    rows.append(dict(name=name,asset=name,mesh=path.relative_to(HERE).as_posix(),mesh_units='mm',
        color=part['color'],role=part['role'],volume_mm3=shape.Volume(),valid=True,solid_count=1,
        bbox_mm=[list(ext[:3]),list(ext[3:])],sha256_stl=hashlib.sha256(path.read_bytes()).hexdigest()))
    assembly.add(shape,name=name,color=cq.Color(*part['color']))
assembly.save(str(OUT/'hand_articulation_candidate.step'))
(OUT/'scene.json').write_text(json.dumps(dict(hand=rows,arm_reference=scene['arm_reference']),indent=2))
report.update(part_count=len(rows),all_valid=True,source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (SOURCE/'design.json',SOURCE/'generated/hand_form_candidate.step')})
(OUT/'build_report.json').write_text(json.dumps(report,indent=2))
print('Exported',len(rows),'parts',flush=True)
