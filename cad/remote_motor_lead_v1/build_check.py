"""One fixed remote motor pigtail, provisional cable/plug geometry only.

The existing motor package remains unadopted because of its palm clashes.
This step models three electrical leads from one clear remote module; it
does not alter the hand, approve the connector mating or install hardware.
"""
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import time

import cadquery as cq
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_lead_native',ROOT/'cad/wiring_layout_v1/build_screen.py')
m=importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
p=m.p


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def sweep_lead(bend_x,y,radius,end_y,wire_radius=.95):
    start=(-11.1,y,-8.)
    centre_y=y-radius
    a=(bend_x,y,-8.)
    middle=(bend_x-radius/math.sqrt(2),centre_y+radius/math.sqrt(2),-8.)
    b=(bend_x-radius,centre_y,-8.)
    end=(bend_x-radius,end_y,-8.)
    edges=[cq.Edge.makeLine(cq.Vector(*start),cq.Vector(*a)),
        cq.Edge.makeThreePointArc(cq.Vector(*a),cq.Vector(*middle),cq.Vector(*b)),
        cq.Edge.makeLine(cq.Vector(*b),cq.Vector(*end))]
    path=cq.Wire.assembleEdges(edges)
    plane=cq.Plane(origin=start,xDir=(0,1,0),normal=(-1,0,0))
    shape=cq.Workplane(plane).circle(wire_radius).sweep(cq.Workplane().newObject([path]),isFrenet=True).val()
    expected_length=abs(start[0]-bend_x)+math.pi*radius/2+abs(centre_y-end_y)
    if not shape.isValid() or len(shape.Solids())!=1 or abs(shape.Volume()-math.pi*wire_radius**2*expected_length)>1e-4:
        raise ValueError('Invalid wire sweep/volume')
    return shape,dict(start_mm=list(start),end_mm=list(end),bend_x_mm=bend_x,
        centreline_radius_mm=radius,centreline_length_mm=expected_length,outer_diameter_mm=2*wire_radius)


def test_against(leads,obstacles,v,floor=.001):
    boxes={r['name']:np.asarray(v.bounds(r['shape'])) for r in obstacles}
    overlaps=[]
    errors=[]
    exact=0
    for lead in leads:
        a=np.asarray(v.bounds(lead['shape']))
        for row in obstacles:
            b=boxes[row['name']]
            if np.any(np.minimum(a[1],b[1])-np.maximum(a[0],b[0])<=0):
                continue
            exact+=1
            try:
                volume=float(lead['shape'].intersect(row['shape'],tol=1e-6).Volume())
                if not math.isfinite(volume) or volume < -1e-9:
                    raise ValueError('Invalid common volume')
                if volume>floor:
                    overlaps.append(dict(lead=lead['name'],part=row['name'],material_intersection_mm3=volume))
            except Exception as exc:
                errors.append(dict(lead=lead['name'],part=row['name'],error=repr(exc)))
    return dict(pairs_in_scope=len(leads)*len(obstacles),exact_pairs=exact,overlaps=overlaps,operation_failures=errors)


def main():
    out=HERE/'reviews/check.json'
    if out.exists():
        raise FileExistsError('Keep previous wire candidates')
    started=time.monotonic()
    native,v,cat,scene,hand=p.load()
    layout=p.read(ROOT/'cad/wiring_layout_v1/generated/layout.json')
    motor=next(row for row in layout['modules'] if row['joint']=='index_spread')
    transform=motor['transform_world_mm']
    metadata=p.read(ROOT/'cad/wiring_layout_v1/generated/packaging.json')['parts']
    source={r:sha(ROOT/r) for r in ['cad/remote_motor_lead_v1/build_check.py',
        'cad/wiring_layout_v1/build_screen.py','cad/wiring_layout_v1/generated/layout.json',
        'cad/wiring_layout_v1/generated/packaging.json','cad/wiring_layout_v1/reviews/screen.json',
        'cad/wiring_layout_v1/references.json','cad/wiring_layout_v1/references_current.json',
        'cad/wiring_access_v10/connector_reference.json','cad/functional_motion_v3/audit.py',
        'cad/articulation_drive_v2/verify.py','cad/gesture_hand_v3/audit_gestures.py']}
    obstacles=[]
    for row in metadata:
        source[row['step']]=row['sha256_step']
        if sha(ROOT/row['step'])!=row['sha256_step']:
            raise ValueError('Changed saved motor package')
        obstacles.append(dict(name=row['name'],shape=cq.importers.importStep(str(ROOT/row['step'])).val()))
    for row in scene['hand']:
        source[row['step']]=row['sha256_step']
        source[row['mesh']]=row['sha256_stl']
    initial=p.read(ROOT/'cad/wiring_layout_v1/reviews/screen.json')
    report=dict(completed=False,source_sha256=source,motor_module=motor['name'],transform_world_mm=transform,
        hand_parts=364,motor_packaging_parts=len(obstacles),nominal_wire_OD_mm=1.9,reporting_floor_mm3=.001,
        boolean_tolerance_mm=1e-6,candidates=[],selected=None,hand_pose_checks=[],
        connector_fit_verified=False,wire_sku_verified=False,bend_life_validated=False,board_modelled=False,
        upstream_motor_layout_adopted=False,upstream_palm_interference_pairs=8,
        scope='One fixed forearm-motor electrical pigtail geometric candidate; no whole-hand harness, tendon, motor installation or electrical/physical certification.',
        assumptions=['Historical1.9mm OD bounding model and10/12.5/15mm centreline radii are engineering geometry, not verified supplied21AWG cable dimensions or maker flex-life ratings.',
            'Old EHR3 external housing registration is provisional; hollow/keys/pin insertion and crimp compatibility remain unresolved.',
            'No positive material-overlap exemptions are applied to the wires. Existing palm clashes are preserved separately and not included as newly solved.'])
    chosen=None
    for bend_x in [-15.,-16.5,-18.]:
        leads=[]
        for index,(y,radius) in enumerate(zip([-11.5,-9.,-6.5],[10.,12.5,15.]),1):
            shape,description=sweep_lead(bend_x,y,radius,-36.)
            leads.append(dict(name='remote_lead_'+str(index),shape=m.pack._transform(shape,transform),local=description))
        package=test_against(leads,obstacles,v)
        pair_results=[]
        for i,row in enumerate(leads):
            for other in leads[i+1:]:
                volume=float(row['shape'].intersect(other['shape'],tol=1e-6).Volume())
                pair_results.append(dict(a=row['name'],b=other['name'],material_intersection_mm3=volume,
                    surface_gap_mm=float(row['shape'].distance(other['shape']))))
        clear=not package['overlaps'] and not package['operation_failures'] and all(r['material_intersection_mm3']<=.001 for r in pair_results)
        report['candidates'].append(dict(id='bend_'+str(bend_x),bend_x_mm=bend_x,leads=[dict(name=r['name'],**r['local']) for r in leads],
            packaging=package,wire_pairs=pair_results,wire_package_clear=clear))
        p.write(out,report)
        print(json.dumps(dict(bend_x=bend_x,packaging_overlaps=len(package['overlaps']),wire_pairs=pair_results,clear=clear)),flush=True)
        if clear:
            chosen=leads
            report['selected']=report['candidates'][-1]['id']
            break
    if chosen:
        neutral=v.frames(cat['design'],np.zeros(20))
        for case in initial['proposed']:
            frames=v.frames(cat['design'],case['native']['angles_deg'])
            current=[dict(name=row['name'],shape=v.relocate(hand[row['name']],frames[row['body']],neutral[row['body']])) for row in scene['hand']]
            got=test_against(chosen,current,v)
            report['hand_pose_checks'].append(dict(id=case['id'],angles_deg=case['native']['angles_deg'],native=got))
            p.write(out,report)
        folder=HERE/'generated/parts'
        folder.mkdir(parents=True,exist_ok=True)
        saved=[]
        for row in chosen:
            step=folder/(row['name']+'.step')
            mesh=folder/(row['name']+'.stl')
            cq.exporters.export(row['shape'],str(step))
            cq.exporters.export(row['shape'],str(mesh),tolerance=.05,angularTolerance=.1)
            saved.append(dict(name=row['name'],step=step.relative_to(ROOT).as_posix(),mesh=mesh.relative_to(ROOT).as_posix(),
                sha256_step=sha(step),sha256_stl=sha(mesh),local=row['local'],
                world_start_mm=(np.array(transform)@np.r_[row['local']['start_mm'],1.])[:3].tolist(),
                world_end_mm=(np.array(transform)@np.r_[row['local']['end_mm'],1.])[:3].tolist()))
        # An outer bounding volume is a space reservation, not a hollow plug.
        reference=p.read(ROOT/'cad/wiring_access_v10/connector_reference.json')['connector']
        plug=cq.Solid.makeBox(*reference['registered_outer_size_mm'],cq.Vector(*reference['registered_outer_origin_mm']))
        plug=m.pack._transform(plug,transform)
        plug_check=test_against([dict(name='provisional_plug_outer_envelope',shape=plug)],obstacles,v)
        step=folder/'provisional_plug_outer_envelope.step'
        mesh=folder/'provisional_plug_outer_envelope.stl'
        cq.exporters.export(plug,str(step))
        cq.exporters.export(plug,str(mesh))
        saved.append(dict(name='provisional_plug_outer_envelope',step=step.relative_to(ROOT).as_posix(),mesh=mesh.relative_to(ROOT).as_posix(),
            sha256_step=sha(step),sha256_stl=sha(mesh),role='Outer space reservation only; positive case overlap is not physical mating proof.'))
        report['provisional_plug_check']=plug_check
        p.write(HERE/'generated/scene.json',dict(parts=saved,motor_module=motor['name'],transform_world_mm=transform,
            board_termination='Three open geometric endpoints only; final board and strain relief to be designed.',
            wire_sku_verified=False,connector_fit_verified=False,upstream_layout_adopted=False))
    report.update(completed=True,input_hash_guard=all(sha(ROOT/r)==h for r,h in source.items()),elapsed_s=time.monotonic()-started,
        nominal_leads_clear=bool(chosen) and all(not r['native']['overlaps'] and not r['native']['operation_failures'] for r in report['hand_pose_checks']))
    p.write(out,report)
    if not report['input_hash_guard']:
        raise ValueError('Changed input')


if __name__=='__main__':
    main()
