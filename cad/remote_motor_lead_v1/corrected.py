"""Correct historical joint-to-motor registration before routing a fixed tail.

The first screen omitted the -14.5mm datum translation. Its collisions are
preserved as calculation-error evidence, not evidence against the hand CAD.
"""
import importlib.util
import math
from pathlib import Path
import sys
import time

import cadquery as cq
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_lead_helpers',HERE/'build_check.py')
b=importlib.util.module_from_spec(spec)
spec.loader.exec_module(b)
m,p=b.m,b.p


def lead_shape(stub,y,radius=10.):
    start=(-25.6,y,-8.)
    bend=start[0]-stub
    a=(bend,y,-8.)
    middle=(bend-radius/math.sqrt(2),y,-8.-radius*(1-1/math.sqrt(2)))
    c=(bend-radius,y,-8.-radius)
    end=(bend-radius,y,-36.)
    path=cq.Wire.assembleEdges([cq.Edge.makeLine(cq.Vector(*start),cq.Vector(*a)),
        cq.Edge.makeThreePointArc(cq.Vector(*a),cq.Vector(*middle),cq.Vector(*c)),
        cq.Edge.makeLine(cq.Vector(*c),cq.Vector(*end))])
    plane=cq.Plane(origin=start,xDir=(0,1,0),normal=(-1,0,0))
    shape=cq.Workplane(plane).circle(.95).sweep(cq.Workplane().newObject([path]),isFrenet=True).val()
    length=stub+math.pi*radius/2+18.
    if not shape.isValid() or len(shape.Solids())!=1 or abs(shape.Volume()-math.pi*.95**2*length)>1e-4:
        raise ValueError('Invalid swept wire')
    return shape,dict(start_mm=list(start),end_mm=list(end),straight_stub_mm=stub,
        centreline_radius_mm=radius,centreline_length_mm=length,outer_diameter_mm=1.9,
        arc_start_mm=list(a),arc_middle_mm=list(middle),arc_end_mm=list(c))


def load_obstacles():
    return [dict(name=r['name'],shape=cq.importers.importStep(str(ROOT/r['step'])).val())
        for r in p.read(ROOT/'cad/wiring_layout_v1/generated/packaging.json')['parts']]


def main():
    out=HERE/'reviews/corrected.json'
    if out.exists():
        raise FileExistsError('Keep prior route results')
    started=time.monotonic()
    native,v,cat,scene,hand=p.load()
    initial=p.read(HERE/'reviews/check.json')
    source=dict(initial['source_sha256'])
    for rel in ['cad/remote_motor_lead_v1/reviews/check.json','cad/remote_motor_lead_v1/corrected.py',
        'cad/modular_roots_v9/build_candidate.py','cad/index_joint_v1/joint_manifest.json',
        'cad/wiring_access_v10/review_lead_access.py','cad/wiring_access_v10/make_route_templates.py']:
        source[rel]=b.sha(ROOT/rel)
    # Both source constructions rotate vendor axes identically; only origins differ.
    joint=p.read(ROOT/'cad/index_joint_v1/joint_manifest.json')['vendor_coordinate_transform']
    assert joint['p_mm']==[8,0,0] and joint['R']==[[0,0,1],[0,1,0],[-1,0,0]]
    translation=[-6.5-joint['p_mm'][0],0.,0.]
    assert translation==[-14.5,0.,0.]
    obstacles=load_obstacles()
    transform=initial['transform_world_mm']
    report=dict(completed=False,source_sha256=source,motor_module=initial['motor_module'],transform_world_mm=transform,
        scope='Three fixed electrical tails from one remote motor; fixed hand/forearm reference only. No tendon, complete harness or installed motor claim.',
        datum_correction=dict(historical_joint_to_canonical_motor_translation_mm=translation,
            old_start_x_mm=-11.1,correct_start_x_mm=-25.6,
            previous_screen_interpretation='Incorrect datum registration, not a CAD defect or valid route rejection.'),
        nominal_wire_OD_mm=1.9,reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,
        assumptions=['OD1.9mm,10mm centreline radius and3/5mm straight stub are geometric provisions, not verified supplied cable/bend-life/strain-relief ratings.',
            'Colours identify lanes only, not verified GND/VDD/DATA pin order.',
            'Connector is a conservative outer box; actual hollow housing/keys/pin mating/crimp remain unverified.'],
        candidates=[],selected=None,hand_pose_checks=[],neutral_arm_check=None,
        connector_fit_verified=False,wire_sku_verified=False,bend_life_validated=False,
        strain_relief_complete=False,board_modelled=False,upstream_motor_layout_adopted=False,
        upstream_fixed_palm_interference_pairs=8)
    chosen=None
    for stub in [3.,5.]:
        leads=[]
        for i,y in enumerate([-11.5,-9.,-6.5],1):
            shape,description=lead_shape(stub,y)
            leads.append(dict(name='remote_lead_'+str(i),shape=m.pack._transform(shape,transform),local=description))
        packaging=b.test_against(leads,obstacles,v)
        pairs=[]
        for i,row in enumerate(leads):
            for other in leads[i+1:]:
                pairs.append(dict(a=row['name'],b=other['name'],
                    material_intersection_mm3=float(row['shape'].intersect(other['shape'],tol=1e-6).Volume()),
                    surface_gap_mm=float(row['shape'].distance(other['shape']))))
        clear=not packaging['overlaps'] and not packaging['operation_failures'] and all(r['material_intersection_mm3']<=.001 for r in pairs)
        report['candidates'].append(dict(id='corrected_outboard_stub'+str(stub),stub_mm=stub,
            leads=[dict(name=r['name'],**r['local']) for r in leads],packaging=packaging,wire_pairs=pairs,clear=clear))
        p.write(out,report)
        print('stub',stub,'overlaps',len(packaging['overlaps']),'clear',clear,flush=True)
        if clear:
            chosen=leads
            report['selected']=report['candidates'][-1]['id']
            break
    if chosen:
        neutral=v.frames(cat['design'],np.zeros(20))
        for case in p.read(ROOT/'cad/wiring_layout_v1/reviews/screen.json')['proposed']:
            frames=v.frames(cat['design'],case['native']['angles_deg'])
            current=[dict(name=r['name'],shape=v.relocate(hand[r['name']],frames[r['body']],neutral[r['body']])) for r in scene['hand']]
            report['hand_pose_checks'].append(dict(id=case['id'],angles_deg=case['native']['angles_deg'],native=b.test_against(chosen,current,v)))
        arm=[dict(name=r['name'],shape=cq.importers.importStep(str(ROOT/'cad/modular_roots_v9/parts'/(r['asset']+'.step'))).val())
            for r in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference']]
        report['neutral_arm_check']=b.test_against(chosen,arm,v)
        for r in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference']:
            rel='cad/modular_roots_v9/parts/'+r['asset']+'.step'
            source[rel]=b.sha(ROOT/rel)
        folder=HERE/'generated/parts'
        folder.mkdir(parents=True,exist_ok=False)
        saved=[]
        def save(row,role):
            step=folder/(row['name']+'.step')
            mesh=folder/(row['name']+'.stl')
            cq.exporters.export(row['shape'],str(step))
            cq.exporters.export(row['shape'],str(mesh),tolerance=.05,angularTolerance=.1)
            saved.append(dict(name=row['name'],role=role,step=step.relative_to(ROOT).as_posix(),mesh=mesh.relative_to(ROOT).as_posix(),
                sha256_step=b.sha(step),sha256_stl=b.sha(mesh),local=row.get('local'),bbox_mm=np.asarray(v.bounds(row['shape'])).tolist()))
        for row in chosen:
            save(row,'nominal_electrical_wire_space')
            saved[-1].update(world_start_mm=(np.array(transform)@np.r_[row['local']['start_mm'],1.])[:3].tolist(),
                world_end_mm=(np.array(transform)@np.r_[row['local']['end_mm'],1.])[:3].tolist())
        ref=p.read(ROOT/'cad/wiring_access_v10/connector_reference.json')['connector']
        plug=cq.Solid.makeBox(*ref['registered_outer_size_mm'],cq.Vector(*ref['registered_outer_origin_mm'])).translate(translation)
        plug=m.pack._transform(plug,transform)
        report['provisional_plug_check']=b.test_against([dict(name='provisional_plug_outer_envelope',shape=plug)],obstacles,v)
        save(dict(name='provisional_plug_outer_envelope',shape=plug),'catalog_outer_box_not_physical_mating_approval')
        p.write(HERE/'generated/scene.json',dict(parts=saved,motor_module=report['motor_module'],transform_world_mm=transform,
            board_termination='Three provisional outboard open endpoints. Board and restraint not yet designed.',
            connector_fit_verified=False,upstream_motor_layout_adopted=False))
    clear_checks=[r['native'] for r in report['hand_pose_checks']]
    if report['neutral_arm_check']:
        clear_checks.append(report['neutral_arm_check'])
    report.update(completed=True,input_hash_guard=all(b.sha(ROOT/r)==h for r,h in source.items()),
        elapsed_s=time.monotonic()-started,nominal_leads_clear=bool(chosen) and all(not r['overlaps'] and not r['operation_failures'] for r in clear_checks))
    p.write(out,report)
    if not report['input_hash_guard']:
        raise ValueError('Changed input')


if __name__=='__main__':
    main()
