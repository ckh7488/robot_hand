"""Check actual motor packaging before drawing an invented full-hand harness.

The hand is never cut or rescaled. A separate four-palm/sixteen-remote
packaging candidate moves two obstructing fixed modules off the palm.
Mount strength, transmission and connector fit are deliberately unresolved.
"""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time

import cadquery as cq
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.dont_write_bytecode = True


def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    value=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


p=module('_wiring_hand_native',ROOT/'cad/functional_motion_v3/audit.py')
pack=module('_wiring_packaging',ROOT/'cad/form_drive_v1/actuator_packaging.py')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def moved_layout(old):
    layout=copy.deepcopy(old)
    templates={r['joint']:r for r in layout['modules']}
    for joint,reference in [('index_mcp','index_spread'),('little_mcp','middle_spread')]:
        row=templates[joint]
        template=templates[reference]
        transform=np.array(template['transform_world_mm'],float)
        shift=-316.-template['module_envelope_center_mm'][1]
        transform[1,3]+=shift
        row.update(zone='forearm',rigid_body='forearm',transform_world_mm=transform.tolist(),
            module_envelope_center_mm=[template['module_envelope_center_mm'][0],-316.,-95.],
            relocation_reason='Original fixed palm module obstructs current hand root structures; no joint target angle can move a fixed mounting housing.')
    layout['counts']={'palm':4,'wrist':4,'forearm':12}
    layout['supports']['forearm_rail_y_mm']=[-339.,-1.]
    layout['status']='Provisional4palm/16remote motor packaging for wiring preparation; not installed or mechanically driven.'
    layout['limitations']+=['Two additional rear modules extend the rack42mm towards the elbow. Neutral arm fit must be screened; full arm motion remains unresolved.',
        'No actual motor-to-tendon integration, connector mating, harness, final board, fastening strength or hardware performance approval.']
    return layout


def read_saved_packaging():
    folder=ROOT/'cad/form_drive_v1/generated/packaging'
    metadata=p.read(folder/'placements_mass_inertia.json')['parts']
    cache={}
    rows=[]
    for row in metadata:
        path=ROOT/row['source'] if row['source'] else folder/row['saved_world_step']
        if path not in cache:
            cache[path]=cq.importers.importStep(str(path)).val()
        shape=cache[path]
        if row['source']:
            shape=pack._transform(shape,row['transform_world_mm'])
        elif sha(path)!=row['saved_step_sha256']:
            raise ValueError('Changed saved packaging')
        rows.append(dict(name=row['name'],shape=shape,body=row['body'],module=row['module'],color=row['color']))
    return rows


def screen(hand,packaging,v,scene,parts,design,angles):
    neutral=v.frames(design,np.zeros(20))
    frames=v.frames(design,angles)
    pb={r['name']:np.array(v.bounds(r['shape'])) for r in packaging}
    observations=[]
    failures=[]
    exact=0
    for row in hand:
        shape=v.relocate(parts[row['name']],frames[row['body']],neutral[row['body']])
        box=np.array(v.bounds(shape))
        for item in packaging:
            if np.any(np.minimum(box[1],pb[item['name']][1])-np.maximum(box[0],pb[item['name']][0])<=0):
                continue
            exact+=1
            try:
                volume=float(shape.intersect(item['shape'],tol=1e-6).Volume())
                if volume>.02:
                    observations.append(dict(hand=row['name'],packaging=item['name'],module=item.get('module'),material_intersection_mm3=volume))
            except Exception as exc:
                failures.append(dict(hand=row['name'],packaging=item['name'],error=repr(exc)))
    return dict(angles_deg=list(map(float,angles)),hand_parts=len(hand),packaging_parts=len(packaging),
        pairs_in_scope=len(hand)*len(packaging),exact_pairs=exact,overlaps=observations,operation_failures=failures)


def main():
    output=HERE/'reviews/screen.json'
    if output.exists():
        raise FileExistsError('Preserve original and proposed placement results')
    started=time.monotonic()
    sources={r:sha(ROOT/r) for r in ['cad/wiring_layout_v1/build_screen.py','cad/form_drive_v1/layout.json',
        'cad/form_drive_v1/actuator_packaging.py','cad/form_drive_v1/generated/packaging/placements_mass_inertia.json',
        'cad/form_candidate_v1/generated/scene.json','cad/functional_motion_v3/audit.py',
        'cad/gesture_hand_v3/audit_gestures.py','cad/articulation_drive_v2/verify.py',
        'cad/posture_correction_v1/generated/selected_postures.json','cad/object_pinch_v2/generated/selected_posture.json',
        'cad/combined_pose_v1/generated/selected_posture.json','cad/gesture_hand_v3/poses.json',
        'cad/wiring_layout_v1/references.json']}
    native,v,cat,scene,parts=p.load()
    for row in scene['hand']:
        sources[row['step']]=row['sha256_step']
        sources[row['mesh']]=row['sha256_stl']
    old_geometry=p.read(ROOT/'cad/form_drive_v1/generated/packaging/geometry.json')
    for rel,h in old_geometry['source_sha256'].items():
        if sha(ROOT/rel)!=h:
            raise ValueError('Changed historical packaging source '+rel)
        sources[rel]=h
    before=read_saved_packaging()
    for row in p.read(ROOT/'cad/form_drive_v1/generated/packaging/placements_mass_inertia.json')['parts']:
        if not row['source']:
            rel='cad/form_drive_v1/generated/packaging/'+row['saved_world_step']
            sources[rel]=row['saved_step_sha256']
    report=dict(completed=False,source_sha256=sources,hand_geometry_changed=False,hand_axes_changed=False,
        hand_lengths_changed=False,baseline_hand_parts=364,proposed_motor_count=20,
        baseline=[],proposed=[],arm_screen=None,reporting_floor_mm3=.02,boolean_tolerance_mm=1e-6,
        motor_installed=False,whole_hand_harness_complete=False,connector_fit_verified=False,
        scope='Existing versus proposed motor packaging against current364hand at named endpoints; neutral reference arm only. No hand self-check replay, continuous motion, module internal/mating proof, transmission or electrical approval.')
    old_open=screen(scene['hand'],before,v,scene,parts,cat['design'],np.zeros(20))
    report['baseline'].append(dict(id='open',native=old_open))
    p.write(output,report)
    print(json.dumps(dict(stage='old_open',overlaps=len(old_open['overlaps']),errors=len(old_open['operation_failures']))),flush=True)
    layout=moved_layout(p.read(ROOT/'cad/form_drive_v1/layout.json'))
    p.write(HERE/'generated/layout.json',layout)
    after=pack.build_parts(layout)
    rack=next(r for r in after if r['name']=='Forearm_twenty_axis_module_rack')
    # Close the newly extended rail pair at the additional motor row.
    rack['shape']=rack['shape'].fuse(cq.Workplane('XY').box(52.,4.,3.).translate((-29.4,-337.,-74.5)).val()).clean()
    if not rack['shape'].isValid() or len(rack['shape'].Solids())!=1:
        raise ValueError('Extended rack must remain one valid solid')
    saved=[]
    for row in after:
        folder=HERE/'generated/parts'
        folder.mkdir(parents=True,exist_ok=True)
        step=folder/(row['name']+'.step')
        mesh=folder/(row['name']+'.stl')
        cq.exporters.export(row['shape'],str(step))
        cq.exporters.export(row['shape'],str(mesh),tolerance=.2,angularTolerance=.2)
        saved.append(dict(name=row['name'],body=row['body'],module=row.get('module'),color=list(row['color']),
            step=step.relative_to(ROOT).as_posix(),mesh=mesh.relative_to(ROOT).as_posix(),
            sha256_step=sha(step),sha256_stl=sha(mesh),bbox_mm=np.asarray(v.bounds(row['shape'])).tolist()))
    p.write(HERE/'generated/packaging.json',dict(parts=saved,hand_reference='cad/motion_clearance_v2/generated/assembly_catalog.json',
        motors=20,palm=4,remote=16,installed=False,hand_geometry_changed=False))
    authored=np.array(p.read(ROOT/'cad/gesture_hand_v3/poses.json')['reference']['authored_fist_deg'])
    thumbup=authored.copy()
    thumbup[16:]=[0,0,0,-15]
    crossing=p.read(ROOT/'cad/posture_correction_v1/generated/selected_postures.json')['postures'][0]['angles_deg']
    goals=[('open',np.zeros(20)),('thumb_up',thumbup),('cross',crossing),
        ('pinch25',p.read(ROOT/'cad/object_pinch_v2/generated/selected_posture.json')['angles_deg']),
        ('combined',p.read(ROOT/'cad/combined_pose_v1/generated/selected_posture.json')['angles_deg'])]
    for label,q in goals:
        got=screen(scene['hand'],after,v,scene,parts,cat['design'],q)
        report['proposed'].append(dict(id=label,native=got))
        p.write(output,report)
        print(json.dumps(dict(stage='proposed_'+label,overlaps=len(got['overlaps']),errors=len(got['operation_failures']))),flush=True)
    arm_rows=[]
    arm_parts={}
    for row in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference']:
        rel='cad/modular_roots_v9/parts/'+row['asset']+'.step'
        sources[rel]=sha(ROOT/rel)
        arm_parts[row['name']]=cq.importers.importStep(str(ROOT/rel)).val()
        arm_rows.append(dict(name=row['name'],body='palm'))
    report['arm_screen']=screen(arm_rows,after,v,scene,arm_parts,cat['design'],np.zeros(20))
    report.update(completed=True,input_hash_guard=all(sha(ROOT/r)==h for r,h in sources.items()),elapsed_s=time.monotonic()-started,
        proposed_hand_endpoints_clear=all(not row['native']['overlaps'] and not row['native']['operation_failures'] for row in report['proposed']),
        neutral_reference_arm_clear=not report['arm_screen']['overlaps'] and not report['arm_screen']['operation_failures'])
    p.write(output,report)
    if not report['input_hash_guard']:
        raise ValueError('Inputs changed')


if __name__=='__main__':
    main()
