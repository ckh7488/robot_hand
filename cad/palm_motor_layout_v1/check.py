"""Rigid XY/yaw packing on the actual palm, without cutting the hand."""
import argparse
import copy
import importlib.util
import math
from pathlib import Path
import sys
import time
import types

import cadquery as cq
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.dont_write_bytecode = True


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


c = module('_palm_prior_assembly', ROOT/'cad/assembly_sequence_v1/check.py')
b, m, p = c.b, c.m, c.p


def matrix(change):
    angle = math.radians(change['yaw_deg'])
    co, si = math.cos(angle), math.sin(angle)
    rotation = np.array([[co,-si,0],[si,co,0],[0,0,1.]])
    pivot = np.asarray(change['pivot_mm'], float)
    result = np.eye(4)
    result[:3,:3] = rotation
    result[:3,3] = pivot + change['translation_mm'] - rotation@pivot
    return result


def transform(row, changes):
    change = changes.get(row.get('module'))
    if change is None:
        return row
    return dict(row, shape=m.pack._transform(row['shape'], matrix(change)))


def material(moving, fixed):
    return b.test_against(moving, fixed, types.SimpleNamespace(bounds=m.pack._bbox), floor=.001)


def load(rows, source=None):
    result = []
    for row in rows:
        for key, hkey in [('step','sha256_step'), ('mesh','sha256_stl')]:
            if key not in row:
                continue
            if b.sha(ROOT/row[key]) != row[hkey]:
                raise ValueError('Changed input '+row[key])
            if source is not None:
                source[row[key]] = row[hkey]
        result.append(dict(row, shape=cq.importers.importStep(str(ROOT/row['step'])).val()))
    return result


def floor_contacts(rows, palm):
    result = []
    for row in rows:
        if not row['name'].endswith('_adapter'):
            continue
        footfaces = [f for f in row['shape'].Faces()
                     if abs(m.pack._bbox(f)[0][2]+40.5)<1e-5 and abs(m.pack._bbox(f)[1][2]+40.5)<1e-5]
        if len(footfaces) != 1:
            raise ValueError('Expected one flat palm foot '+row['name'])
        face = footfaces[0]
        box = np.asarray(m.pack._bbox(face))
        samples = []
        for u,v in [(.05,.05),(.05,.95),(.5,.5),(.95,.05),(.95,.95)]:
            point = box[0]*(1-np.array([u,v,.5]))+box[1]*np.array([u,v,.5])
            below = bool(palm.isInside(cq.Vector(*list(point+[0,0,-.05])), 1e-6))
            above = bool(palm.isInside(cq.Vector(*list(point+[0,0,.05])), 1e-6))
            samples.append(dict(point_mm=point.tolist(), below_in_palm=below, above_in_palm=above))
        result.append(dict(module=row['module'], foot_area_mm2=face.Area(),
                           distance_mm=face.distance(palm), samples=samples,
                           seated=all(r['below_in_palm'] and not r['above_in_palm'] for r in samples)))
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--plan', type=Path, default=HERE/'placement.json')
    ap.add_argument('--output', type=Path, default=HERE/'reviews/check.json')
    args = ap.parse_args()
    if args.output.exists():
        raise FileExistsError('Preserve prior packing study')
    started = time.monotonic()
    plan = p.read(args.plan)
    source = {r: b.sha(ROOT/r) for r in [
        'cad/palm_motor_layout_v1/check.py', args.plan.relative_to(ROOT).as_posix(),
        'cad/wiring_layout_v1/generated/layout.json', 'cad/wiring_layout_v1/generated/packaging.json',
        'cad/wire_guides_v1/generated/scene.json', 'cad/remote_motor_lead_v2/generated/scene.json',
        'cad/wiring_layout_v1/reviews/screen.json', 'cad/assembly_sequence_v1/check.py',
        'cad/remote_motor_lead_v1/build_check.py', 'cad/form_drive_v1/actuator_packaging.py',
        'cad/functional_motion_v3/audit.py', 'cad/functional_motion_v3/generated/catalog.json']}
    native,v,cat,scene,hand = p.load()
    for row in scene['hand']:
        source[row['step']] = row['sha256_step']
        source[row['mesh']] = row['sha256_stl']
    saved = p.read(ROOT/'cad/wire_guides_v1/generated/scene.json')
    original = load(saved['retained_packaging']+saved['new_parts'], source)
    assert len(original) == 353
    changed_names = set(plan['changes'])
    old = [r for r in original if r.get('module') in changed_names]
    unchanged = [r for r in original if r.get('module') not in changed_names]
    changed = [transform(r, plan['changes']) for r in old]
    assert len(changed) == 24 and len(unchanged) == 329
    q0 = np.zeros(20)
    neutral = v.frames(cat['design'], q0)
    def hand_at(q):
        frames = v.frames(cat['design'], q)
        return [dict(name=r['name'], shape=v.relocate(hand[r['name']],frames[r['body']],neutral[r['body']])) for r in scene['hand']]
    report = dict(completed=False, source_sha256=source, plan=plan,
                  hand_geometry_changed=False, axes_changed=False, lengths_changed=False,
                  part_geometry_changed=False, moved_parts=24, unchanged_packaging_guide_parts=329,
                  installed=False, reporting_floor_mm3=.001, boolean_tolerance_mm=1e-6,
                  whole_hand_harness_complete=False, chassis_fasteners_modelled=False,
                  scope='24rigidly moved palm module solids against all364hand at5endpoints, other329packaging/guide solids, different moved modules,48fixed remote leads/16plug envelopes/10neutral arm. In-module contacts remain rigidly identical. No hand self-collision replay, continuous motion, physical fastening, coupling, strength or full harness approval.', cases=[])
    report['before_open'] = material(old, hand_at(q0))
    p.write(args.output, report)
    print('before',len(report['before_open']['overlaps']),flush=True)
    for goal in p.read(ROOT/'cad/wiring_layout_v1/reviews/screen.json')['proposed']:
        result = material(changed, hand_at(goal['native']['angles_deg']))
        report['cases'].append(dict(id=goal['id'], angles_deg=goal['native']['angles_deg'], native=result))
        p.write(args.output, report)
        print(goal['id'],len(result['overlaps']),len(result['operation_failures']),flush=True)
    report['other_packaging'] = material(changed, unchanged)
    cross = []
    for i, a in enumerate(sorted(changed_names)):
        for z in sorted(changed_names)[i+1:]:
            cross.append(dict(modules=[a,z],native=material([r for r in changed if r['module']==a],[r for r in changed if r['module']==z])))
    report['between_moved_modules'] = cross
    wiremeta = p.read(ROOT/'cad/remote_motor_lead_v2/generated/scene.json')
    wires = load(wiremeta['parts'], source)
    assert len(wires)==64
    report['fixed_remote_wires_and_plug_envelopes'] = material(changed, wires)
    arm = []
    for r in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference']:
        rel='cad/modular_roots_v9/parts/'+r['asset']+'.step'
        source[rel]=b.sha(ROOT/rel)
        arm.append(dict(name=r['name'],shape=cq.importers.importStep(str(ROOT/rel)).val()))
    report['neutral_arm'] = material(changed,arm)
    report['floor_contacts'] = floor_contacts(changed,hand['Palm_tapered_chassis'])
    tests = [r['native'] for r in report['cases']] + [report['other_packaging'],report['neutral_arm'],report['fixed_remote_wires_and_plug_envelopes']] + [r['native'] for r in cross]
    report['nominal_placement_clear'] = all(not r['overlaps'] and not r['operation_failures'] for r in tests)
    report['changed_feet_seated'] = all(r['seated'] and r['distance_mm']<1e-5 for r in report['floor_contacts'])
    rows = []
    for row in changed:
        folder=HERE/'generated/parts';folder.mkdir(parents=True,exist_ok=True)
        step=folder/(row['name']+'.step');mesh=folder/(row['name']+'.stl')
        cq.exporters.export(row['shape'],str(step));cq.exporters.export(row['shape'],str(mesh),tolerance=.2,angularTolerance=.2)
        rows.append(dict(name=row['name'],body=row['body'],module=row['module'],color=row['color'],
                         step=step.relative_to(ROOT).as_posix(),mesh=mesh.relative_to(ROOT).as_posix(),
                         sha256_step=b.sha(step),sha256_stl=b.sha(mesh),bbox_mm=m.pack._bbox(row['shape'])))
    layout=copy.deepcopy(p.read(ROOT/'cad/wiring_layout_v1/generated/layout.json'))
    for row in layout['modules']:
        if row['name'] in changed_names:
            T=matrix(plan['changes'][row['name']])
            row['transform_world_mm']=(T@np.asarray(row['transform_world_mm'])).tolist()
            row['module_envelope_center_mm']=(T@np.r_[row['module_envelope_center_mm'],1.])[:3].tolist()
    p.write(HERE/'generated/layout.json',layout)
    p.write(HERE/'generated/scene.json',dict(changed_parts=rows,retained_packaging_guide=[{k:z for k,z in r.items() if k!='shape'} for r in unchanged],
        wires_reference='cad/remote_motor_lead_v2/generated/scene.json',hand_reference='cad/motion_clearance_v2/generated/assembly_catalog.json',
        motor_count=20,palm=4,remote=16,installed=False,plan=plan))
    report.update(completed=True,elapsed_s=time.monotonic()-started,input_hash_guard=all(b.sha(ROOT/r)==h for r,h in source.items()))
    p.write(args.output,report)
    print('complete',report['nominal_placement_clear'],report['changed_feet_seated'],report['elapsed_s'],flush=True)
    if not report['input_hash_guard']:
        raise ValueError('Changed inputs')


if __name__=='__main__':
    main()
