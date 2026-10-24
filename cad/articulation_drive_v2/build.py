"""Integrate root repairs with the saved, articulated hand. All lengths in mm.

The webcam candidate is a read-only source. Every output part gets its own STEP,
mesh and explicit rigid-body assignment; the integration never invokes its
side-effecting CAD builder or changes the active webcam application.
"""
from pathlib import Path
import hashlib
import importlib.util
import json
import re
import sys

import cadquery as cq
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SOURCE = ROOT / 'cad/webcam_articulation_v1'
OUT = HERE / 'generated'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def hashes():
    files = [SOURCE/p for p in ('design.json','build.py','digits.py','thumb_shell.py',
        'generated/scene.json','generated/hand_articulation_candidate.step')]
    files.extend([HERE/'build.py', HERE/'root_module.py',
                  ROOT/'cad/form_drive_v1/exterior.py'])
    return {p.relative_to(ROOT).as_posix(): digest(p) for p in files}


def load_mass_helper():
    path = ROOT/'cad/form_drive_v1/exterior.py'
    spec = importlib.util.spec_from_file_location('cad_exact_mass_helper', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def source_body(name):
    match = re.match(r'^(index|middle|ring|little|thumb)_(.*)$',name)
    if not match:
        return 'palm'
    finger, rest = match.groups()
    if rest == 'spread_housing':
        return 'palm'
    if rest.startswith(('spread_ring','root_fork')):
        index = 0
    elif rest.startswith('joint_pin_'):
        index = int(rest[-1])
    else:
        index = int(re.match(r'segment([123])_',rest)[1])
    labels = ['base_sweep','opposition','mcp','ip'] if finger=='thumb' else ['spread','mcp','pip','dip']
    return finger+'_'+labels[index]


def load_source():
    helper = load_mass_helper()
    design = json.loads((SOURCE/'design.json').read_text(encoding='utf-8-sig'))
    scene = json.loads((SOURCE/'generated/scene.json').read_text())
    solids = cq.importers.importStep(str(SOURCE/'generated/hand_articulation_candidate.step')).val().Solids()
    if len(solids)!=len(scene['hand']):
        raise ValueError('Source STEP/scene count mismatch')
    parts = {}
    mapping = []
    for row, solid in zip(scene['hand'],solids):
        mass = helper.mass_properties(solid,1.25e-6)
        dv = abs(solid.Volume()-row['volume_mm3'])
        db = float(np.max(np.abs(np.array(mass['bbox_mm'])-row['bbox_mm'])))
        if dv>max(.05,row['volume_mm3']*1e-5) or db>.002:
            raise ValueError('Source STEP/scene mapping changed: '+row['name'])
        rho, material = helper.material_density(row)
        parts[row['name']] = dict(name=row['name'],shape=solid,color=row['color'],
            role=row['role'],body=source_body(row['name']),density_kg_per_mm3=rho,
            material=material,source='webcam_articulation_v1_saved_STEP')
        mapping.append(dict(name=row['name'],volume_error_mm3=dv,bbox_error_mm=db))
    return design,scene,parts,mapping


def main():
    from root_module import build_root_modules, apply_palm_mounts
    before=hashes()
    design,source_scene,parts,mapping = load_source()
    # Explicit replacement list preserves every unmodified exterior component.
    replacements,remove_names,root_report = build_root_modules(design)
    missing=set(remove_names)-set(parts)
    if missing:raise ValueError('Unknown replacement inputs: '+repr(missing))
    for name in remove_names:del parts[name]
    for part in replacements:
        if part['name'] in parts:raise ValueError('Duplicate part: '+part['name'])
        if 'body' not in part or 'density_kg_per_mm3' not in part:
            raise ValueError('Missing explicit body/material: '+part['name'])
        parts[part['name']]=part
    modified, mount_report = apply_palm_mounts(list(parts.values()), design)
    parts = {p['name']:p for p in modified}
    design['revision']='articulation_drive_v2'
    design['status']='Root assembly repair candidate; remaining fabrication and drive blockers are reported'
    design['source_candidate']='cad/webcam_articulation_v1'
    design['actuator_packaging']['installed_actuators_in_this_cad']=0
    design['performance']['new_geometry_tested']=False
    (HERE/'design.json').write_text(json.dumps(design,indent=2)+'\n')
    for sub in ('meshes','parts'): (OUT/sub).mkdir(parents=True,exist_ok=True)
    mass_helper=load_mass_helper();rows=[];mass_rows=[]
    assembly=cq.Assembly(name='articulation_drive_v2')
    for index,part in enumerate(parts.values()):
        name=part['name'];shape=part['shape']
        if not shape.isValid() or len(shape.Solids())!=1:
            raise ValueError(f'Invalid/disconnected part: {name}')
        step=OUT/'parts'/(name+'.step');mesh=OUT/'meshes'/(name+'.stl')
        cq.exporters.export(shape,str(step))
        cq.exporters.export(shape,str(mesh),tolerance=.1,angularTolerance=.1)
        props=mass_helper.mass_properties(shape,part['density_kg_per_mm3'])
        row=dict(name=name,step=step.relative_to(HERE).as_posix(),step_index=index,
            sha256_step=digest(step),mesh=mesh.relative_to(HERE).as_posix(),mesh_units='mm',
            sha256_stl=digest(mesh),body=part['body'],color=part['color'],role=part['role'],
            volume_mm3=props['volume_mm3'],bbox_mm=props['bbox_mm'],valid=True,solid_count=1,
            source=part.get('source','root_repair_candidate'))
        rows.append(row)
        mass_rows.append(dict(name=name,body=part['body'],role=part['role'],
            material=part.get('material','unvalidated_density_assumption'),**props))
        assembly.add(shape,name=name,color=cq.Color(*part['color'][:3]))
    assembly.save(str(OUT/'hand.step'))
    # Keep reference arm paths portable from the new candidate's directory.
    import os
    arm=[]
    for item in source_scene.get('arm_reference',[]):
        p=dict(item);p['mesh']=Path(os.path.relpath((SOURCE/item['mesh']).resolve(),HERE)).as_posix();arm.append(p)
    after=hashes()
    if before!=after:raise RuntimeError('Inputs changed during build')
    (OUT/'scene.json').write_text(json.dumps(dict(hand=rows,arm_reference=arm,
        source_sha256=before,inputs_unchanged=True),indent=2)+'\n')
    (OUT/'mass_inertia.json').write_text(json.dumps(dict(parts=mass_rows,
        units='mm; kg; kg m2',source_sha256={**before,'cad/articulation_drive_v2/generated/hand.step':digest(OUT/'hand.step')},
        total_mass_kg=sum(p['mass_kg'] for p in mass_rows),
        moving_mass_kg=sum(p['mass_kg'] for p in mass_rows if p['body']!='palm'),
        limitations=['CAD volume and assumed densities; not measured weights.',
            'Unresolved overlaps are counted separately; this is not a fabricated assembly mass.',
            'Motor drive transmission, tendons, wiring and missing fasteners are excluded.']),indent=2)+'\n')
    report=dict(source_sha256=before,inputs_unchanged=True,part_count=len(rows),
        removed_parts=sorted(remove_names),added_parts=[p['name'] for p in replacements],
        source_step_mapping=mapping,root_module=root_report,palm_mounts=mount_report,all_parts_valid_single_solid=True,
        manufacture_approved=False,motor_performance_validated=False)
    (OUT/'build_report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(parts=len(rows),root_report=root_report)),flush=True)


if __name__=='__main__':main()
