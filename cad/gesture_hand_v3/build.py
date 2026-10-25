"""Native BRep reconstruction of FORM002, preserving its visible proportions.

Only skins inherit the authored affine shape. Round pins, bushings and fasteners
are replaced by the explicit fabrication repair stage. Source CAD stays frozen.
"""
from pathlib import Path
import argparse, hashlib, importlib.util, json
import cadquery as cq
import numpy as np
from OCP.gp import gp_Trsf
from poses import forward_kinematics

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
SOURCE=HERE/'source_native';FORM=ROOT/'cad/accepted_form_v1'

def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def mesh_vertices(path):
    raw=Path(path).read_bytes();n=int.from_bytes(raw[80:84],'little')
    dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attr','<u2')])
    if len(raw)!=84+50*n:raise ValueError('Expected binary STL: '+str(path))
    return np.frombuffer(raw,dtype=dtype,offset=84,count=n)['vertices'].reshape(-1,3).astype(float)

def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m

def transformed(shape,A):
    if np.max(np.abs(A[:3,:3].T@A[:3,:3]-np.eye(3)))<1e-12:
        t=gp_Trsf();t.SetValues(*[float(A[i,j]) for i in range(3) for j in range(4)])
        return shape.moved(cq.Location(t))
    return shape.transformGeometry(cq.Matrix(A.tolist()))

def load_source(design):
    old=json.loads((SOURCE/'design.json').read_text())
    scene=json.loads((SOURCE/'generated/scene.json').read_text())
    accepted_parts=json.loads((FORM/'parts.json').read_text())
    bodies={r['part']:('palm' if r['frame']=='world' else r['frame']) for r in accepted_parts.values()}
    accepted_mesh={r['part']:FORM/path for path,r in accepted_parts.items()}
    native=cq.importers.importStep(str(SOURCE/'generated/hand_articulation_candidate.step')).val().Solids()
    if len(native)!=len(scene['hand']):raise ValueError('Native STEP/scene count mismatch')
    old_fk=forward_kinematics(old,[0]*20)['bodies'];new_fk=forward_kinematics(design,[0]*20)['bodies']
    helper=module('gesture_mass_helper',ROOT/'cad/form_drive_v1/exterior.py')
    parts=[];changes=[]
    for row,solid in zip(scene['hand'],native):
        name=row['name'];body=bodies[name]
        props=helper.mass_properties(solid,1.25e-6)
        if abs(props['volume_mm3']-row['volume_mm3'])>max(.05,1e-5*row['volume_mm3']) or np.max(np.abs(np.array(props['bbox_mm'])-row['bbox_mm']))>.002:
            raise ValueError('Native STEP mapping mismatch: '+name)
        S=np.eye(4)
        if body=='palm':
            if name.endswith('_spread_housing'):
                digit=name.split('_')[0];a=old['thumb'] if digit=='thumb' else next(f for f in old['fingers'] if f['name']==digit)
                b=design['thumb'] if digit=='thumb' else next(f for f in design['fingers'] if f['name']==digit)
                A=np.eye(4);A[:3,3]=np.array(b['root'])-a['root']
            else:
                S[1,1]=design['palm_target_envelope_mm'][1]/old['palm_target_envelope_mm'][1];A=S
        else:
            if body=='thumb_opposition':S[1,1]=design['thumb']['links'][0]/old['thumb']['links'][0]
            if body=='thumb_mcp':S[1,1]=design['thumb']['links'][1]/old['thumb']['links'][1]
            A=new_fk[body]@S@np.linalg.inv(old_fk[body])
        shape=transformed(solid,A)
        original_v=mesh_vertices(SOURCE/row['mesh'])
        expected_v=mesh_vertices(accepted_mesh[name])
        if original_v.shape!=expected_v.shape:raise ValueError('Accepted/native mesh correspondence mismatch: '+name)
        native_world=original_v@A[:3,:3].T+A[:3,3]
        T=new_fk[body];accepted_world=expected_v@T[:3,:3].T+T[:3,3]
        error=float(np.max(np.linalg.norm(native_world-accepted_world,axis=1)))
        if error>.001:raise ValueError('Reconstruction differs from accepted visible form: '+name+' '+str(error))
        rho,material=helper.material_density(row)
        parts.append(dict(name=name,shape=shape,color=row['color'],role=row['role'],body=body,
            density_kg_per_mm3=rho,material=material,source='FORM002_native_reconstruction'))
        changes.append(dict(name=name,body=body,source_to_world_matrix=A.tolist(),
            accepted_vertex_correspondence_max_error_mm=error,
            affine_hardware_not_fabrication_approved=bool(abs(S[1,1]-1)>1e-12)))
    return parts,changes,helper

def mount_roots(parts,design,report):
    import root_module as root
    out=[];changes=[]
    webs=[p for p in parts if p['name'].startswith('Palm_side_web_')]
    for p in parts:
        if p in webs:continue
        s=p['shape'];v=s.Volume()
        if p['name']=='Palm_tapered_chassis':
            for web in webs:s=s.fuse(web['shape'])
            s,pocket_report=root.cut_palm_root_interfaces(s,design)
            changes.append(dict(name=p['name'],root_clearance_and_floor_holes=pocket_report))
        elif p['name']=='thumb_segment1_thenar_shell':
            f=dict(design['thumb'],name='thumb');h=f['flexion_offset'][2]
            s=s.cut(root._world(root._cyl(cq,3.4,16.02,(-8.01,0,h),(1,0,0)),f))
        if abs(s.Volume()-v)>1e-7:
            s=s.clean();changes.append(dict(name=p['name'],before_mm3=v,after_mm3=s.Volume(),
                integrated_same_material_parts=[w['name'] for w in webs] if p['name']=='Palm_tapered_chassis' else []))
        out.append(dict(p,shape=s))
    return out,changes

def main():
    p=argparse.ArgumentParser();p.add_argument('--stage',choices=['baseline','roots','fabrication'],default='fabrication')
    args=p.parse_args()
    design=json.loads((FORM/'design.json').read_text())
    manifest=json.loads((FORM/'manifest.json').read_text())
    for name,sha in manifest.items():
        if digest(FORM/name)!=sha:raise ValueError('Accepted form changed: '+name)
    source_manifest=json.loads((SOURCE/'manifest.json').read_text(encoding='utf-8-sig'))
    for name,sha in source_manifest.items():
        if digest(SOURCE/name)!=sha:raise ValueError('Native source snapshot changed: '+name)
    paths=[FORM/'manifest.json',*[FORM/n for n in manifest],SOURCE/'design.json',SOURCE/'generated/scene.json',
        SOURCE/'generated/hand_articulation_candidate.step',SOURCE/'manifest.json',HERE/'build.py',HERE/'poses.py',HERE/'poses.json',ROOT/'cad/form_drive_v1/exterior.py']
    paths.extend(SOURCE/r['mesh'] for r in json.loads((SOURCE/'generated/scene.json').read_text())['hand'])
    # Bind the bounded motion study to the same canonical pose definition.
    paths.extend([HERE/'build_core_paths.py',HERE/'core_paths.json'])
    if args.stage!='baseline':paths.append(HERE/'root_module.py')
    if args.stage=='fabrication':paths.append(HERE/'fabrication_repairs.py')
    hashes=lambda:{x.relative_to(ROOT).as_posix():digest(x) for x in paths}
    before=hashes();parts,source_mapping,helper=load_source(design)
    root_report={};mount_changes=[];fabrication_report={};removed=[];added=[]
    if args.stage!='baseline':
        from root_module import build_root_modules
        replacement,removed,root_report=build_root_modules(design)
        parts=[p for p in parts if p['name'] not in removed]+replacement
        added=[p['name'] for p in replacement]
        parts,mount_changes=mount_roots(parts,design,root_report)
    if args.stage=='fabrication':
        from fabrication_repairs import repair
        parts,fabrication_report=repair(parts,design)
    out=HERE/'generated';out.mkdir(exist_ok=True)
    for name in ['parts','meshes']:(out/name).mkdir(exist_ok=True)
    catalog=json.loads((HERE/'poses.json').read_text(encoding='utf-8'))
    targets={g['id']:g['angles_deg'] for g in catalog['gestures']}
    design.update(revision='gesture_hand_v3',source_candidate='cad/accepted_form_v1',build_stage=args.stage,
        status='FORM002 preserved form rebuilt for gesture and fabrication checks; no manufacturing approval',
        gesture_candidate_pose_catalog=dict(path='poses.json',sha256=digest(HERE/'poses.json'),
            fist_deg=targets['fist'],thumb_up_deg=targets['thumb_up'],
            reference_fields_notice='Top-level fist_deg/fist_path remain the original accepted FORM002 reference. Current target gestures and candidate paths are defined separately in poses.json.',
            transition_collision_verified=False))
    (HERE/'design.json').write_text(json.dumps(design,indent=2)+'\n')
    rows=[];masses=[];names=set();assembly=cq.Assembly(name='gesture_hand_v3')
    for i,part in enumerate(parts):
        name=part['name'];s=part['shape']
        if name in names or not s.isValid() or len(s.Solids())!=1:raise ValueError('Duplicate/invalid/disconnected part: '+name)
        names.add(name)
        step=out/'parts'/(name+'.step');mesh=out/'meshes'/(name+'.stl')
        cq.exporters.export(s,str(step));cq.exporters.export(s,str(mesh),tolerance=.1,angularTolerance=.1)
        props=helper.mass_properties(s,part['density_kg_per_mm3'])
        rows.append(dict(name=name,step=step.relative_to(HERE).as_posix(),step_index=i,sha256_step=digest(step),
            mesh=mesh.relative_to(HERE).as_posix(),mesh_units='mm',sha256_stl=digest(mesh),body=part['body'],
            role=part['role'],color=part['color'],volume_mm3=props['volume_mm3'],bbox_mm=props['bbox_mm'],
            valid=True,solid_count=1,source=part.get('source','gesture_fabrication_repair')))
        masses.append(dict(name=name,body=part['body'],role=part['role'],material=part['material'],**props))
        assembly.add(s,name=name,color=cq.Color(*part['color'][:3]))
    assembly.save(str(out/'hand.step'))
    if hashes()!=before:raise RuntimeError('Build input changed')
    (out/'scene.json').write_text(json.dumps(dict(hand=rows,arm_reference=[],source_sha256=before,inputs_unchanged=True),indent=2)+'\n')
    (out/'mass_inertia.json').write_text(json.dumps(dict(parts=masses,source_sha256=before,
        total_mass_kg=sum(r['mass_kg'] for r in masses),moving_mass_kg=sum(r['mass_kg'] for r in masses if r['body']!='palm'),
        limitations=['Assumed densities, no motors/tendons/wiring; not complete hardware mass.']),indent=2)+'\n')
    (out/'build_report.json').write_text(json.dumps(dict(stage=args.stage,source_sha256=before,inputs_unchanged=True,
        part_count=len(rows),source_step_mapping=source_mapping,root_module=root_report,palm_mounts=mount_changes,
        fabrication_repairs=fabrication_report,removed_parts=removed,added_parts=added,all_parts_valid_single_solid=True,
        manufacture_approved=False,motor_performance_validated=False),indent=2)+'\n')
    print(json.dumps(dict(stage=args.stage,parts=len(rows),inputs_unchanged=True)),flush=True)

if __name__=='__main__':main()
