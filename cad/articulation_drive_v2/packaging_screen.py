"""Recheck saved DRIVE001 motor placement against new hand; no installation claim."""
from pathlib import Path
import hashlib
import importlib.util
import json
import sys
import numpy as np
import cadquery as cq
from OCP.gp import gp_Trsf

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
PACK=ROOT/'cad/form_drive_v1/generated/packaging'


def main():
    sys.path.insert(0,str(ROOT/'cad/webcam_articulation_v1'))
    spec=importlib.util.spec_from_file_location('packaging_pose_frames',ROOT/'cad/webcam_articulation_v1/verify.py')
    pose=importlib.util.module_from_spec(spec);spec.loader.exec_module(pose)
    from render import body_key
    paths={HERE/'design.json',HERE/'generated/scene.json',PACK/'placements_mass_inertia.json',Path(__file__),
        HERE/'render.py',ROOT/'cad/webcam_articulation_v1/verify.py',ROOT/'cad/webcam_articulation_v1/thumb_shell.py'}
    design=json.loads((HERE/'design.json').read_text());rows=json.loads((HERE/'generated/scene.json').read_text())['hand']
    native_rows=json.loads((PACK/'placements_mass_inertia.json').read_text())['parts']
    for r in rows:paths.add(HERE/r['step'])
    for r in native_rows:paths.add(ROOT/r['source'] if r['source'] else PACK/r['saved_world_step'])
    hashes=lambda:{p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(paths)}
    before=hashes();parts=[];cache={}
    for row in rows:
        if before[(HERE/row['step']).relative_to(ROOT).as_posix()]!=row['sha256_step']:
            raise ValueError('Scene/hand STEP mismatch: '+row['name'])
    for row in native_rows:
        path=ROOT/row['source'] if row['source'] else PACK/row['saved_world_step']
        if not row['source'] and before[path.relative_to(ROOT).as_posix()]!=row['saved_step_sha256']:
            raise ValueError('Saved packaging STEP changed: '+row['name'])
        if path not in cache:cache[path]=cq.importers.importStep(str(path)).val()
        shape=cache[path]
        if row['source']:
            matrix=row['transform_world_mm'];tf=gp_Trsf();tf.SetValues(*[float(matrix[i][j]) for i in range(3) for j in range(4)])
            shape=shape.moved(cq.Location(tf))
        parts.append(dict(name=row['name'],shape=shape,box=pose.bounds(shape)))
    hand={r['name']:cq.importers.importStep(str(HERE/r['step'])).val() for r in rows}
    neutral=pose.frames(design,np.zeros(20));cases=[]
    for label,angles in [('open',np.zeros(20)),('fist',np.array(design['fist_deg']))]:
        frame=pose.frames(design,angles);clashes=[];errors=[];pairs=0
        for row in rows:
            key=body_key(row['body']);shape=pose.move(hand[row['name']],frame[key],neutral[key]);box=pose.bounds(shape)
            for item in parts:
                if np.any(np.minimum(box[1],item['box'][1])-np.maximum(box[0],item['box'][0])<=1e-6):continue
                pairs+=1
                try:
                    v=shape.intersect(item['shape'],tol=1e-6).Volume()
                    if v>.02:clashes.append(dict(hand=row['name'],packaging=item['name'],volume_mm3=v))
                except Exception as exc:errors.append(dict(hand=row['name'],packaging=item['name'],error=repr(exc)))
        case=dict(pose=label,angles_deg=angles.tolist(),exact_pairs=pairs,overlaps=clashes,boolean_errors=errors)
        cases.append(case);print(label,'pairs',pairs,'overlaps',len(clashes),'errors',len(errors),flush=True)
    if hashes()!=before:raise RuntimeError('Input changed during packaging screen')
    motor_modules=sorted({r['module'] for r in native_rows if r['kind']=='motor'})
    report=dict(input_sha256=before,inputs_unchanged=True,checked_motor_count=len(motor_modules),
        checked_motor_modules=motor_modules,motor_component_count=sum(r['kind']=='motor' for r in native_rows),
        saved_packaging_components=len(parts),cases=cases,scope='Existing DRIVE001 static packaging versus new hand only, open and fist poses.',
        passed=all(not c['overlaps'] and not c['boolean_errors'] for c in cases),
        limitations=['This does not install motors or validate tendon transmission.',
            'Hand base and forearm are fixed relative to one another for this comparison.',
            'Packaging self-interference, arm motion, continuous hand motion, fastening strength and full joint ranges are outside this screen.'])
    (HERE/'generated/packaging_screen.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
