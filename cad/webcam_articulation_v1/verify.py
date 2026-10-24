"""Measure actual STEP contact gaps and inter-body overlaps in fixed poses."""
import json
import argparse
import hashlib
import math
from pathlib import Path
import re
import cadquery as cq
import numpy as np
from scipy.spatial.transform import Rotation
from OCP.gp import gp_Trsf,gp_Quaternion,gp_Vec
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from thumb_shell import verify_distal_axes

HERE=Path(__file__).resolve().parent

def rotate(axis,degrees):
    return Rotation.from_euler(axis,degrees,degrees=True).as_matrix()

def frames(design,angles):
    result={'world':(np.zeros(3),np.eye(3))}
    for fi,config in enumerate([*design['fingers'],dict(design['thumb'],name='thumb')]):
        a=angles[fi*4:fi*4+4];o=np.array(config['root'],float)
        r=rotate('z',config.get('base_yaw_deg',0)-a[0])
        result[(config['name'],0)]=(o.copy(),r.copy())
        o+=r@config.get('flexion_offset',[0,0,0]);r=r@rotate('x',a[1])
        result[(config['name'],1)]=(o.copy(),r.copy())
        for ji in (2,3):
            o+=r@np.array([0,config['links'][ji-2],0])
            if ji==2:r=r@rotate('z',config.get('second_yaw_deg',0))@rotate('y',config.get('second_roll_deg',0))
            r=r@rotate('x',a[ji]);result[(config['name'],ji)]=(o.copy(),r.copy())
    return result

def parent(name):
    match=re.match(r'^(index|middle|ring|little|thumb)_(.*)$',name)
    if not match:return 'world'
    finger,rest=match.groups()
    if rest=='spread_housing':return 'world'
    if rest.startswith(('spread_ring','root_fork')):return finger,0
    if rest.startswith('joint_pin_'):return finger,int(rest[-1])
    return finger,int(re.match(r'segment([123])_',rest)[1])

def move(shape,frame,neutral):
    o,r=frame;on,rn=neutral
    rotation=r@rn.T;translation=o-rotation@on
    transform=gp_Trsf();transform.SetRotation(gp_Quaternion(*Rotation.from_matrix(rotation).as_quat()))
    transform.SetTranslationPart(gp_Vec(*translation))
    return shape.moved(cq.Location(transform))

def bounds(shape):
    box=Bnd_Box();BRepBndLib.AddOptimal_s(shape.wrapped,box,False,False)
    values=box.Get();return np.array(values[:3]),np.array(values[3:])

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fist-only',action='store_true')
    options=parser.parse_args()
    design=json.loads((HERE/'design.json').read_text())
    scene=json.loads((HERE/'generated/scene.json').read_text())['hand']
    solids=cq.importers.importStep(str(HERE/'generated/hand_articulation_candidate.step')).val().Solids()
    parts={};import_checks=[]
    if len(solids)!=len(scene):raise ValueError('STEP part count mismatch')
    for row,solid in zip(scene,solids):
        delta=abs(solid.Volume()-row['volume_mm3'])
        # STEP re-import can perturb trimmed BRep volume by ~0.02 mm3.
        # Check volume AND spatial bounds to establish the part correspondence.
        box_error=float(np.max(np.abs(np.array(bounds(solid))-row['bbox_mm'])))
        if delta>max(.05,row['volume_mm3']*1e-5) or box_error>.002:
            raise ValueError(f"STEP mapping mismatch: {row['name']}, stored={row['volume_mm3']}, STEP={solid.Volume()}")
        import_checks.append(dict(name=row['name'],volume_delta_mm3=delta,bounds_max_delta_mm=box_error))
        parts[row['name']]=solid
    neutral=frames(design,np.zeros(20));fist=np.array(design['fist_deg'])
    local_thumb=move(parts['thumb_segment1_thenar_shell'],(np.zeros(3),np.eye(3)),neutral[('thumb',1)])
    bores=verify_distal_axes([dict(shape=local_thumb,solid_count=len(local_thumb.Solids()))],
        design['thumb']['links'][0],design['thumb']['second_yaw_deg'])
    # Actual main structures, root forks and fingertip pads. Bearings, pins,
    # screws, fascia attachment and same-link assembly interfaces are excluded.
    names=[n for n in parts if n=='Palm_shaped_front_cover' or any(s in n for s in
           ('segment_shell','thenar_shell','tip_core','rounded_tip_cap','root_fork'))]
    report=dict(scope='STEP structural inter-body checks in sampled poses; not full configuration-space clearance',
                excluded='Bearings, pins, screws, fascia attachments, same-body mating parts, rear palm and motor hardware',
                checked_parts=names,overlap_threshold_mm3=.02,step_reimport=import_checks,
                actual_thumb_distal_bores=bores,
                input_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in
                    (HERE/'design.json',HERE/'generated/hand_articulation_candidate.step',HERE/'generated/scene.json')},
                target_fist_deg=fist.tolist(),cases=[])
    path=design['fist_path']
    fractions=sorted(set(float(t) for a,b in zip(path,path[1:]) for t in np.linspace(a['fraction'],b['fraction'],5)))
    for fraction in ((1,) if options.fist_only else fractions):
        angles=np.array([np.interp(fraction,[p['fraction'] for p in path],[p['angles_deg'][i] for p in path]) for i in range(20)])
        current=frames(design,angles)
        moved={n:move(parts[n],current[parent(n)],neutral[parent(n)]) for n in names}
        boxes={n:bounds(s) for n,s in moved.items()};clashes=[]
        for i,a in enumerate(names):
            for b in names[i+1:]:
                if parent(a)==parent(b):continue
                if np.any(np.minimum(boxes[a][1],boxes[b][1])-np.maximum(boxes[a][0],boxes[b][0])<=1e-6):continue
                volume=moved[a].intersect(moved[b],tol=1e-6).Volume()
                if volume>0.02:clashes.append(dict(a=a,b=b,volume_mm3=volume))
        gaps={f['name']:moved[f['name']+'_segment3_rounded_tip_cap'].distance(parts['Palm_shaped_front_cover']) for f in design['fingers']}
        case=dict(fraction=fraction,angles_deg=angles.tolist(),clashes=clashes,fingertip_gap_mm=gaps)
        report['cases'].append(case)
        print(json.dumps(case),flush=True)
    report['sampled_structural_clearance_passed']=all(not c['clashes'] for c in report['cases'])
    report['fist_fingertip_gaps_within_1_to_3_mm']=all(1<v<3 for v in report['cases'][-1]['fingertip_gap_mm'].values())
    (HERE/'generated'/('fist_review.json' if options.fist_only else 'motion_review.json')).write_text(json.dumps(report,indent=2))
    print('Review saved',flush=True)
    if not report['sampled_structural_clearance_passed'] or not report['fist_fingertip_gaps_within_1_to_3_mm'] or not bores['passed']:
        raise SystemExit(1)

if __name__=='__main__':main()
