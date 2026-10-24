"""Aggregate exact exterior BRep properties; transmission remains a named hypothesis."""
from pathlib import Path
import argparse
import hashlib
import json
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'sim/form_drive_v1'))
from model_adapter import kinematic_blueprint, validate_contract, SCHEMA_VERSION


def build(coupling=1.0):
    raw = json.loads((ROOT / 'cad/form_drive_v1/generated/exterior_mass_inertia.json').read_text())
    parts = {p['name']: p for p in raw['parts']}
    design = json.loads((ROOT / 'cad/form_candidate_v1/design.json').read_text())
    bp = kinematic_blueprint(design, list(parts))
    for b in bp['bodies']:
        rows = [parts[n] for n in b['component_names']]
        mass = sum(p['mass_kg'] for p in rows)
        com = sum(p['mass_kg'] * np.array(p['com_world_mm']) for p in rows) / mass
        inertia = np.zeros((3, 3))
        for p in rows:
            d = (np.array(p['com_world_mm']) - com) / 1000
            inertia += np.array(p['inertia_com_world_kg_m2']) + p['mass_kg'] * (np.dot(d,d)*np.eye(3)-np.outer(d,d))
        T = np.array(b['frame_world_mm']); R = T[:3,:3]
        b.update(mass_kg=mass, com_local_mm=(R.T @ (com-T[:3,3])).tolist(),
                 inertia_com_local_kg_m2=(R.T @ inertia @ R).tolist())
    for i,j in enumerate(bp['joints']):
        # Required trajectory envelope only, not a verified mechanical travel limit.
        j['range_deg'] = [-45,45] if i % 4 == 0 else [-5,100]
    A = np.eye(20)
    for base in range(0,20,4):
        for distal in (2,3):
            for ancestor in range(1,distal):
                A[base+distal,base+ancestor] = coupling
    paths = ['cad/form_candidate_v1/design.json', 'cad/form_candidate_v1/generated/scene.json',
             'cad/form_candidate_v1/generated/hand_form_candidate.step',
             'cad/form_drive_v1/generated/exterior_mass_inertia.json', 'cad/form_drive_v1/exterior.py']
    c = dict(schema_version=SCHEMA_VERSION, design=design, bodies=bp['bodies'], joints=bp['joints'],
        source_sha256={p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in paths},
        mass_scope='120 valid exterior BRep solids; assumed polymer 1250 kg/m3 and steel 7850 kg/m3. Motors are stationary relative to the fixed hand test base and not carried by digits. Moving transmission hardware, retention, tendons and wiring are incomplete and excluded. This is NOT total assembled hand mass.',
        limitations=bp['assumptions']+[
            'No contact/collision forces or validated joint travel in dynamics.',
            'All 20 remote motor paths are assumed compliant pull-pull cables; no finished routing or motor-to-joint fastening.',
            'Spread cable cross-coupling is assumed zero; spatial thumb transmission is unvalidated.',
            'Material density and cable/motor/controller parameters are unmeasured. No physical latency approval.',
            'Missing moving transmission mass makes this an incomplete assembly screening model.',
        ], transmission=dict(status='assumed_not_validated', motor_angle_jacobian=A.tolist(),
            cable_actuator_indices=list(range(20)), hypothesis_upstream_coefficient=coupling,
            assumptions=[f'Motor/joint radii both 8 mm; flex-chain ancestor coefficient {coupling}; spread coupling assumed zero.',
                'Identity diagonal is an assumed 1:1 angle ratio, including root axes, not direct motor attachment.',
                'Coefficient 0 and 0.3 are sensitivity hypotheses, not implemented compensation hardware.',
                'Constant Jacobian cannot represent finite 3D wrapping, Bowden friction or changing fleet angles.']))
    validate_contract(c,repo_root=ROOT)
    return c


if __name__ == '__main__':
    p=argparse.ArgumentParser();p.add_argument('--coupling',type=float,default=1.0);p.add_argument('--output',type=Path)
    a=p.parse_args(); path=a.output or ROOT/'cad/form_drive_v1/generated/dynamics_contract.json'
    path.parent.mkdir(parents=True,exist_ok=True);c=build(a.coupling)
    path.write_text(json.dumps(c,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(output=str(path),body_count=len(c['bodies']),part_count=sum(len(b['component_names']) for b in c['bodies']),
        modeled_exterior_mass_kg=sum(b['mass_kg'] for b in c['bodies']),moving_exterior_mass_kg=sum(b['mass_kg'] for b in c['bodies'][1:]))))
