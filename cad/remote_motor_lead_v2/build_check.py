"""Screen a tilted bend plane, retaining every prior successful fixed tail."""
import importlib.util
from pathlib import Path
import sys
import time

import cadquery as cq
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_post_route',HERE/'route.py')
t=importlib.util.module_from_spec(spec)
spec.loader.exec_module(t)
b,m,p=t.b,t.m,t.p


def main():
    out=HERE/'reviews/check.json'
    if out.exists():
        raise FileExistsError('Preserve previous checks')
    start=time.monotonic()
    inherited=p.read(ROOT/'cad/remote_motor_lead_v1/reviews/batch.json')
    source=dict(inherited['source_sha256'])
    for rel in ['cad/remote_motor_lead_v2/build_check.py','cad/remote_motor_lead_v2/route.py',
        'cad/remote_motor_lead_v1/reviews/batch.json','cad/remote_motor_lead_v1/generated/batch_scene.json']:
        source[rel]=b.sha(ROOT/rel)
    rows=t.saved_rows()
    for row in rows:
        source[row['step']]=row['sha256_step']
        source[row['mesh']]=row['sha256_stl']
    native,v,cat,scene,hand=p.load()
    obstacles=t.c.load_obstacles()
    plugs=[dict(name=r['name'],shape=cq.importers.importStep(str(ROOT/r['step'])).val()) for r in rows if r['role']!='nominal_electrical_wire_space']
    before=[dict(name=r['name'],shape=cq.importers.importStep(str(ROOT/r['step'])).val()) for r in rows
        if r['role']=='nominal_electrical_wire_space' and r['module'] in t.CHANGED]
    before_check=b.test_against(before,obstacles,v)
    report=dict(completed=False,source_sha256=source,changed_modules=sorted(t.CHANGED),changed_leads=12,
        remote_modules=16,total_leads=48,unchanged_wire_geometry=36,before=before_check,candidates=[],selected=None,
        hand_pose_checks=[],neutral_arm_check=None,reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,
        assumptions=['WireOD1.9mm,centreline radius10mm and5mm straight exit remain engineering provisions, not actual supplied cable/bend-life ratings.',
            'Bend plane rotation is electrical wire routing only, not a motor target or joint/module relocation.'],
        hand_geometry_changed=False,motor_locations_changed=False,rack_geometry_changed=False,
        connector_fit_verified=False,strain_relief_complete=False,connected_bus_complete=False,upstream_fixed_palm_pairs=8)
    chosen=None
    for angle in [35.,37.,39.]:
        leads=t.build_leads(angle,rows)
        packaging=b.test_against(leads,obstacles,v)
        wirepairs=t.old.wire_pairs(leads,v)
        envelopes=b.test_against(leads,plugs,v)
        clear=all(not r['overlaps'] and not r['operation_failures'] for r in [packaging,wirepairs,envelopes])
        report['candidates'].append(dict(bend_plane_angle_deg=angle,packaging=packaging,wire_pairs=wirepairs,wire_vs_connector_envelopes=envelopes,clear=clear))
        p.write(out,report)
        print('angle',angle,'packaging',len(packaging['overlaps']),'wirepairs',len(wirepairs['overlaps']),'clear',clear,flush=True)
        if clear:
            chosen=leads
            report['selected']=angle
            break
    if chosen:
        neutral=v.frames(cat['design'],np.zeros(20))
        for case in inherited['hand_pose_checks']:
            frames=v.frames(cat['design'],case['angles_deg'])
            current=[dict(name=r['name'],shape=v.relocate(hand[r['name']],frames[r['body']],neutral[r['body']])) for r in scene['hand']]
            report['hand_pose_checks'].append(dict(id=case['id'],angles_deg=case['angles_deg'],native=b.test_against(chosen,current,v)))
        arm=[]
        for row in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference']:
            rel='cad/modular_roots_v9/parts/'+row['asset']+'.step'
            source[rel]=b.sha(ROOT/rel)
            arm.append(dict(name=row['name'],shape=cq.importers.importStep(str(ROOT/rel)).val()))
        report['neutral_arm_check']=b.test_against(chosen,arm,v)
        saved=[]
        replacements={row['name']:row for row in chosen if row['module'] in t.CHANGED}
        folder=HERE/'generated/parts'
        folder.mkdir(parents=True,exist_ok=False)
        for old in rows:
            if old['name'] not in replacements:
                saved.append(old)
                continue
            row=replacements[old['name']]
            step=folder/(row['name']+'.step')
            mesh=folder/(row['name']+'.stl')
            cq.exporters.export(row['shape'],str(step))
            cq.exporters.export(row['shape'],str(mesh),tolerance=.05,angularTolerance=.1)
            saved.append(dict(old,step=step.relative_to(ROOT).as_posix(),mesh=mesh.relative_to(ROOT).as_posix(),
                sha256_step=b.sha(step),sha256_stl=b.sha(mesh),local=row['local'],
                world_end_mm=(np.asarray(old['transform_world_mm'])@np.r_[row['local']['end_mm'],1.])[:3].tolist()))
        p.write(HERE/'generated/scene.json',dict(parts=saved,total_leads=48,changed_leads=12,
            unchanged_wire_geometry=36,changed_modules=sorted(t.CHANGED),bend_plane_angle_deg=report['selected'],
            connector_fit_verified=False,board_termination='Open endpoints; no connected bus/board/strain relief yet.'))
    checks=[r['native'] for r in report['hand_pose_checks']]
    if report['neutral_arm_check']:
        checks.append(report['neutral_arm_check'])
    report.update(completed=True,input_hash_guard=all(b.sha(ROOT/r)==h for r,h in source.items()),
        all_nominal_leads_clear=bool(chosen) and all(not r['overlaps'] and not r['operation_failures'] for r in checks),elapsed_s=time.monotonic()-start)
    p.write(out,report)
    if not report['input_hash_guard']:
        raise ValueError('Changed input')


if __name__=='__main__':
    main()
