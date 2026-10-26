"""Bounded palm-tail routing screen, then five endpoint checks of the selection."""
import importlib.util
from pathlib import Path
import sys
import time

import cadquery as cq
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_palm_lead_model',HERE/'fan_model.py')
g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
c,b,m,p=g.c,g.b,g.m,g.p


def main():
    out=HERE/'reviews/fan.json'
    if out.exists():raise FileExistsError('Preserve prior palm lead screen')
    started=time.monotonic()
    source={z:b.sha(ROOT/z) for z in ['cad/palm_motor_lead_v1/fan_model.py','cad/palm_motor_lead_v1/zero_model.py','cad/palm_motor_lead_v1/fan_build.py',
        'cad/palm_motor_layout_v1/selected/generated/layout.json','cad/palm_motor_mount_v1/selected3/generated/scene.json',
        'cad/remote_motor_lead_v2/generated/scene.json','cad/wiring_access_v10/connector_reference.json',
        'cad/palm_motor_layout_v1/reviews/check2.json']}
    saved=p.read(ROOT/'cad/palm_motor_mount_v1/selected3/generated/scene.json')
    pack=c.load(saved['new_parts']+saved['retained_parts'],source);assert len(pack)==360
    remote=c.load(p.read(ROOT/'cad/remote_motor_lead_v2/generated/scene.json')['parts'],source)
    motors=[z for z in p.read(ROOT/'cad/palm_motor_layout_v1/selected/generated/layout.json')['modules'] if z['zone']=='palm']
    assert len(motors)==4
    scene=p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')
    for z in scene['hand']:source.update({z['step']:z['sha256_step'],z['mesh']:z['sha256_stl']})
    fixedpalm=c.load([z for z in scene['hand'] if z['body']=='palm'],source)
    plugrows=g.plugs(motors)
    report=dict(completed=False,source_sha256=source,reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,
        hand_geometry_changed=False,motor_positions_changed=False,mount_changed=False,
        wire_OD_mm=1.9,selected=[],screens=[],cases=[],installed=False,wire_sku_verified=False,
        connector_fit_verified=False,bend_life_validated=False,strain_relief_complete=False,connected_bus=False,
        scope='Four fixed palm motor electrical tail space;finite candidates and five hand endpoints only.No powered motion,full harness,mating,supplier bend approval or dynamic service loop.')
    # Preserve the actual failure of copying the adopted remote R10/stub5 route.
    baseline=[r for motor in motors for r in g.wires(motor,radius=10.,stub=5.,straight=18.,angle=0.)]
    original=p.read(HERE/'reviews/check.json');assert original['completed'] and not original['selected_all_modules'];source['cad/palm_motor_lead_v1/reviews/check.json']=b.sha(HERE/'reviews/check.json');report['baseline_reused_from']='reviews/check.json';report['remote_copy_packaging']=original['remote_copy_packaging']
    report['remote_copy_fixed_palm']=original['remote_copy_fixed_palm']
    p.write(out,report)
    selected=[]
    for motor in motors:
        chosen=None
        for angle in [[180.,180.,210.],[180.,180.,225.],[180.,180.,195.]]:
            params=dict(radius=4.,stub=0.,straight=8.,angles=angle)
            rows=g.wires(motor,**params)
            checks=dict(packaging=c.material(rows,pack),fixed_palm=c.material(rows,fixedpalm),
                existing_wiring=c.material(rows,remote),previous_palm_leads=c.material(rows,selected),
                wire_pairs=g.pairs(rows),plug_outer_boxes=c.material(rows,plugrows))
            passed=all(g.clear(z) for z in checks.values())
            report['screens'].append(dict(module=motor['name'],parameters=params,checks=checks,clear=passed))
            p.write(out,report)
            print(motor['name'],angle,'overlaps',sum(len(z['overlaps']) for z in checks.values()),'clear',passed,flush=True)
            if passed:
                chosen=rows;report['selected'].append(dict(module=motor['name'],parameters=params));break
        if chosen is None:
            report.update(completed=True,selected_all_modules=False,elapsed_s=time.monotonic()-started)
            p.write(out,report);return
        selected+=chosen
    _,v,cat,scene,hand=p.load()
    neutral=v.frames(cat['design'],np.zeros(20))
    for goal in p.read(ROOT/'cad/palm_motor_layout_v1/reviews/check2.json')['cases']:
        F=v.frames(cat['design'],goal['angles_deg'])
        current=[dict(name=z['name'],shape=v.relocate(hand[z['name']],F[z['body']],neutral[z['body']])) for z in scene['hand']]
        got=c.material(selected,current)
        report['cases'].append(dict(id=goal['id'],angles_deg=goal['angles_deg'],native=got))
        p.write(out,report);print('hand',goal['id'],len(got['overlaps']),flush=True)
    report['packaging']=c.material(selected,pack)
    report['existing_wiring']=c.material(selected,remote)
    report['new_wire_pairs']=g.pairs(selected)
    report['new_plug_boxes']=c.material(selected,plugrows)
    arm=[]
    for r in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference']:
        rel='cad/modular_roots_v9/parts/'+r['asset']+'.step';source[rel]=b.sha(ROOT/rel)
        arm.append(dict(name=r['name'],shape=cq.importers.importStep(str(ROOT/rel)).val()))
    report['neutral_arm']=c.material(selected,arm)
    # Solid external connector boxes are kept as unresolved mating envelopes.
    report['plug_vs_packaging']=c.material(plugrows,pack)
    report['plug_vs_fixed_palm']=c.material(plugrows,fixedpalm)
    folder=HERE/'generated/parts';folder.mkdir(parents=True,exist_ok=False)
    savedrows=[]
    for r in selected+plugrows:
        step=folder/(r['name']+'.step');mesh=folder/(r['name']+'.stl')
        cq.exporters.export(r['shape'],str(step));cq.exporters.export(r['shape'],str(mesh),tolerance=.05,angularTolerance=.1)
        savedrows.append(dict(**{k:v for k,v in r.items() if k!='shape'},step=step.relative_to(ROOT).as_posix(),mesh=mesh.relative_to(ROOT).as_posix(),
            sha256_step=b.sha(step),sha256_stl=b.sha(mesh),bbox_mm=m.pack._bbox(r['shape'])))
    p.write(HERE/'generated/scene.json',dict(parts=savedrows,wire_count=12,connector_outer_count=4,
        installed=False,wire_OD_mm=1.9,centreline_radius_mm=4.,radius_basis='Unqualified static geometry assumption;not supplier minimum or bend-life approval.',
        termination='Twelve open short-tail endpoints;no connected bus,board,axial restraint or joint-crossing cable.'))
    checks=[z['native'] for z in report['cases']]+[report[k] for k in ['packaging','existing_wiring','new_wire_pairs','new_plug_boxes','neutral_arm']]
    report.update(completed=True,selected_all_modules=True,nominal_leads_clear=all(g.clear(z) for z in checks),
        input_hash_guard=all(b.sha(ROOT/z)==h for z,h in source.items()),elapsed_s=time.monotonic()-started)
    p.write(out,report)
    print('complete',report['nominal_leads_clear'],flush=True)


if __name__=='__main__':main()
