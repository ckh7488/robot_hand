"""Save a bounded four-port study without adopting unresolved wire geometry."""
import importlib.util
from pathlib import Path
import sys
import time

import cadquery as cq

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_palm_lead_assessment',HERE/'model.py')
g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
c,b,m,p=g.c,g.b,g.m,g.p


def main():
    out=HERE/'reviews/assessment.json'
    if out.exists():raise FileExistsError('Preserve assessment')
    started=time.monotonic();source={z:b.sha(ROOT/z) for z in ['cad/palm_motor_lead_v1/assessment.py','cad/palm_motor_lead_v1/model.py',
        'cad/palm_motor_mount_v1/selected3/generated/scene.json','cad/palm_motor_layout_v1/selected/generated/layout.json',
        'cad/remote_motor_lead_v2/generated/scene.json','cad/wiring_access_v10/connector_reference.json']}
    mount=p.read(ROOT/'cad/palm_motor_mount_v1/selected3/generated/scene.json')
    pack=c.load(mount['new_parts']+mount['retained_parts'],source)
    oldwires=c.load(p.read(ROOT/'cad/remote_motor_lead_v2/generated/scene.json')['parts'],source)
    catalog=p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')
    for z in catalog['hand']:source.update({z['step']:z['sha256_step'],z['mesh']:z['sha256_stl']})
    fixed=c.load([z for z in catalog['hand'] if z['body']=='palm'],source)
    motors=[z for z in p.read(ROOT/'cad/palm_motor_layout_v1/selected/generated/layout.json')['modules'] if z['zone']=='palm']
    params=dict(radius=4.,stub=.5,straight=8.,angle=180.)
    wires=[r for motor in motors for r in g.wires(motor,**params)];plugs=g.plugs(motors)
    assert len(pack)==360 and len(motors)==4 and len(wires)==12 and len(plugs)==4
    original=p.read(HERE/'reviews/check.json');assert original['completed'] and not original['selected_all_modules']
    report=dict(completed=False,source_sha256=source,candidate_parameters=params,hand_parts_hashed=364,fixed_palm_parts_imported=len(fixed),
        reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,hand_geometry_changed=False,motor_positions_changed=False,mount_changed=False,
        candidate_adopted=False,wire_sku_verified=False,connector_fit_verified=False,bend_life_validated=False,
        no_new_hand_endpoint_or_path_checks=True,selected_all_modules=False,
        scope='Four fixed palm port spaces,unadopted wire envelope study.Static obstruction already present;no full364hand endpoints or movement rerun.No supplier radius,mating,bus or physical approval.')
    for key,a,z in [('packaging',wires,pack),('fixed_palm',wires,fixed),('existing_wiring',wires,oldwires),('new_plug_boxes',wires,plugs),
                    ('plug_vs_packaging',plugs,pack),('plug_vs_fixed_palm',plugs,fixed)]:
        report[key]=c.material(a,z);p.write(out,report);print(key,len(report[key]['overlaps']),flush=True)
    report['new_wire_pairs']=g.pairs(wires)
    arm=[]
    for r in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference']:
        rel='cad/modular_roots_v9/parts/'+r['asset']+'.step';source[rel]=b.sha(ROOT/rel)
        arm.append(dict(name=r['name'],shape=cq.importers.importStep(str(ROOT/rel)).val()))
    report['neutral_arm']=c.material(wires,arm)
    for key in ['remote_copy_packaging','remote_copy_fixed_palm']:report[key]=original[key]
    report['baseline_reused_from']='reviews/check.json;same rigid parts and registered route,no new calculation'
    folder=HERE/'generated/parts';folder.mkdir(parents=True,exist_ok=False)
    rows=[]
    for r in wires+plugs:
        step=folder/(r['name']+'.step');mesh=folder/(r['name']+'.stl')
        cq.exporters.export(r['shape'],str(step));cq.exporters.export(r['shape'],str(mesh),tolerance=.05,angularTolerance=.1)
        rows.append(dict(**{k:v for k,v in r.items() if k!='shape'},step=step.relative_to(ROOT).as_posix(),mesh=mesh.relative_to(ROOT).as_posix(),
            sha256_step=b.sha(step),sha256_stl=b.sha(mesh),bbox_mm=m.pack._bbox(r['shape'])))
    p.write(HERE/'generated/scene.json',dict(parts=rows,wire_count=12,connector_outer_count=4,candidate_adopted=False,
        wire_OD_mm=1.9,centreline_radius_mm=4.,parameters=params,termination='Unadopted open tail envelopes;not routed complete harness.',
        radius_basis='Unqualified space assumption;actual supplied cable/terminal/housing/exit geometry requires confirmation.'))
    tests=[report[k] for k in ['packaging','fixed_palm','existing_wiring','new_wire_pairs','new_plug_boxes','neutral_arm']]
    report.update(completed=True,nominal_leads_clear=all(g.clear(z) for z in tests),
        input_hash_guard=all(b.sha(ROOT/z)==h for z,h in source.items()),elapsed_s=time.monotonic()-started)
    p.write(out,report)


if __name__=='__main__':main()
