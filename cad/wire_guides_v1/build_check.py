"""Check final guide material and accessible lids without hiding prior palm clashes."""
import importlib.util
from pathlib import Path
import sys
import time

import cadquery as cq
import numpy as np
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.gp import gp_Pnt
from OCP.TopAbs import TopAbs_IN

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_guide_build',HERE/'model.py')
g=importlib.util.module_from_spec(spec)
spec.loader.exec_module(g)
b,m,p=g.b,g.m,g.p


def load_saved(rows):
    out=[]
    for row in rows:
        if b.sha(ROOT/row['step'])!=row['sha256_step']:
            raise ValueError('Changed inherited geometry')
        out.append(dict(name=row['name'],module=row.get('module'),shape=cq.importers.importStep(str(ROOT/row['step'])).val()))
    return out


def main():
    out=HERE/'reviews/check.json'
    if out.exists():
        raise FileExistsError('Preserve prior guide checks')
    start=time.monotonic()
    previous=p.read(ROOT/'cad/remote_motor_lead_v2/reviews/check.json')
    if not previous['completed'] or not previous['all_nominal_leads_clear']:
        raise ValueError('No screened wire source')
    source=dict(previous['source_sha256'])
    for rel in ['cad/wire_guides_v1/model.py','cad/wire_guides_v1/build_check.py',
        'cad/remote_motor_lead_v2/reviews/check.json','cad/remote_motor_lead_v2/generated/scene.json']:
        source[rel]=b.sha(ROOT/rel)
    packaging=p.read(ROOT/'cad/wiring_layout_v1/generated/packaging.json')['parts']
    new,groups=g.build()
    names={r['name'] for r in new}
    oldreplaced=[r for r in packaging if r['name'] in names]
    retained_meta=[r for r in packaging if r['name'] not in names]
    assert len(oldreplaced)==32 and len(retained_meta)==193 and len(new)==160
    retained=load_saved(retained_meta)
    wiresmeta=p.read(ROOT/'cad/remote_motor_lead_v2/generated/scene.json')['parts']
    wires=load_saved([r for r in wiresmeta if r['role']=='nominal_electrical_wire_space'])
    plugs=load_saved([r for r in wiresmeta if r['role']!='nominal_electrical_wire_space'])
    for row in packaging+wiresmeta:
        source[row['step']]=row['sha256_step']
        source[row['mesh']]=row['sha256_stl']
    native,v,cat,scene,hand=p.load()
    report=dict(completed=False,source_sha256=source,new_parts=len(new),retained_packaging=len(retained),replaced_screws=32,
        new_guides=16,wire_geometry_changed=False,hand_geometry_changed=False,motor_locations_changed=False,rack_geometry_changed=False,
        scope='Nominal removable transverse guide candidates, not verified axial strain relief, motor installation or physical release.',
        reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,axial_strain_relief_validated=False,wire_sku_verified=False,
        supplier_fasteners_selected=False,upstream_palm_pairs=8,
        assumptions=['Existing generic M3 head retained with12mmshaft and custom1mmspacer. M2x6/nuts are nominal unthreaded envelopes; torque/preload/complete-thread supply geometry not certified.',
            'NominalOD1.9mm cable,groove7.5mmwide/2.4mmhigh with removable lid. Captures sideways; permits axial sliding, no pull-force/friction/wear qualification.'],
        final_checks={},nominal_mount_checks=[],nominal_wire_support=[],lid_removal=[],hand_pose_checks=[],neutral_arm_check=None)
    report['final_checks']['new_vs_retained']=b.test_against(new,retained,v)
    report['final_checks']['new_pairs']=g.t.old.wire_pairs(new,v)
    report['final_checks']['new_vs_wires']=b.test_against(new,wires,v)
    report['final_checks']['new_vs_plug_envelopes']=b.test_against(new,plugs,v)
    p.write(out,report)
    print({key:len(value['overlaps']) for key,value in report['final_checks'].items()},flush=True)
    newmap={r['name']:r for r in new};oldmap={r['name']:r for r in retained}
    for group in groups:
        name=group['module'];base=newmap[name+'_wire_guide_base']['shape'];adapter=oldmap[name+'_adapter']['shape']
        witnesses=[]
        for xy in group['bolt_axis_xy_mm']:
            for dx,dy in [(2.3,0),(-2.3,0),(0,2.3),(0,-2.3),(2.3,2.3),(-2.3,-2.3)]:
                states=[BRepClass3d_SolidClassifier(s.Solids()[0].wrapped,gp_Pnt(xy[0]+dx,xy[1]+dy,z),1e-6).State()==TopAbs_IN
                    for s,z in [(base,-79.05),(adapter,-78.95)]]
                witnesses.append(dict(xy_mm=[xy[0]+dx,xy[1]+dy],base_below_and_adapter_above_IN=states))
        report['nominal_mount_checks'].append(dict(module=name,base_adapter_gap_mm=float(base.distance(adapter)),
            bearing_plane_z_mm=-79,witnesses=witnesses,nominal_nut_tip_projection_mm=.6,
            passed=all(all(r['base_below_and_adapter_above_IN']) for r in witnesses) and float(base.distance(adapter))<=1e-5))
        for wire in [r for r in wires if r['module']==name]:
            report['nominal_wire_support'].append(dict(wire=wire['name'],base_gap_mm=float(wire['shape'].distance(base))))
    # Lids lift12mm after their two M2 screws are removed; guide bases stay bolted.
    lids=[r for r in new if r['kind']=='removable_lid']
    fixed=[r for r in new if r['kind'] not in ['removable_lid','M2x6_envelope']]+retained+wires+plugs
    for distance in [0.,3.,6.,9.,12.]:
        moving=[dict(name=r['name'],shape=r['shape'].translate((0,0,distance))) for r in lids]
        report['lid_removal'].append(dict(lift_mm=distance,native=b.test_against(moving,fixed,v)))
    neutral=v.frames(cat['design'],np.zeros(20))
    for case in previous['hand_pose_checks']:
        frames=v.frames(cat['design'],case['angles_deg'])
        current=[dict(name=r['name'],shape=v.relocate(hand[r['name']],frames[r['body']],neutral[r['body']])) for r in scene['hand']]
        report['hand_pose_checks'].append(dict(id=case['id'],angles_deg=case['angles_deg'],native=b.test_against(new,current,v)))
    arm=[]
    for row in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference']:
        rel='cad/modular_roots_v9/parts/'+row['asset']+'.step';source[rel]=b.sha(ROOT/rel)
        arm.append(dict(name=row['name'],shape=cq.importers.importStep(str(ROOT/rel)).val()))
    report['neutral_arm_check']=b.test_against(new,arm,v)
    folder=HERE/'generated/parts';folder.mkdir(parents=True,exist_ok=False)
    saved=[]
    for row in new:
        step=folder/(row['name']+'.step');mesh=folder/(row['name']+'.stl')
        cq.exporters.export(row['shape'],str(step));cq.exporters.export(row['shape'],str(mesh),tolerance=.05,angularTolerance=.1)
        saved.append(dict(name=row['name'],module=row['module'],kind=row['kind'],color=row['color'],
            step=step.relative_to(ROOT).as_posix(),mesh=mesh.relative_to(ROOT).as_posix(),sha256_step=b.sha(step),sha256_stl=b.sha(mesh)))
    p.write(HERE/'generated/scene.json',dict(new_parts=saved,retained_packaging=retained_meta,replaced_packaging=oldreplaced,
        groups=groups,wire_scene='cad/remote_motor_lead_v2/generated/scene.json',axial_strain_relief_validated=False))
    checks=[*report['final_checks'].values(),report['neutral_arm_check'],*[r['native'] for r in report['hand_pose_checks']]]
    report.update(completed=True,input_hash_guard=all(b.sha(ROOT/r)==h for r,h in source.items()),
        final_nominal_clear=all(not r['overlaps'] and not r['operation_failures'] for r in checks),
        all_mounts_nominal_seated=all(r['passed'] for r in report['nominal_mount_checks']),
        all_wires_nominal_supported=all(r['base_gap_mm']<=1e-5 for r in report['nominal_wire_support']),
        lids12mm_sampled_clear=all(not r['native']['overlaps'] and not r['native']['operation_failures'] for r in report['lid_removal']),
        elapsed_s=time.monotonic()-start)
    p.write(out,report)
    if not report['input_hash_guard']:
        raise ValueError('Changed guide input')


if __name__=='__main__':
    main()
