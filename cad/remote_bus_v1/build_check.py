"""Check two fixed main runs and explicitly provisional connector spaces."""
import itertools
from pathlib import Path
import sys
import time

sys.path.insert(0,str(Path(__file__).resolve().parent))
import model as g
c,b,m,p,cq,np,HERE,ROOT=g.c,g.b,g.m,g.p,g.cq,g.np,g.HERE,g.ROOT


def hand_boxes(objects,source=None):
    v=g.module('_bus_fk',ROOT/'cad/articulation_drive_v2/verify.py')
    catalog=p.read(ROOT/'cad/functional_motion_v3/generated/catalog.json')
    scene=p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')
    neutral=v.frames(catalog['design'],np.zeros(20));cases=[];cache={}
    if source is not None:
        for z in scene['hand']:source.update({z['step']:z['sha256_step'],z['mesh']:z['sha256_stl']})
        source.update(catalog['input_sha256'])
    bounds=[np.asarray(m.pack._bbox(z['shape'])) for z in objects]
    for case in p.read(ROOT/'cad/palm_motor_layout_v1/reviews/check2.json')['cases']:
        F=v.frames(catalog['design'],case['angles_deg']);candidates=[];exact=[];gaps=[]
        for row in scene['hand']:
            box=np.asarray(row['bbox_mm']);assert box.shape==(2,3) and row['valid']
            corners=np.array(list(itertools.product(*zip(box[0]-.002,box[1]+.002))))
            origin,R=F[row['body']];oldorigin,oldR=neutral[row['body']]
            relative=R@oldR.T;t=origin-relative@oldorigin
            pts=corners@relative.T+t;posed=np.array([pts.min(axis=0),pts.max(axis=0)])
            for obj,other in zip(objects,bounds):
                overlap=np.minimum(posed[1],other[1])-np.maximum(posed[0],other[0])
                if np.any(overlap<=0):gaps.append(float(np.max(-overlap)));continue
                candidates.append([obj['name'],row['name']])
                if row['name'] not in cache:cache[row['name']]=cq.importers.importStep(str(ROOT/row['step'])).val()
                moved=v.relocate(cache[row['name']],F[row['body']],neutral[row['body']])
                exact.append(c.material([obj],[dict(name=row['name'],shape=moved)]))
        cases.append(dict(id=case['id'],angles_deg=case['angles_deg'],pairs_in_scope=len(objects)*364,
            transformed_saved_neutral_bbox=True,numerical_bbox_padding_mm=.002,native_candidates=candidates,native_refinements=exact,
            minimum_separating_axis_gap_mm=min(gaps),clear=all(g.clear(z) for z in exact)))
    return dict(cases=cases,hand_parts_hashed=364,hand_parts_native_imported=len(cache),
        method='Eight corners of saved neutral STEP AABBs,rigid FK transform,0.002mm numerical padding.Native refinement for every overlapping box only.No full hand self-collision/path or wrist/arm movement replay.')


def continuity(wires,old):
    index={z['name']:z for z in old};result=[]
    for row in wires:
        source=index[row['source_lead']];a,z=row['shape'],source['shape']
        volume=float(a.intersect(z,tol=1e-6).Volume());distance=float(a.distance(z))
        united=a.fuse(z,tol=1e-6)
        got=dict(name=row['name'],source_lead=source['name'],end_start_distance_mm=distance,common_volume_mm3=volume,
            start_matches_saved_end=bool(np.allclose(row['start_mm'],source['world_end_mm'],rtol=0,atol=1e-8)),
            tangent_dot=1.,combined_single_solid=len(united.Solids())==1,combined_valid=united.isValid(),
            combined_volume_error_mm3=abs(united.Volume()-a.Volume()-z.Volume()))
        got['matched']=got['start_matches_saved_end'] and distance<1e-5 and abs(volume)<.001 and got['combined_single_solid'] and got['combined_valid'] and got['combined_volume_error_mm3']<1e-4
        result.append(got)
    return result


def main():
    out=HERE/'reviews/check.json'
    if out.exists():raise FileExistsError('Preserve previous bus check')
    started=time.monotonic();source={z:b.sha(ROOT/z) for z in ['cad/remote_bus_v1/model.py','cad/remote_bus_v1/build_check.py',
        'cad/remote_motor_lead_v2/generated/scene.json','cad/palm_motor_mount_v1/selected3/generated/scene.json',
        'cad/motion_clearance_v2/generated/assembly_catalog.json','cad/functional_motion_v3/generated/catalog.json',
        'cad/palm_motor_layout_v1/reviews/check2.json','cad/articulation_drive_v2/verify.py']}
    saved=p.read(ROOT/'cad/palm_motor_mount_v1/selected3/generated/scene.json')
    pack=c.load(saved['new_parts']+saved['retained_parts'],source);assert len(pack)==360
    oldrows=p.read(ROOT/'cad/remote_motor_lead_v2/generated/scene.json')['parts'];old=c.load(oldrows,source)
    wires=g.build(oldrows);terms=g.terminations(wires);new=wires+terms
    arm=[]
    for z in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference']:
        rel='cad/modular_roots_v9/parts/'+z['asset']+'.step';source[rel]=b.sha(ROOT/rel)
        arm.append(dict(name=z['name'],shape=cq.importers.importStep(str(ROOT/rel)).val()))
    report=dict(completed=False,source_sha256=source,reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,
        hand_geometry_changed=False,motor_positions_changed=False,rack_and_mount_changed=False,old48leads_changed=False,
        new_main_run_wires=6,source_modules=g.MODULES,other14remote_modules_connected=False,complete16motor_bus=False,
        termination_space_count=2,board_selected=False,wire_sku_verified=False,axial_strain_relief_complete=False,
        installed=False,scope='Two3wire main-run continuations to temporary termination spaces.Static packaging/neutral arm and5hand pose bounding checks;not full electrical bus,assembly/restraint,dynamic arm/cable or physical approval.')
    for key,a,z in [('packaging',new,pack),('existing_wires_and_outerboxes',new,old),('neutral_arm',new,arm)]:
        report[key]=c.material(a,z);p.write(out,report);print(key,len(report[key]['overlaps']),flush=True)
    report['new_pairs']=g.pairs(new)
    report['continuity']=continuity(wires,old)
    report['hand_pose_bounds']=hand_boxes(new,source)
    report['termination_faces']=[dict(wire=z['name'],region=t['name'],distance_mm=float(z['shape'].distance(t['shape'])))
        for z in wires for t in terms if z['module']==t['module']]
    folder=HERE/'generated/parts';folder.mkdir(parents=True,exist_ok=False)
    rows=[]
    for z in new:
        step=folder/(z['name']+'.step');mesh=folder/(z['name']+'.stl')
        cq.exporters.export(z['shape'],str(step));cq.exporters.export(z['shape'],str(mesh),tolerance=.05,angularTolerance=.1)
        rows.append(dict(**{k:v for k,v in z.items() if k!='shape'},step=step.relative_to(ROOT).as_posix(),mesh=mesh.relative_to(ROOT).as_posix(),
            sha256_step=b.sha(step),sha256_stl=b.sha(mesh),bbox_mm=m.pack._bbox(z['shape'])))
    p.write(HERE/'generated/scene.json',dict(parts=rows,wire_extensions=6,temporary_spaces=2,space_only=True,installed=False,
        source_remote_leads='cad/remote_motor_lead_v2/generated/scene.json',current_packaging='cad/palm_motor_mount_v1/selected3/generated/scene.json',
        unconnected_remote_modules=14,complete_bus=False,assumptions='OD1.9mm and10/12/14mm static centreline bends;not supplied wire limits.14x20x6mm temporary connector regions are not a selected board or component.'))
    checks=[report[k] for k in ['packaging','existing_wires_and_outerboxes','neutral_arm','new_pairs']]
    report.update(completed=True,nominal_space_clear=all(g.clear(z) for z in checks) and all(z['clear'] for z in report['hand_pose_bounds']['cases']),
        continuity_confirmed=all(z['matched'] for z in report['continuity']),input_hash_guard=all(b.sha(ROOT/z)==h for z,h in source.items()),
        elapsed_s=time.monotonic()-started)
    p.write(out,report);print('complete',report['nominal_space_clear'],report['continuity_confirmed'],flush=True)


if __name__=='__main__':main()
