"""Native saved-STEP whole-hand task audit, including required contacts.

No mating exemptions, clipping, mesh-only collision verdict or clearance target.
The 0.02mm3 reporting floor is inherited and is not a functional safety margin.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import platform
import sys
import time

import cadquery as cq
import numpy as np
from OCP.BRepExtrema import BRepExtrema_DistShapeShape

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True


def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p, value):
    p=Path(p);p.parent.mkdir(parents=True, exist_ok=True)
    temp=p.with_suffix(p.suffix+'.partial');temp.write_text(json.dumps(value,ensure_ascii=False,separators=(',',':'),allow_nan=False)+'\n',encoding='utf-8');temp.replace(p)


def load():
    spec=importlib.util.spec_from_file_location('_functional_native', ROOT/'cad/gesture_hand_v3/audit_gestures.py')
    native=importlib.util.module_from_spec(spec);spec.loader.exec_module(native)
    v=native.load_v2(ROOT);catalog=read(HERE/'generated/catalog.json')
    scene=read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')
    for name,h in catalog['input_sha256'].items():
        if sha(ROOT/name)!=h:raise ValueError('Stale input '+name)
    parts={}
    for i,r in enumerate(scene['hand']):
        if sha(ROOT/r['step'])!=r['sha256_step']:raise ValueError('Changed STEP')
        shape=cq.importers.importStep(str(ROOT/r['step'])).val()
        if not shape.isValid() or len(shape.Solids())!=1:raise ValueError('Invalid saved part')
        parts[r['name']]=shape
        if i%50==0:print(json.dumps(dict(loaded=i+1)),flush=True)
    return native,v,catalog,scene,parts


def cross_surfaces(v, d, parts, scene, q, crossings):
    if not crossings:return []
    current=v.frames(d,q);neutral=v.frames(d,np.zeros(20));result=[]
    for cross in crossings:
        compounds=[]
        for digit,key in [('index','index_segment'),('middle','middle_segment')]:
            segment=cross[key]+1
            rows=[r for r in scene['hand'] if r['name'].startswith(digit+'_segment'+str(segment)+'_')]
            shapes=[v.relocate(parts[r['name']],current[r['body']],neutral[r['body']]) for r in rows]
            compounds.append(cq.Compound.makeCompound(shapes))
        calc=BRepExtrema_DistShapeShape(compounds[0].wrapped,compounds[1].wrapped)
        calc.Perform()
        if not calc.IsDone():raise ValueError('Cross surface distance failed')
        p=calc.PointOnShape1(1);r=calc.PointOnShape2(1)
        points=[list(p.Coord()),list(r.Coord())];xy=np.array(cross['xy_mm'])
        radii=[float(np.linalg.norm(np.array(p)[:2]-xy)) for p in points]
        result.append(dict(**cross, segment_surface_distance_mm=float(calc.Value()),
            closest_points_mm=points, closest_xy_distance_to_cross_mm=radii,
            distance_is_local_to_cross=bool(max(radii)<=25),
            local_definition='Closest saved-segment surface pair within25mm XY of projected crossing; engineering localization only'))
    return result


def compact(engine,q):
    row=engine.evaluate(q)
    row.pop('exact_candidate_results');row.pop('near_pairs')
    return row


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--core-ids',nargs='*');ap.add_argument('--endpoints-only',action='store_true')
    args=ap.parse_args()
    if args.output.exists():raise FileExistsError('Preserve earlier results')
    started=time.monotonic();native,v,cat,scene,parts=load()
    source_paths=['cad/functional_motion_v3/audit.py','cad/gesture_hand_v3/audit_gestures.py','cad/articulation_drive_v2/verify.py','cad/functional_motion_v3/generated/catalog.json']
    report=dict(completed=False,schema='functional_motion_v3.native_audit.1',
        input_sha256={p:sha(ROOT/p) for p in source_paths},catalog_input_sha256=cat['input_sha256'],
        hand_parts=len(parts), all_pairs_per_pose=len(parts)*(len(parts)-1)//2,
        thresholds=dict(material_overlap_reporting_floor_mm3=.02,boolean_tolerance_mm=1e-6),
        clearance_target_applied=False, cases=[],paths=[],
        scope='All saved364parts, samebody/mating pairs included; endpoints and explicitly selected5deg paths. No continuous motion, object retention, strength, cord/motor or physical approval.',
        versions=dict(python=platform.python_version(),cadquery=cq.__version__,numpy=np.__version__),
        hardware_validated=False,whole_hand_motion_approved=False)
    engine=native.PairScreen(v,parts,scene['hand'],cat['design'],0.,True)
    selected=[c for c in cat['cases'] if not args.core_ids or c['id'] in args.core_ids]
    if args.core_ids and len(selected)!=len(set(args.core_ids)):raise ValueError('Unknown core ID')
    for case in selected:
        result=compact(engine,case['angles_deg'])
        row=dict(id=case['id'],label_ko=case['label_ko'],category=case['category'],native=result,
            functional_status='named_shape_requires_inspection')
        if 'contact_digit' in case:
            row['functional_metrics']=case['functional_metrics']
            row['functional_status']='surface_landmark_candidate' if case['functional_metrics']['landmark_target_met'] else 'surface_landmark_target_not_reached'
            a=cat['pad_anchors'][case['contact_digit']]['part'];b=cat['pad_anchors']['thumb']['part']
            current=v.frames(cat['design'],case['angles_deg']);neutral=engine.neutral
            mapping={r['name']:r for r in scene['hand']}
            moved=[v.relocate(parts[n],current[mapping[n]['body']],neutral[mapping[n]['body']]) for n in [a,b]]
            row['native_tip_cap_surface_gap_mm']=float(moved[0].distance(moved[1]))
        if 'crossings' in case:
            row['crossings']=cross_surfaces(v,cat['design'],parts,scene,case['angles_deg'],case['crossings'])
            row['functional_status']='projected_cross_with_measured_segment_gap' if case['crossings'] else 'projected_cross_target_not_reached'
        if result['collision_count']:row['functional_status']='actual_material_interference'
        if result['operation_failures']:row['functional_status']='native_calculation_uncertainty'
        report['cases'].append(row);write(args.output,report)
        print(json.dumps(dict(case=case['id'],collisions=result['collision_count'],failures=len(result['operation_failures']),status=row['functional_status'])),flush=True)
    if not args.endpoints_only and not args.core_ids:
        # Full finite approach/release is run for candidates with no endpoint
        # interference and a reached contact/cross target. Failed endpoints are
        # retained, not promoted by running a path that cannot finish.
        tested={r['id']:r for r in report['cases']}
        for path in cat['paths']:
            target=tested[path['target_case']]
            blocked=target['native']['collision_count'] or target['native']['operation_failures']
            failed=target['functional_status'] in ['surface_landmark_target_not_reached','projected_cross_target_not_reached']
            if blocked or failed:
                report['paths'].append(dict(id=path['id'],target=path['target_case'],status='endpoint_failed_path_not_run',reason=target['functional_status']))
                continue
            rows=[];points=np.array(path['waypoints_deg']);maxstep=0.
            for si,(a,b) in enumerate(zip(points[:-1],points[1:])):
                n=max(1,int(np.ceil(np.max(np.abs(b-a))/path['sample_step_deg'])))
                for k in range(n+1):
                    if si and k==0:continue
                    q=a.copy() if k==0 else b.copy() if k==n else a+(b-a)*k/n
                    if rows:maxstep=max(maxstep,float(np.max(np.abs(q-np.array(rows[-1]['native']['angles_deg'])))))
                    rows.append(dict(segment=si,fraction=k/n,native=compact(engine,q)))
            report['paths'].append(dict(id=path['id'],target=path['target_case'],status='sampled_path_with_interference' if any(r['native']['collision_count'] for r in rows) else 'sampled_path_without_reported_interference',
                observations=rows,maximum_actual_joint_step_deg=maxstep,
                calculation_failures=sum(len(r['native']['operation_failures']) for r in rows),
                reverse_tested_as_geometric_configurations=True,continuous_or_dynamic_approval=False))
            write(args.output,report)
            print(json.dumps(dict(path=path['id'],samples=len(rows),colliding_samples=sum(bool(r['native']['collision_count']) for r in rows))),flush=True)
    guard=all(sha(ROOT/p)==h for p,h in cat['input_sha256'].items()) and all(sha(ROOT/p)==h for p,h in report['input_sha256'].items())
    report.update(completed=True,input_hash_guard=guard,elapsed_s=time.monotonic()-started,
        summary=dict(endpoints=len(report['cases']),colliding_endpoints=sum(bool(r['native']['collision_count']) for r in report['cases']),
        uncertain_endpoints=sum(bool(r['native']['operation_failures']) for r in report['cases']),
        paths=len(report['paths']),paths_executed=sum('observations' in p for p in report['paths']),
        actual_clearance_micromargin_decisions=0))
    write(args.output,report)
    if not guard:raise ValueError('Inputs changed')


if __name__=='__main__':main()
