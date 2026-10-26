"""Native whole-hand and explicitly sized-object tests of search candidates."""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import sys
import time

import cadquery as cq
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_contact_native',ROOT/'cad/functional_motion_v3/audit.py')
prior=importlib.util.module_from_spec(spec);spec.loader.exec_module(prior)


class ObjectAudit:
    def __init__(self):
        native,self.v,self.cat,self.scene,self.parts=prior.load()
        self.engine=native.PairScreen(self.v,self.parts,self.scene['hand'],self.cat['design'],0.,True)
        self.bodies={r['name']:r['body'] for r in self.scene['hand']}

    def evaluate(self,q,centre,radius,object_shape=None):
        hand=prior.compact(self.engine,q);frames=self.v.frames(self.cat['design'],q)
        obj=object_shape or cq.Solid.makeSphere(radius,cq.Vector(*centre),angleDegrees1=-90,angleDegrees2=90,angleDegrees3=360)
        obox=np.array(self.v.bounds(obj));collisions=[];uncertainties=[];target_shapes={}
        moved={}
        for name,shape in self.parts.items():
            current=self.v.relocate(shape,frames[self.bodies[name]],self.engine.neutral[self.bodies[name]])
            moved[name]=current;box=np.array(self.v.bounds(current))
            overlap=np.minimum(box[1],obox[1])-np.maximum(box[0],obox[0])
            if np.all(overlap>0):
                try:
                    volume=float(current.intersect(obj,tol=1e-6).Volume())
                    if not math.isfinite(volume) or volume< -1e-9:raise ValueError('Invalid common volume')
                    if volume>.02:collisions.append(dict(part=name,material_intersection_mm3=volume))
                except Exception as e:uncertainties.append(dict(part=name,error=repr(e)))
        return dict(hand=hand,object_centre_mm=list(map(float,centre)),object_radius_mm=float(radius),
            object_collisions=collisions,object_operation_failures=uncertainties,object_pairs_in_scope=len(self.parts)),moved,obj


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);ap.add_argument('--core',nargs='*');a=ap.parse_args()
    if a.output.exists():raise FileExistsError('Keep earlier evidence')
    start=time.monotonic();search=prior.read(HERE/'generated/search.json')
    if not search['completed'] or not search['input_hash_guard']:raise ValueError('Incomplete search')
    source=dict(search['source_sha256'])
    source.update({p:prior.sha(ROOT/p) for p in ['cad/object_contact_v1/audit.py','cad/object_contact_v1/generated/search.json',
        'cad/functional_motion_v3/audit.py','cad/gesture_hand_v3/audit_gestures.py','cad/articulation_drive_v2/verify.py']})
    if not all(prior.sha(ROOT/p)==h for p,h in source.items()):raise ValueError('Stale input')
    audit=ObjectAudit();report=dict(completed=False,schema='object_contact_v1.native.1',source_sha256=source,
        hand_parts=364,self_pairs_per_pose=66066,object_pairs_per_pose=364,
        reporting_floor_mm3=.02,boolean_tolerance_mm=1e-6,contact_gap_screen_mm=.5,
        cases=[],paths=[],geometry_changed=False,whole_hand_motion_approved=False,physical_hold_validated=False)
    candidates=[]
    for goal in search['goals']:
        for s in goal['solutions'][:3]:candidates.append((goal,s))
    if a.core:candidates=[(g,s) for g,s in candidates if g['id']+'_seed'+str(s['seed']) in a.core]
    for goal,s in candidates:
        ident=goal['id']+'_seed'+str(s['seed']);centre=s['metrics']['centre_mm'];radius=goal['object']['diameter_mm']/2
        path=HERE/'generated/objects'/(ident+'.step');path.parent.mkdir(parents=True,exist_ok=True)
        obj=cq.Solid.makeSphere(radius,cq.Vector(*centre),angleDegrees1=-90,angleDegrees2=90,angleDegrees3=360)
        if not path.exists():cq.exporters.export(obj,str(path))
        obj=cq.importers.importStep(str(path)).val()
        if not obj.isValid() or len(obj.Solids())!=1 or abs(obj.Volume()-4/3*math.pi*radius**3)>1e-5:
            raise ValueError('Saved sphere invalid or changed size')
        if np.linalg.norm(np.array(obj.Center().toTuple())-centre)>1e-6:raise ValueError('Saved sphere moved')
        got,moved,obj=audit.evaluate(s['angles_deg'],centre,radius,obj);contacts=[]
        for digit in [goal['digit'],'thumb']:
            name=search['pad_anchors'][digit]['part']
            contacts.append(dict(digit=digit,part=name,gap_mm=float(moved[name].distance(obj))))
        status='nominal_two_surface_contact_candidate'
        m=s['metrics']
        if m['object_centre_disagreement_mm']>1 or m['outward_normal_dot']>-.8 or any(c['gap_mm']>.5 for c in contacts):status='object_contact_target_not_reached'
        if got['object_collisions']:status='actual_object_material_interference'
        if got['hand']['collision_count']:status='actual_whole_hand_interference'
        if got['hand']['operation_failures'] or got['object_operation_failures']:status='native_calculation_uncertainty'
        row=dict(id=ident,goal=goal['id'],digit=goal['digit'],angles_deg=s['angles_deg'],metrics=m,native=got,contacts=contacts,
            object_step=path.relative_to(ROOT).as_posix(),object_sha256=prior.sha(path),status=status)
        report['cases'].append(row);prior.write(a.output,report)
        print(json.dumps(dict(case=ident,status=status,hand_pairs=got['hand']['collision_count'],object_pairs=len(got['object_collisions']),contacts=contacts)),flush=True)
    if not a.core:
        # Only actual clear, reached endpoints receive an approach/withdrawal.
        # The object stays outside while the hand pre-shapes, then enters in5mm
        # increments. Joint increments stay<=5deg. Finite configurations only.
        for goal in search['goals']:
            eligible=[r for r in report['cases'] if r['goal']==goal['id'] and r['status']=='nominal_two_surface_contact_candidate']
            if not eligible:
                report['paths'].append(dict(goal=goal['id'],status='all_tested_endpoints_failed_path_not_run'));continue
            selected=eligible[0];target=np.array(selected['angles_deg']);centre=np.array(selected['native']['object_centre_mm']);radius=selected['native']['object_radius_mm']
            far=centre+np.array([0.,0.,120.]);pre=target*.7;zero=np.zeros(20)
            states=[(zero,far),(pre,far),(pre,centre),(target,centre),(pre,centre),(pre,far),(zero,far)]
            observations=[]
            for si,((qa,pa),(qb,pb)) in enumerate(zip(states[:-1],states[1:])):
                n=max(1,int(np.ceil(max(np.max(np.abs(qb-qa))/5,np.linalg.norm(pb-pa)/5))))
                for k in range(n+1):
                    if si and not k:continue
                    q=qa.copy() if not k else qb.copy() if k==n else qa+(qb-qa)*k/n
                    pos=pa.copy() if not k else pb.copy() if k==n else pa+(pb-pa)*k/n
                    got,_,_=audit.evaluate(q,pos,radius)
                    observations.append(dict(segment=si,fraction=k/n,native=got))
            failures=sum(bool(o['native']['hand']['operation_failures'] or o['native']['object_operation_failures']) for o in observations)
            collisions=sum(bool(o['native']['hand']['collision_count'] or o['native']['object_collisions']) for o in observations)
            report['paths'].append(dict(goal=goal['id'],target=selected['id'],status='sampled_path_clear' if not failures and not collisions else 'sampled_path_failed',
                method='Pre-shape with object120mm above target, insert object along worldZ, close contact, open and withdraw, geometric reverse',
                max_joint_increment_deg=5,max_object_increment_mm=5,observations=observations,colliding_samples=collisions,uncertain_samples=failures))
            prior.write(a.output,report)
            print(json.dumps(dict(path=goal['id'],samples=len(observations),colliding_samples=collisions)),flush=True)
    report.update(completed=True,input_hash_guard=all(prior.sha(ROOT/p)==h for p,h in source.items()),elapsed_s=time.monotonic()-start)
    prior.write(a.output,report)
    if not report['input_hash_guard']:raise ValueError('Inputs changed')


if __name__=='__main__':main()
