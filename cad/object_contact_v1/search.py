"""Object-sized opposition search with broad whole-hand collision guidance.

Capsules and a palm box are optimizer guides, never collision certificates.
All requested joint ranges remain available. Native saved CAD decides results.
"""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time

import numpy as np
from scipy.optimize import least_squares

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_object_fk',ROOT/'cad/gesture_hand_v3/poses.py')
fk=importlib.util.module_from_spec(spec);spec.loader.exec_module(fk)
DIGITS=['index','middle','ring','little','thumb']


def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,doc):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True);temp=p.with_suffix(p.suffix+'.partial')
    temp.write_text(json.dumps(doc,ensure_ascii=False,separators=(',',':'),allow_nan=False)+'\n',encoding='utf-8');temp.replace(p)


def segment_distances(a,b,c,d):
    """Finite segment distance, including parallel/endpoint minima."""
    u=b-a;v=d-c;w=a-c
    A=np.einsum('ij,ij->i',u,u);B=np.einsum('ij,ij->i',u,v);C=np.einsum('ij,ij->i',v,v)
    D=np.einsum('ij,ij->i',u,w);E=np.einsum('ij,ij->i',v,w);den=A*C-B*B
    s=np.divide(B*E-C*D,den,out=np.zeros_like(den),where=np.abs(den)>1e-10)
    t=np.divide(A*E-B*D,den,out=np.zeros_like(den),where=np.abs(den)>1e-10)
    interior=np.linalg.norm(w+s[:,None]*u-t[:,None]*v,axis=1)
    interior=np.where((np.abs(den)>1e-10)&(s>=0)&(s<=1)&(t>=0)&(t<=1),interior,np.inf)
    def point_line(p,x,y):
        z=y-x;alpha=np.clip(np.einsum('ij,ij->i',p-x,z)/np.einsum('ij,ij->i',z,z),0,1)
        return np.linalg.norm(p-x-alpha[:,None]*z,axis=1)
    return np.minimum.reduce([interior,point_line(a,c,d),point_line(b,c,d),point_line(c,a,b),point_line(d,a,b)])


def evaluate(d,q,anchors,digit,radius,pairs,palm_box):
    frames=fk.forward_kinematics(d,q);bodies=frames['bodies'];points=[];normals=[]
    for name in [digit,'thumb']:
        anchor=anchors[name];T=bodies[anchor['body']]
        points.append(T[:3,:3]@anchor['local_point_mm']+T[:3,3]);normals.append(T[:3,:3]@anchor['local_outward_normal'])
    centres=[p+radius*n for p,n in zip(points,normals)];centre=(centres[0]+centres[1])/2
    starts=[];ends=[]
    for name in DIGITS:
        chain=frames['chains'][name];starts.extend(chain[:-1]);ends.extend(chain[1:])
    starts=np.array(starts);ends=np.array(ends)
    a,b=pairs[:,0],pairs[:,1]
    distances=segment_distances(starts[a],ends[a],starts[b],ends[b])
    radii=np.array([13.]*12+[14.]*3)
    deficits=np.maximum(radii[a]+radii[b]-distances,0.)
    # A convex palm guide rejects obvious through-palm solutions. These are
    # sampled centreline spheres on non-root links; exact CAD may differ.
    active=[DIGITS.index(digit)*3+1,DIGITS.index(digit)*3+2,13,14]
    testpoints=np.concatenate([starts[active]*.5+ends[active]*.5,ends[active]])
    lo,hi=palm_box;outside=np.maximum(np.maximum(lo-testpoints,testpoints-hi),0.)
    palm_distance=np.linalg.norm(outside,axis=1)
    palm_radii=np.array([radii[i] for i in active]*2)
    palm_deficits=np.maximum(palm_radii-palm_distance,0.)
    normal_dot=float(normals[0]@normals[1]);centre_error=float(np.linalg.norm(centres[0]-centres[1]))
    return dict(centre_mm=centre.tolist(),point_mm=[p.tolist() for p in points],normal=[n.tolist() for n in normals],
        outward_normal_dot=normal_dot,object_centre_disagreement_mm=centre_error,
        maximum_capsule_overlap_guide_mm=float(np.max(deficits)),maximum_palm_overlap_guide_mm=float(np.max(palm_deficits)),
        centre_residual=centres[0]-centres[1],capsule_residual=deficits,palm_residual=palm_deficits)


def make():
    started=time.monotonic();cat=read(ROOT/'cad/functional_motion_v3/generated/catalog.json')
    scene=read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json');d=cat['design'];anchors=cat['pad_anchors']
    for p,h in cat['input_sha256'].items():
        if sha(ROOT/p)!=h:raise ValueError('Stale baseline '+p)
    cover=next(p for p in scene['hand'] if p['name']=='Palm_shaped_front_cover')
    # Use a broad engineering approximation derived from the saved cover box.
    # It only guides search, and cannot turn a missed object contact into pass.
    box=np.array(cover['bbox_mm'],float);box[0,2]=min(box[0,2],-35.)
    rng=np.random.default_rng(20261026);results=[]
    report=dict(completed=False,schema='object_contact_v1.search.1',baseline='cad/functional_motion_v3',
        source_sha256={**cat['input_sha256'],'cad/functional_motion_v3/generated/catalog.json':sha(ROOT/'cad/functional_motion_v3/generated/catalog.json'),
            'cad/object_contact_v1/search.py':sha(__file__)},design=d,pad_anchors=anchors,goals=[],
        proxy_note='13/14mm link capsules and convex palm box guide only; exclude adjacent joints only from guide, never from final native audit. Finite local optimization is not an infeasibility proof.',
        engineering_screen=dict(maximum_object_centre_error_mm=1.,maximum_normal_dot=-.8,maximum_proxy_penetration_mm=.2),
        no_physical_grasp_approval=True,hardware_validated=False)
    for digit,diameter in [('index',8.),('index',25.),('middle',8.),('ring',8.),('little',8.)]:
        fi=DIGITS.index(digit);active=list(range(fi*4,fi*4+4))+list(range(16,20));limits=fk.joint_ranges(d)[active]
        pairs=[]
        for a in range(15):
            for b in range(a+1,15):
                if a//3==b//3 and abs(a-b)<=1:continue
                if a//3 not in [fi,4] and b//3 not in [fi,4]:continue
                pairs.append([a,b])
        pairs=np.array(pairs);nseeds=12 if digit=='index' else 8
        previous=np.array(next(c for c in cat['cases'] if c['id']=='pinch_'+digit)['angles_deg'])[active]
        seeds=[previous,[0,35,55,25,0,45,30,35],[10,50,60,30,30,70,45,20],[-10,25,40,35,-35,20,50,45]]
        for _ in range(nseeds-len(seeds)):
            seeds.append([rng.uniform(-20,20),rng.uniform(10,85),rng.uniform(20,110),rng.uniform(0,80),
                          rng.uniform(-65,55),rng.uniform(0,85),rng.uniform(0,80),rng.uniform(0,85)])
        solutions=[]
        for si,seed in enumerate(seeds):
            seed=np.array(seed,float)
            if np.any(seed<limits[:,0]) or np.any(seed>limits[:,1]):raise ValueError('Seed outside declared exploration; no clipping')
            def residual(x):
                q=np.zeros(20);q[active]=x;e=evaluate(d,q,anchors,digit,diameter/2,pairs,box)
                # The normal term is a one-sided opposing-contact requirement,
                # allowing harmless orientation freedom below the target dot.
                return np.r_[e['centre_residual'],20*max(e['outward_normal_dot']+.85,0),
                    4*e['capsule_residual'],4*e['palm_residual'],.03*max(sum(x[1:4])-225,0),.001*(x-np.array([0,40,55,30,0,45,30,35]))]
            fit=least_squares(residual,seed,bounds=(limits[:,0],limits[:,1]),max_nfev=220,ftol=2e-7,xtol=2e-7,gtol=2e-7)
            q=np.zeros(20);q[active]=fit.x;e=evaluate(d,q,anchors,digit,diameter/2,pairs,box)
            for k in ['centre_residual','capsule_residual','palm_residual']:e.pop(k)
            e['guide_target_met']=bool(e['object_centre_disagreement_mm']<=1 and e['outward_normal_dot']<=-.8 and max(e['maximum_capsule_overlap_guide_mm'],e['maximum_palm_overlap_guide_mm'])<=.2)
            solutions.append(dict(seed=si,angles_deg=q.tolist(),metrics=e,objective_norm=float(np.linalg.norm(fit.fun)),
                optimizer_success=bool(fit.success),evaluations=int(fit.nfev)))
            print(json.dumps(dict(goal=digit+'_'+str(diameter),seed=si,objective=solutions[-1]['objective_norm'],metrics=e)),flush=True)
        solutions.sort(key=lambda s:s['objective_norm'])
        report['goals'].append(dict(id=digit+'_sphere_'+format(diameter,'g'),digit=digit,object=dict(shape='sphere',diameter_mm=diameter),
            goal='Opposing real saved tip-cap landmarks, sized sphere between them, no whole-hand/object material penetration',
            solutions=solutions,selected_seeds=[s['seed'] for s in solutions[:3]],physical_hold_tested=False))
        write(HERE/'generated/search.json',report)
    report.update(completed=True,input_hash_guard=all(sha(ROOT/p)==h for p,h in report['source_sha256'].items()),elapsed_s=time.monotonic()-started)
    write(HERE/'generated/search.json',report)
    if not report['input_hash_guard']:raise ValueError('Inputs changed')


if __name__=='__main__':
    if (HERE/'generated/search.json').exists():raise FileExistsError('Keep earlier search evidence')
    make()
