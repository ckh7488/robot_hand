"""Keep CAD fixed; coordinate both crossing fingers and use real distal tips.

Mesh/capsule results guide candidate selection only. Native whole-hand checks
remain mandatory. Preference ranges are engineering choices, not anatomy laws.
"""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time

import numpy as np
from scipy.optimize import least_squares
import trimesh

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True


def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m


guide=module('_posture_contact_guide',ROOT/'cad/object_contact_v1/search.py')
fk=guide.fk
read=guide.read
write=guide.write
sha=guide.sha


def projected_crossings(d,q):
    chains=fk.forward_kinematics(d,q)['chains'];out=[]
    for i in [1,2]:
        for j in [1,2]:
            a,b=chains['index'][i:i+2];c,e=chains['middle'][j:j+2]
            A=np.stack([(b-a)[:2],-(e-c)[:2]],axis=1)
            if abs(np.linalg.det(A))<1e-8:continue
            u,v=np.linalg.solve(A,(c-a)[:2])
            if 0<u<1 and 0<v<1:
                p=a+u*(b-a);r=c+v*(e-c)
                out.append(dict(index_segment=i,middle_segment=j,xy_mm=p[:2].tolist(),signed_depth_mm=float(p[2]-r[2])))
    return out


def cross_search(d,scene,cat):
    neutral=fk.forward_kinematics(d,np.zeros(20))['bodies'];managers={};rows={}
    for digit in ['index','middle']:
        managers[digit]=trimesh.collision.CollisionManager()
        rows[digit]=[r for r in scene['hand'] if r['name'].startswith(digit+'_')]
        for r in rows[digit]:managers[digit].add_object(r['name'],trimesh.load_mesh(ROOT/r['mesh'],process=False))
    base=np.array(next(c for c in cat['cases'] if c['id']=='cross_old_reference')['angles_deg'])
    result=[]
    for front in ['middle','index']:
        fi=1 if front=='middle' else 0;back=1-fi;tested=0;projected=0;candidates=[]
        for mcp in [26.,28.,30.,32.]:
            for pip in [30.,35.,40.,45.,50.,55.,60.,65.]:
                for dip in [0.,10.,20.]:
                    for frontpip in [0.,5.,10.]:
                        for spread in [16.,18.,20.]:
                            q=base.copy();q[:8]=0.;q[fi*4+1]=mcp;q[fi*4+2]=frontpip
                            q[back*4+2:back*4+4]=[pip,dip]
                            q[fi*4]=spread if fi==1 else -spread
                            tested+=1;cross=projected_crossings(d,q)
                            if not cross:continue
                            projected+=1;frames=fk.forward_kinematics(d,q)['bodies']
                            for digit in ['index','middle']:
                                for r in rows[digit]:managers[digit].set_transform(r['name'],frames[r['body']]@np.linalg.inv(neutral[r['body']]))
                            hit=managers['index'].in_collision_other(managers['middle'])
                            if hit:continue
                            gap,names=managers['index'].min_distance_other(managers['middle'],return_names=True)
                            if not .05<=gap<=4.:continue
                            # Prefer slight joint adjustments with positive flexion,
                            # avoiding the former forced one-straight/one-reverse pose.
                            depth=min(abs(c['signed_depth_mm']) for c in cross)
                            score=abs(gap-1.)+.015*(mcp-28)**2+.008*(pip-45)**2+.005*dip**2+.01*frontpip**2+.015*(spread-18)**2+.5*max(depth-32,0)+.02*abs(depth-28)
                            candidates.append(dict(angles_deg=q.tolist(),crossings=cross,mesh_cross_digit_gap_mm=float(gap),nearest_mesh_pair=list(names),
                                preference_score=float(score),front_digit=front,
                                interpretation='Finite mesh guide only; terminal/contact mates are not exempted from native final checks'))
        candidates.sort(key=lambda r:r['preference_score'])
        # Keep geometrically different alternatives, rather than nearly identical seeds.
        selected=[]
        for c in candidates:
            if all(np.linalg.norm(np.array(c['angles_deg'])[:8]-np.array(s['angles_deg'])[:8])>=6 for s in selected):selected.append(c)
            if len(selected)==3:break
        result.append(dict(id='cross_front_'+front,grid_cases=tested,projected_cross_cases=projected,mesh_candidates=len(candidates),selected=selected))
        print(json.dumps(dict(cross=front,tested=tested,candidates=len(candidates),selected=selected)),flush=True)
    return result


def tip_search(d,scene,cat):
    anchors=copy.deepcopy(cat['pad_anchors']);neutral=fk.forward_kinematics(d,np.zeros(20))['bodies']
    for digit,i in [('index',0),('thumb',4)]:
        row=next(r for r in scene['hand'] if r['name']==digit+'_segment3_rounded_tip_cap')
        mesh=trimesh.load_mesh(ROOT/row['mesh'],process=False);mesh.vertices=trimesh.transform_points(mesh.vertices,np.linalg.inv(neutral[row['body']]))
        length=(d['fingers']+[d['thumb']])[i]['links'][2]
        point,distance,face=trimesh.proximity.closest_point_naive(mesh,[[0.,length,0.]])
        anchors[digit].update(local_point_mm=point[0].tolist(),local_outward_normal=mesh.face_normals[face[0]].tolist(),
            target_to_actual_surface_mm=float(distance[0]),role='actual distal front-cap surface; tip-to-tip target, distinct from palmar-pad target')
    active=[0,1,2,3,16,17,18,19];limits=fk.joint_ranges(d)[active];pairs=[]
    for a in range(15):
        for b in range(a+1,15):
            if a//3==b//3 and abs(a-b)<=1:continue
            if a//3 not in [0,4] and b//3 not in [0,4]:continue
            if (a,b)==(2,14):continue
            pairs.append([a,b])
    pairs=np.array(pairs);cover=next(r for r in scene['hand'] if r['name']=='Palm_shaped_front_cover');box=np.array(cover['bbox_mm']);box[0,2]=-35.
    rng=np.random.default_rng(20261027);seeds=[[0,40,55,25,0,45,35,25],[8,30,65,35,10,60,20,30],
        [-8,50,40,20,-20,40,50,40],[0,60,75,35,30,65,45,35]]
    for _ in range(12):seeds.append([rng.uniform(-15,15),rng.uniform(10,75),rng.uniform(20,90),rng.uniform(0,60),
        rng.uniform(-50,45),rng.uniform(5,80),rng.uniform(5,75),rng.uniform(0,70)])
    solutions=[];pref=np.array([0,40,55,25,0,45,35,25])
    for si,seed in enumerate(seeds):
        def residual(x):
            q=np.zeros(20);q[active]=x;e=guide.evaluate(d,q,anchors,'index',0.,pairs,box)
            n=np.array(e['normal'])
            # Near-touch unloaded posture, not a claim of object contact/hold.
            return np.r_[10*(e['centre_residual']+.25*n[0]),100*(n[0]+n[1]),
                4*e['capsule_residual'],4*e['palm_residual'],.015*(x-pref),
                .6*np.maximum(-x[[1,2,3,5,6,7]],0),.2*max(sum(x[1:4])-210,0),.15*max(abs(x[0])-18,0)]
        fit=least_squares(residual,np.array(seed),bounds=(limits[:,0],limits[:,1]),max_nfev=250,ftol=1e-8,xtol=1e-8,gtol=1e-8)
        q=np.zeros(20);q[active]=fit.x;e=guide.evaluate(d,q,anchors,'index',0.,pairs,box)
        for key in ['centre_residual','capsule_residual','palm_residual']:e.pop(key)
        e['requested_unloaded_tip_gap_mm']=.25
        e['tip_target_met']=bool(e['object_centre_disagreement_mm']<=1 and e['outward_normal_dot']<=-.99)
        e['reverse_flexion_used']=bool(np.any(fit.x[[1,2,3,5,6,7]]< -1.))
        solutions.append(dict(seed=si,angles_deg=q.tolist(),metrics=e,preference_score=float(np.linalg.norm(fit.fun))))
    solutions.sort(key=lambda s:s['preference_score']);selected=[]
    for c in solutions:
        if all(np.linalg.norm(np.array(c['angles_deg'])-np.array(s['angles_deg']))>=5 for s in selected):selected.append(c)
        if len(selected)==3:break
    print(json.dumps(dict(tip_selected=selected)),flush=True)
    return dict(id='index_thumb_distal_contact',anchors=anchors,all_solutions=solutions,selected=selected,
        proxy_note='Intended terminal-to-terminal pair omitted only from capsule guide because point tips coincide; all actual materials including those caps remain in final native scope')


def main():
    output=HERE/'generated/search.json'
    if output.exists():raise FileExistsError('Preserve old search')
    start=time.monotonic();cat=read(ROOT/'cad/functional_motion_v3/generated/catalog.json');scene=read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')
    if not all(sha(ROOT/p)==h for p,h in cat['input_sha256'].items()):raise ValueError('Changed baseline')
    paths=['cad/posture_correction_v1/search.py','cad/object_contact_v1/search.py','cad/functional_motion_v3/generated/catalog.json']
    hashes={**cat['input_sha256'],**{p:sha(ROOT/p) for p in paths}}
    result=dict(completed=False,source_sha256=hashes,design=cat['design'],cross=[],tip=None,
        geometry_changed=False,axes_changed=False,lengths_changed=False,
        human_form_policy='Coordinate ordinary-looking positive flexion and preserve accepted proportions. Numerical preferences are engineering choices; rendering and whole-hand path checks still required.',
        interpretation='One required candidate colliding does not establish a CAD defect. No physical motor travel, continuous clearance or hardware approval.')
    result['cross']=cross_search(cat['design'],scene,cat);write(output,result)
    result['tip']=tip_search(cat['design'],scene,cat)
    result.update(completed=True,input_hash_guard=all(sha(ROOT/p)==h for p,h in hashes.items()),elapsed_s=time.monotonic()-start)
    write(output,result)
    if not result['input_hash_guard']:raise ValueError('Inputs changed')


if __name__=='__main__':main()
