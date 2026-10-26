"""Re-fit fingertip posture using observed native root interference guidance."""
import copy
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
from scipy.interpolate import RegularGridInterpolator
from scipy.optimize import least_squares

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_refine_guide',ROOT/'cad/object_contact_v1/search.py')
g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)


def main():
    out=HERE/'generated/tip_refined.json'
    if out.exists():raise FileExistsError('Preserve previous refinement')
    search=g.read(HERE/'generated/search.json');root=g.read(HERE/'generated/root_posture_map.json')
    if not root['completed'] or not root['input_hash_guard']:raise ValueError('Incomplete root map')
    sources={p:g.sha(ROOT/p) for p in ['cad/posture_correction_v1/refine_tip.py','cad/posture_correction_v1/generated/search.json',
        'cad/posture_correction_v1/generated/root_posture_map.json','cad/object_contact_v1/search.py']}
    sources.update(search['source_sha256']);sources.update(root['source_sha256'])
    if not all(g.sha(ROOT/p)==h for p,h in sources.items()):raise ValueError('Changed guide')
    d=search['design'];anchors=search['tip']['anchors'];active=[0,1,2,3,16,17,18,19];limits=g.fk.joint_ranges(d)[active]
    interpolation=RegularGridInterpolator((root['spread_deg'],root['mcp_deg']),np.array(root['overlap_volume_mm3'])**(1/3),bounds_error=True)
    scene=g.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json');cover=next(r for r in scene['hand'] if r['name']=='Palm_shaped_front_cover')
    box=np.array(cover['bbox_mm']);box[0,2]=-35.;pairs=[]
    for a in range(15):
        for b in range(a+1,15):
            if a//3==b//3 and abs(a-b)<=1:continue
            if a//3 not in [0,4] and b//3 not in [0,4]:continue
            if (a,b)==(2,14):continue
            pairs.append([a,b])
    pairs=np.array(pairs);pref=np.array([0,45,60,30,25,50,45,30]);rng=np.random.default_rng(501)
    seeds=[np.array(search['tip']['selected'][0]['angles_deg'])[active],pref.copy()]
    for _ in range(14):seeds.append(np.array([rng.uniform(-10,15),rng.uniform(30,80),rng.uniform(35,95),rng.uniform(10,65),
        rng.uniform(-20,55),rng.uniform(20,80),rng.uniform(20,80),rng.uniform(10,70)]))
    solutions=[]
    for si,seed in enumerate(seeds):
        def residual(x):
            q=np.zeros(20);q[active]=x;e=g.evaluate(d,q,anchors,'index',0.,pairs,box);n=np.array(e['normal'])
            # The full search range remains available. The observed root map
            # supplies a smooth collision penalty, rather than clipping angles.
            root_penalty=float(interpolation([[x[0],x[1]]])[0])
            return np.r_[10*(e['centre_residual']+.25*n[0]),100*(n[0]+n[1]),4*e['capsule_residual'],4*e['palm_residual'],
                10*root_penalty,.015*(x-pref),.6*np.maximum(-x[[1,2,3,5,6,7]],0),.2*max(sum(x[1:4])-210,0)]
        fit=least_squares(residual,np.array(seed),bounds=(limits[:,0],limits[:,1]),max_nfev=400,ftol=1e-9,xtol=1e-9,gtol=1e-9)
        q=np.zeros(20);q[active]=fit.x;e=g.evaluate(d,q,anchors,'index',0.,pairs,box)
        for key in ['centre_residual','capsule_residual','palm_residual']:e.pop(key)
        e.update(tip_target_met=bool(e['object_centre_disagreement_mm']<=1 and e['outward_normal_dot']<=-.99),requested_unloaded_tip_gap_mm=.25,
            interpolated_root_penetration_guide=float(interpolation([[fit.x[0],fit.x[1]]])[0]),reverse_flexion_used=bool(np.any(fit.x[[1,2,3,5,6,7]]< -1.)))
        solutions.append(dict(seed=si,angles_deg=q.tolist(),metrics=e,preference_score=float(np.linalg.norm(fit.fun))))
    solutions.sort(key=lambda s:s['preference_score']);selected=[]
    for c in solutions:
        if all(np.linalg.norm(np.array(c['angles_deg'])-np.array(s['angles_deg']))>=4 for s in selected):selected.append(c)
        if len(selected)==3:break
    g.write(out,dict(completed=True,source_sha256=sources,all_solutions=solutions,selected=selected,
        input_hash_guard=all(g.sha(ROOT/p)==h for p,h in sources.items()),geometry_changed=False,
        interpretation='Posture correction with native interference guidance; candidates still need full hand/path and visual naturalness checks.'))
    print(json.dumps(dict(selected=selected)),flush=True)


if __name__=='__main__':main()
