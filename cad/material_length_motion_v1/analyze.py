"""Finite parent approach/release and wrist references; no contact optimization."""
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import material_model as a
p,np,ROOT,HERE=a.p,a.np,a.ROOT,a.HERE
def main():
    out=HERE/'reviews/analysis.json';assert not out.exists();start=time.monotonic();target=np.asarray(next(z for z in p.read(ROOT/'cad/thumb_guide_assembly_v2/reviews/static.json')['cases'] if z['id']=='parent_full')['angles_deg']);cases=[]
    for j,(factor,flex,yaw) in enumerate([(0,0,0),(.25,0,0),(.5,0,0),(.75,0,0),(1,0,0),(.75,0,0),(.5,0,0),(.25,0,0),(0,0,0),(0,-20,-15),(0,20,15)]):
        q=target*factor;solution=a.curve_solution(q,flex,yaw);case=dict(id=j,factor=factor,angles_deg=q.tolist(),wrist_flex_deg=flex,wrist_deviation_deg=yaw,solution=solution,curvature_checks=[a.ext.extrema(C) for C in np.asarray(solution['controls'])]);case['local_radius_necessary_condition']=all(z['necessary_radius_greater_than_housing_outer_radius_1p5'] for z in case['curvature_checks']);case['material_markers']=[a.material_points(case,i,np.linspace(0,1,9)) for i in [0,1]];cases.append(case);a.save(out,dict(completed=False,cases=cases));print(j,solution['closed'],case['local_radius_necessary_condition'],[(z['prescribed_loop_bulge_mm'],z['guide_material_shift_from_neutral_mm']) for z in solution['lines']],flush=True)
    seed=a.read_seed();C0=np.asarray(seed['solver']['controls']);assert np.max(abs(np.asarray(cases[0]['solution']['controls'])-C0))<1e-8
    report=dict(completed=True,cases=cases,neutral_controls_exactly_reused=True,new_candidate_length_basis='CONTACT036 actual neutral saved centerline lengths, independently split-knot integrated;not a repair/pass of its earlier preset target.',original_CONTACT036_candidate_admissible=seed['candidate_admissible'],original_CONTACT036_length_residual_mm=seed['solver']['length_residual_mm'],constant_length_closed=all(z['solution']['closed'] for z in cases),all_local_radius_necessary_conditions=all(z['local_radius_necessary_condition'] for z in cases),native_motion_tested=False,calibrated_physics_preload_tension_or_installation_verified=False,prescribed_transport_not_actual_elastic_equilibrium=True,source_sha256={f:a.m.drive.sha(ROOT/f) for f in ['cad/material_length_motion_v1/material_model.py','cad/material_length_motion_v1/analyze.py','cad/contact_housing_link_v1/reviews/check.json','cad/contact_housing_link_v1/reviews/length_supplement.json','cad/thumb_guide_assembly_v2/reviews/static.json','cad/modular_roots_v9/design.json']},elapsed_s=time.monotonic()-start)
    a.save(out,report)
if __name__=='__main__':main()
