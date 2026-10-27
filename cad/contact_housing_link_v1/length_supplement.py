"""Independent length integration split at cubic knots, preserving solver values."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import contact_solver as s
np,p,ROOT,HERE=s.np,s.p,s.ROOT,s.HERE
from scipy.integrate import quad
def length(C):return sum(quad(lambda t:float(np.linalg.norm(s.SPL.derivative()(t)@C)),a,b,epsabs=1e-10,epsrel=1e-12,limit=100)[0] for a,b in zip([0.,.2,.4,.6,.8],[.2,.4,.6,.8,1.]))
def main():
    r=p.read(HERE/'reviews/check.json');old=r['solver'];C=np.asarray(old['controls']);initial=np.asarray(old['initial_controls']);actual=[length(z) for z in C];seed=[length(z) for z in initial];records=[]
    for i in [0,1]:
        total=sum(actual[2*i:2*i+2]);records.append(dict(line=i,saved_solver_free_length_target_mm=old['new_free_length_targets_mm'][i],saved_solver_Gauss64_length_residual_mm=old['length_residual_mm'][i],independent_split_quad_free_length_mm=total,independent_residual_to_same_saved_target_mm=total-old['new_free_length_targets_mm'][i],initial_curve_split_quad_length_mm=sum(seed[2*i:2*i+2]),saved_solver_selected_total_minus_independent_mm=sum(old['free_span_lengths_mm'][2*i:2*i+2])-total))
    p.write(HERE/'reviews/length_supplement.json',dict(completed=True,quad_epsabs=1e-10,quad_epsrel=1e-12,intervals=[[0,.2],[.2,.4],[.4,.6],[.6,.8],[.8,1]],selected_free_span_lengths_mm=actual,initial_free_span_lengths_mm=seed,physical_lines=records,original_solver_targets_residuals_and_failure_preserved=True,optimizer_native_or_geometry_changes=False));print(records,flush=True)
if __name__=='__main__':main()
