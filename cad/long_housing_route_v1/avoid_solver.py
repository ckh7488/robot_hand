"""Same fixed-length curve family with conservative boxes of actual blockers."""
from route_solver import *
import route_solver as original
TS=np.linspace(.015,.985,41);BS=bernstein(5,TS)
def solve_avoiding(q,i,boxes,previous=None):
    e=endpoints(q,i);A,B,tA,tB=[e[k] for k in ['A','B','tA','tB']];boxes=np.asarray(boxes);lo=boxes[:,0];hi=boxes[:,1];center=(lo+hi)/2;half=(hi-lo)/2
    def controls(x):return np.asarray([A,A+tA*x[0],x[2:5],x[5:8],B-tB*x[1],B])
    def separation(x):
        pts=BS@controls(x);d=np.abs(pts[:,None,:]-center)-half;signed=np.linalg.norm(np.maximum(d,0),axis=2)+np.minimum(np.max(d,axis=2),0);return (signed-1.5).ravel()
    seed=np.r_[90.,70.,A+tA*170+[150.,0,0],B-tB*140+[150.,0,0]]
    if previous is not None:seed=np.r_[np.linalg.norm(previous[1]-previous[0]),np.linalg.norm(previous[5]-previous[4]),previous[2],previous[3]]
    result=minimize(lambda x:properties(controls(x))[1]*FLEX_LENGTH,seed,method='SLSQP',bounds=[(10.,240.),(10.,240.)]+[(min(A[k%3],B[k%3])-400,max(A[k%3],B[k%3])+400) for k in range(6)],constraints=[dict(type='eq',fun=lambda x:properties(controls(x))[0]-FLEX_LENGTH),dict(type='ineq',fun=separation)],options=dict(ftol=1e-9,maxiter=500))
    C=controls(result.x);length,bend,kappa=properties(C);D=5*np.diff(C,axis=0);independent=quad(lambda t:float(np.linalg.norm(bernstein(4,np.asarray([t]))[0]@D)),0.,1.,epsabs=1e-8,epsrel=1e-11,limit=200)[0];success=bool(result.success and abs(independent-FLEX_LENGTH)<1e-5 and np.min(separation(result.x))>=-1e-5)
    return dict(cord=i,controls=C.tolist(),solver_success=success,solver_message=str(result.message),iterations=int(result.nit),family='same_quintic_Bezier_with41sample_conservative_material_box_constraints',flex_length_candidate_mm=FLEX_LENGTH,quadrature_length_mm=length,independent_quad_length_mm=independent,bending_objective_1_per_mm=bend,sampled_minimum_curvature_radius_mm=1/kappa if kappa>1e-12 else None,minimum_sampled_box_separation_minus_housing_radius_mm=float(np.min(separation(result.x))),anchors={k:np.asarray(v).tolist() for k,v in e.items()},physical_elastic_equilibrium_verified=False,midspan_guides_installed=False)
