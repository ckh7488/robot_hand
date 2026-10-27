"""Small independent check of an immovable roundoff-negative SLSQP row."""
import json
from pathlib import Path
import numpy as np
from scipy.optimize import minimize
results=[]
for kind in ['linear','quadratic']:
    for fixed in [-3.255173908200959e-14,0.,1.]:
        eq=(lambda x:x[0]-1) if kind=='linear' else (lambda x:x[0]**2-1)
        jac=(lambda x:np.array([1.])) if kind=='linear' else (lambda x:np.array([2*x[0]]))
        opt=minimize(lambda x:(x[0]-6)**2,np.array([5.]),jac=lambda x:np.array([2*(x[0]-6)]),bounds=[(1e-3,None)],method='SLSQP',constraints=[dict(type='eq',fun=eq,jac=jac),dict(type='ineq',fun=lambda x:fixed,jac=lambda x:np.array([0.]))],options=dict(maxiter=80,ftol=1e-9))
        results.append(dict(kind=kind,fixed_inequality=fixed,success=bool(opt.success),message=str(opt.message),x=float(opt.x[0]),eq_residual=float(eq(opt.x)),iterations=int(opt.nit)))
out=Path(__file__).resolve().parent/'reviews/numerical_probe.json';out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(dict(completed=True,cases=results,not_proof_of_full_route_cause=True),indent=2)+'\n',encoding='utf-8');print(json.dumps(results),flush=True)
