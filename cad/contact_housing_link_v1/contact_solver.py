"""Coupled four-span contact-constrained bending candidate at real supports.

Normalized bending only. No calibrated EI, gravity, torsion, friction or tension.
"""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'cad/thumb_entry_bend_v1'))
import check_entry as prior
a,m,g,c,p,np,cq=prior.a,prior.m,prior.g,prior.c,prior.p,prior.np,prior.cq
from scipy.interpolate import BSpline
from scipy.optimize import minimize
from scipy.optimize._numdiff import approx_derivative
from material_field import Field
KNOT=np.r_[np.zeros(4),[.2,.4,.6,.8],np.ones(4)];SPL=BSpline(KNOT,np.eye(8),3);T=np.linspace(0,1,41);B=SPL(T);D=SPL.derivative()(T);DD=SPL.derivative(2)(T);GT,GW=np.polynomial.legendre.leggauss(64);GT=(GT+1)/2;GW=GW/2;GB=SPL(GT);GD=SPL.derivative()(GT);GDD=SPL.derivative(2)(GT)

def anchors(q):
    result=[]
    for i in [0,1]:
        e=m.endpoints(q,i);U=np.array([90. if i==0 else 110.,a.prior.a.REAR_Y,[-3.,2.][i]]);V=np.array([100.,-4.,[-3.,2.][i]])
        if i==0:W,t=e['B'],e['tB']
        else:
            v=g.g.v;O0,R0=v.frames(g.DESIGN,np.zeros(20))['thumb_mcp'];O,R=v.frames(g.DESIGN,q)['thumb_mcp'];F=np.eye(4);F[:3,:3]=R@R0.T;F[:3,3]=O-F[:3,:3]@O0;Q=F@a.frame();W=(Q@np.array([32.,-31.,0.,1.]))[:3];t=-Q[:3,0]
        result.append(dict(line=i,up_A=e['A'],up_B=U,up_tA=e['tA'],up_tB=np.array([0.,1.,0.]),down_A=V,down_B=W,down_tA=np.array([0.,1.,0.]),down_tB=t,terminal=e))
    return result

def setup(q):
    aa=anchors(q);bases=[];maps=[];x=[];bounds=[]
    for i in [0,1]:
        for side in ['up','down']:
            e=aa[i];A,Z,tA,tB=[e[side+'_'+k] for k in ['A','B','tA','tB']];C=np.zeros((8,3));C[0]=A;C[1]=A;C[6]=Z;C[7]=Z;J=np.zeros((8,3,14));J[1,:,0]=tA;J[6,:,1]=-tB
            for j in range(4):J[j+2,:,2+3*j:5+3*j]=np.eye(3)
            if side=='up':seed=np.array([[70+20*i,-280,-250],[130+20*i,-280,-240],[150+20*i,-250,-150],[125+20*i,-240,-10.]]) ;handles=[30.,30.]
            elif i==0:seed=np.array([[103,13,-3],[108,18,0],[106,25,9],[90,25,17.]]) ;handles=[10.,6.]
            else:seed=np.array([[100,12,15],[98,14,30],[88,0,30],[82,-2,32.]]) ;handles=[10.,8.]
            x.extend([*handles,*seed.ravel()]);bases.append(C);maps.append(J);bounds.extend([(1e-3,None),(1e-3,None)]+[(None,None)]*12)
    return aa,np.array(bases),np.array(maps),np.array(x),bounds

def fixed_points(aa):
    points=[];segments=[]
    for i in [0,1]:
        e=aa[i]['terminal'];groups=[np.array([e['motor_seat']+(e['A']-e['motor_seat'])*t for t in np.linspace(0,1,8)])]
        for d in a.prior.a.definitions(i):groups.append(np.array([a.prior.a.edge(d).positionAt(float(t)).toTuple() for t in np.linspace(0,1,9)]))
        if i==1:
            for ed in a.edges():groups.append(np.array([a.placed(cq.Vertex.makeVertex(*ed.positionAt(float(t)).toTuple())).Center().toTuple() for t in np.linspace(0,1,9)]))
        groups.append(np.array([e['B']+(e['distal_seat']-e['B'])*t for t in np.linspace(0,1,9)]));z=[];ss=[]
        for group in groups:
            start=len(z);z.extend(group);ss.extend([(j,j+1) for j in range(start,start+len(group)-1)])
        points.append(np.array(z));segments.append(np.array(ss,dtype=int))
    return points,segments

def segment_pair(A,B):
    u=A[:,1]-A[:,0];v=B[:,1]-B[:,0];w=A[:,0,None,:]-B[None,:,0,:];aa=np.sum(u*u,axis=1)[:,None];bb=np.einsum('ik,jk->ij',u,v);cc=np.sum(v*v,axis=1)[None,:];dd=np.einsum('ijk,ik->ij',w,u);ee=np.einsum('ijk,jk->ij',w,v);den=aa*cc-bb*bb;ss=(bb*ee-cc*dd)/np.where(abs(den)>1e-20,den,1);tt=(aa*ee-bb*dd)/np.where(abs(den)>1e-20,den,1);valid=(abs(den)>1e-20)&(ss>=0)&(ss<=1)&(tt>=0)&(tt<=1);shape=ss.shape
    choices=[(np.zeros(shape),np.clip(ee/np.maximum(cc,1e-20),0,1)),(np.ones(shape),np.clip((bb+ee)/np.maximum(cc,1e-20),0,1)),(np.clip(-dd/np.maximum(aa,1e-20),0,1),np.zeros(shape)),(np.clip((bb-dd)/np.maximum(aa,1e-20),0,1),np.ones(shape)),(ss,tt)]
    distances=[];vectors=[]
    for j,(s,t) in enumerate(choices):
        delta=w+s[:,:,None]*u[:,None,:]-t[:,:,None]*v[None,:,:];distance=np.linalg.norm(delta,axis=2);distances.append(np.where(valid,distance,np.inf) if j==4 else distance);vectors.append(delta)
    ds=np.stack(distances,axis=2);which=np.argmin(ds,axis=2);distance=np.min(ds,axis=2);s=np.take_along_axis(np.stack([z[0] for z in choices],axis=2),which[:,:,None],axis=2)[:,:,0];t=np.take_along_axis(np.stack([z[1] for z in choices],axis=2),which[:,:,None],axis=2)[:,:,0];delta=w+s[:,:,None]*u[:,None,:]-t[:,:,None]*v[None,:,:];return distance,s,t,delta

def solve(q,obstacles,output):
    assert np.allclose(q,0), 'This first full-material coupled solve is neutral-only.'
    aa,bases,maps,x0,bounds=setup(q);field=Field(obstacles,cq,c.m.pack._bbox);fixed,fixedSegments=fixed_points(aa);NVAR=len(x0);hist=[];cache={}
    def controls(x):return bases+np.einsum('sjkv,sv->sjk',maps,x.reshape(4,14))
    def prop(x):
        C=controls(x);v=np.einsum('tj,sjk->stk',GD,C);acc=np.einsum('tj,sjk->stk',GDD,C);speed=np.linalg.norm(v,axis=2);energy=np.sum(GW*np.sum(np.cross(v,acc)**2,axis=2)/np.maximum(speed,1e-9)**5,axis=1);length=speed@GW;return C,energy,length
    _,_,initial_lengths=prop(x0);target=np.array([sum(initial_lengths[:2]),sum(initial_lengths[2:])]);print('new free length targets',target.tolist(),flush=True)
    def energy(x):return float(np.sum(prop(x)[1])*1000.)
    def constraints(x):
        if 'x' in cache and np.array_equal(cache['x'],x):return cache['g'],cache['jac']
        C=controls(x);P=np.einsum('tj,sjk->stk',B,C);J=np.zeros((4,len(T),3,NVAR))
        for s in range(4):J[s,:,:,s*14:(s+1)*14]=np.einsum('tj,jkv->tkv',B,maps[s])
        flat=P.reshape(-1,3);jf=J.reshape(-1,3,NVAR);dist,grad,own=field.evaluate(flat);env=dist-1.5;envJ=np.einsum('nk,nkv->nv',grad,jf)
        lines=[];lineJ=[];segmentIndices=[]
        for i in [0,1]:
            lines.append(np.concatenate([P[i*2],P[i*2+1],fixed[i]]));lineJ.append(np.concatenate([J[i*2],J[i*2+1],np.zeros((len(fixed[i]),3,NVAR))]));segmentIndices.append(np.vstack([np.array([(j,j+1) for j in range(len(T)-1)]+[(len(T)+j,len(T)+j+1) for j in range(len(T)-1)]),fixedSegments[i]+2*len(T)]))
        # Coupling is active in this same solve, across both complete physical lines.
        pair=[];pairJ=[]
        for i in [0,1]:
            other=1-i;ix,iy=segmentIndices[i],segmentIndices[other];dd,ss,tt,delta=segment_pair(lines[i][ix],lines[other][iy]);nearest=np.argmin(dd,axis=1);ar=np.arange(len(nearest));vec=delta[ar,nearest];length=dd[ar,nearest];s,t=ss[ar,nearest],tt[ar,nearest];j1=(1-s[:,None,None])*lineJ[i][ix[:,0]]+s[:,None,None]*lineJ[i][ix[:,1]];nearix=iy[nearest];j2=(1-t[:,None,None])*lineJ[other][nearix[:,0]]+t[:,None,None]*lineJ[other][nearix[:,1]];pair.extend(length-3.);pairJ.extend(np.einsum('nk,nkv->nv',vec/np.maximum(length[:,None],1e-12),j1-j2))
        def radius_condition(z):
            cc=controls(z);v=np.einsum('tj,sjk->stk',D,cc);acc=np.einsum('tj,sjk->stk',DD,cc);speed=np.linalg.norm(v,axis=2);kappa=np.linalg.norm(np.cross(v,acc),axis=2)/np.maximum(speed,1e-9)**3;return (1.-1.5*kappa).ravel()
        radius=radius_condition(x);rJ=approx_derivative(radius_condition,x,method='2-point');values=np.r_[env/10.,np.array(pair)/10.,radius];jac=np.vstack([envJ/10.,np.array(pairJ)/10.,rJ]);cache.update(x=x.copy(),g=values,jac=jac,minimum_obstacle_distance_mm=float(dist.min()),minimum_between_line_sample_distance_mm=float(min(pair)+3),minimum_radius_condition=float(radius.min()),nearest_parts=[obstacles[int(own[j])]['name'] for j in np.argsort(dist)[:8]])
        return values,jac
    def eq(x):
        _,_,ls=prop(x);return (np.array([sum(ls[:2]),sum(ls[2:])])-target)/100.
    def callback(x):
        gg,_=constraints(x);ls=prop(x)[2];hist.append(dict(iteration=len(hist)+1,objective=energy(x),free_span_lengths_mm=ls.tolist(),minimum_inequality=float(gg.min()),length_eq_residual_mm=(eq(x)*100).tolist(),nearest_parts=cache['nearest_parts'],minimum_obstacle_distance_mm=cache['minimum_obstacle_distance_mm'],minimum_between_line_sample_distance_mm=cache['minimum_between_line_sample_distance_mm']))
        if len(hist)%5==0:p.write(output,dict(completed=False,history=hist));print('iteration',len(hist),hist[-1],flush=True)
    opt=minimize(energy,x0,jac=lambda x:approx_derivative(energy,x,method='2-point').ravel(),bounds=bounds,method='SLSQP',constraints=[dict(type='eq',fun=eq,jac=lambda x:approx_derivative(eq,x,method='2-point')),dict(type='ineq',fun=lambda x:constraints(x)[0],jac=lambda x:constraints(x)[1])],callback=callback,options=dict(maxiter=80,ftol=1e-9,disp=False))
    gg,_=constraints(opt.x);C,en,ls=prop(opt.x);result=dict(completed=True,solver_success=bool(opt.success),message=str(opt.message),iterations=int(opt.nit),parameters=opt.x.tolist(),controls=C.tolist(),initial_controls=controls(x0).tolist(),new_free_length_targets_mm=target.tolist(),free_span_lengths_mm=ls.tolist(),length_residual_mm=(eq(opt.x)*100).tolist(),minimum_inequality=float(gg.min()),minimum_obstacle_distance_mm=cache['minimum_obstacle_distance_mm'],minimum_between_line_sample_distance_mm=cache['minimum_between_line_sample_distance_mm'],normalized_bending_integral_per_mm=en.tolist(),history=hist,full_obstacle_parts=len(obstacles),full_boundary_triangles=len(field.tri),maximum_triangle_radius_mm=field.radius,material_field_calls=field.calls,native_near_contact_distance_queries=field.native_queries,mesh_deflection_angle=[.1,.15],centerline_samples_per_free_span=len(T),contact_inequalities_jointly_solved=True,inextensible_total_length_per_line_shared_across_free_spans=True,no_virtual_material_fix_at_guide=True,contact_materials_physics_EI_gravity_twist_friction_tension_calibrated=False,nonlocal_selfcontact_constraints_in_solver=False,nonlocal_selfcontact_will_be_native_checked=True,finite_dimensional_contact_candidate_only=True,admissible_sampled_solution=bool(opt.success and gg.min()>-1e-6 and max(abs(eq(opt.x)))<1e-7))
    p.write(output,result);return result
