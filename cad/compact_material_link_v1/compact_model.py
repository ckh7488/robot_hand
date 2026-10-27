"""New constant-material candidate on PACK049's actual guide passages.

Only this module's private continuation instance receives new port functions.
Closed earlier candidate code, budgets and saved states are never changed.
"""
import sys,math
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'cad/compact_turnback_pack_v1'))
import pack_model as hardware
old,k,np,cq=hardware.n,hardware.k,hardware.np,hardware.cq
n=old.m.drive.module('_link050_coupled_coordinates',ROOT/'cad/coupled_material_continuation_v1/continuation.py')


def ports(case,i):
    result=old.ports(case,i);H=np.array(case['H']);Q=hardware.b.frame(i)
    result.update(up_A=(Q@np.array([0.,-24.,-80.,1.]))[:3],
        up_B=(H@np.r_[hardware.saddle_edges(i)[0].startPoint().toTuple(),1.])[:3],
        up_tA=Q[:3,:3]@np.array([0.,1.,0.]),up_tB=H[:3,:3]@np.array([0.,0.,1.]))
    return result


def fixed_edges(case,i,kind):
    Q=hardware.b.frame(i);H=np.array(case['H'])
    before=[cq.Edge.makeLine(cq.Vector(0,5 if kind=='housing' else 0,0),cq.Vector(0,-12,0))]+hardware.motor_edges()
    _,_,after=old.fixed_edges(case,i,kind)
    return [n.g.original.transform(e,Q) for e in before],[n.g.original.transform(e,H) for e in hardware.saddle_edges(i)],after


n.ports=ports
n.fixed_edges=fixed_edges


def motion_cases():
    target=np.array(n.p.read(ROOT/'cad/coupled_material_continuation_v1/reviews/analysis.json')['target_parent_angles_deg'])
    result=[dict(id='parent_'+str(p),case=n.case_at(target*p/100.)) for p in [0,25,50,75,100]]
    for sign in [-1,1]:
        for f in [.25,.5,.75,1.]:result.append(dict(id='wrist_'+str(sign)+'_'+str(f),case=n.case_at(np.zeros(20),sign*20*f,sign*15*f)))
    return result,target


def neutral_seed():
    base,x,weights,budgets=old.initial();case=n.case_at(np.zeros(20));records=[]
    x=x.copy()
    for i in [0,1]:
        p=ports(case,i);v=p['up_B']-p['up_A'];axis=p['up_tA'];z=float(v@axis);lateral=v-axis*z;d=np.linalg.norm(lateral)
        assert z>0 and d>0 and np.linalg.norm(axis-p['up_tB'])<1e-10
        radius=float((z*z+d*d)/(4*d));theta=float(2*math.atan2(d,z));length=2*radius*theta
        e1,e2=n.rod.normal_frame(axis);u=lateral/d;c=np.array([u@e1,u@e2])*theta/4
        x[i*n.N:i*n.N+16]=np.r_[np.tile(c,4),np.tile(-c,4)]
        down=float(budgets[i]-x[i*n.N+n.N-1]);x[i*n.N+n.N-1]=length
        budgets[i]=length+down;weights[i][0]=np.ones(8)/8
        records.append(dict(line=i,radius_mm=radius,angle_per_arc_rad=theta,new_up_length_mm=length,
            preserved_neutral_down_length_mm=down,initial_free_contour_mm=length+down,
            derivation='Two opposite equal circular arcs at actual parallel+Z mouths: R=(axial^2+lateral^2)/(4*lateral),theta=2atan(lateral/axial). Neutral seed only, not independent moving-pose length solutions.'))
    return case,x,weights,budgets,records


def context():
    ctx=k.context();source=ctx[0][0]
    rel='cad/compact_turnback_pack_v1/generated/hardware/scene.json';source[rel]=n.m.drive.sha(ROOT/rel)
    hw=n.c.load(n.p.read(ROOT/rel)['parts'],source)
    return ctx,hw


def pose(ctx,hw,case):
    removed={z['replaces'] for z in hw};groups=[(key,[z for z in rows if z['name'] not in removed]) for key,rows in k.pose(ctx,case)]
    for z in hw:
        key='packaging' if z['body']=='forearm' else 'wrist_guide'
        for name,rows in groups:
            if name==key:
                rows.append(dict(z,shape=n.g.original.transform(z['shape'],np.array(case['H'])))) if z['body']=='palm' else rows.append(z)
    assert sum(len(rows) for _,rows in groups)==862
    return groups


def paths(x,case,weights,budgets,i,kind):
    middle=n.middle(x,case,weights,budgets,i,kind)
    return middle if kind=='housing' else n.s.paths(case['q'][19],i)[:6]+middle+[ed for _,ed in n.w.distal_edges(case,i)]


def build(x,case,weights,budgets):
    rows=[]
    for kind in ['housing','cord']:
        for i in [0,1]:
            for j,e in enumerate(paths(x,case,weights,budgets,i,kind)):
                sh=n.m.swept(e,1.5).cut(n.m.swept(e,.65)).clean() if kind=='housing' else n.m.swept(e,.3)
                assert sh.isValid() and len(sh.Solids())==1,(kind,i,j)
                rows.append(dict(name='compact_'+kind+'_'+str(i)+'_'+str(j),shape=sh,kind=kind,physical_line=i,path_segment=j,
                    length_mm=e.Length(),body='palm',current_world_pose_only=True,color=[.62,.21,.78] if i==0 else [.1,.48,.78],
                    endpoint_start_mm=e.startPoint().toTuple(),endpoint_end_mm=e.endPoint().toTuple()))
    return rows


def export(rows,label):
    return n.a.a.prior.export([{k:v for k,v in z.items() if k not in ['step','mesh','sha256_step','sha256_stl']} for z in rows],HERE/'generated'/label)
