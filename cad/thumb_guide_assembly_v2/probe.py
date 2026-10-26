"""Bounded same-shape assembly probes; omitted parts are listed, not approved."""
import sys,math,time
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'cad/thumb_root_guide_v1'))
import guide_candidate3 as g
check=g.g.drive.module('_guide021_inputs',ROOT/'cad/thumb_root_guide_v1/build_check.py');check.g=g
c,p,np,cq=g.c,g.p,g.np,g.cq
from OCP.OSD import OSD_ThreadPool

def translate(delta):
    T=np.eye(4);T[:3,3]=delta;return T
def rotate(point,axis,degree):
    axis=np.asarray(axis,float);axis/=np.linalg.norm(axis);x,y,z=axis;a=math.radians(degree)
    K=np.array([[0,-z,y],[z,0,-x],[-y,x,0]]);R=np.eye(3)+math.sin(a)*K+(1-math.cos(a))*(K@K)
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=np.asarray(point)-R@point;return T

def main():
    out=HERE/'reviews/probe.json';assert not out.exists();start=time.monotonic()
    parts,design=g.build();data=check.inputs(design);by={z['name']:z for z in parts}
    local={n:g.original.transform(s,g.original.INV) for n,s in data[4].items()}
    child=[z['name'] for z in data[3]['hand'] if z['body']=='thumb_ip']
    hardware=['thumb_joint_pin_2']+['thumb_distal2_'+z for z in ['left_pin_washer','right_pin_washer','left_thrust_washer','right_thrust_washer','retaining_washer','retaining_screw']]
    absent=child+hardware
    report=dict(completed=False,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),source_sha256=data[0],child_module=child,joint_hardware=hardware,probes=[],roll_probes=[],whole_hand_approved=False)
    def save():p.write(out,report)
    def path(label,names,Ts,installed,missing,scope):
        obstacles=[dict(name=n,shape=s) for n,s in local.items() if n not in missing+design['replaced_hand_parts']+names]
        obstacles += [dict(name=n,shape=by[n]['local_shape']) for n in installed if n not in names]
        rows=[]
        for T in Ts:rows.append(dict(transform_local=T.tolist(),native=c.material([dict(name=n,shape=g.original.transform(by[n]['local_shape'] if n in by else local[n],T)) for n in names],obstacles)))
        got=dict(id=label,moving=list(names),missing_hand_parts=list(missing),installed_new=list(installed),scope=scope,samples=rows,clear=all(g.g.drive.clear(z['native']) for z in rows));report['probes'].append(got);save();print(label,got['clear'],flush=True);return got
    installed=['paired_liner_carrier','guide_cover_M2x12_1','guide_cover_M2x12_2']
    for i in [1,0]:
        row=g.original.layout()[i];Q=row['socket_transform'];axis=Q[:3,1];name='flanged_liner_'+str(i)
        path('no_tip_straight_'+str(i),[name],[translate(axis*x) for x in [20,15,10,5,0]],installed,absent,'Distal child and seven pin-stack parts not installed;test only insertion,not subsequent child restoration.')
        for a in [-1.,1.]:
            Ts=[rotate(Q[:3,3],Q[:3,0],a*k)@translate(axis*d) for d,k in [(20,1),(15,1),(10,1),(7.5,.5),(5,0),(0,0)]]
            path('no_tip_tilt_'+str(i)+'_'+str(a),[name],Ts,installed,absent,'Temporary1degree liner insertion tilt,not operating angle or final guide mount change.Child/pin absent.')
        installed.append(name)
    # Existing nominal slot may constrain roll even though the round collar alone cannot.
    parent=[dict(name=n,shape=s) for n,s in local.items() if n.startswith('thumb_segment2_') and n not in design['replaced_hand_parts']]
    carrier=[dict(name=n,shape=by[n]['local_shape']) for n in by if n!='flanged_liner_0' and n!='flanged_liner_1']
    for row in g.original.layout():
        name='flanged_liner_'+str(row['cord']);Q=row['socket_transform']
        for a in [-2.,-.5,0.,.5,2.]:
            T=rotate(Q[:3,3],Q[:3,1],a);moving=[dict(name=name,shape=g.original.transform(by[name]['local_shape'],T))]
            report['roll_probes'].append(dict(id=name+'_'+str(a),angle_deg=a,transform_local=T.tolist(),native_parent=c.material(moving,parent),native_carrier=c.material(moving,carrier)))
        save();print('roll',name,flush=True)
    report.update(completed=True,input_hash_guard=all(g.g.drive.sha(ROOT/z)==h for z,h in data[0].items()),elapsed_s=time.monotonic()-start)
    save();assert report['input_hash_guard']
if __name__=='__main__':main()
