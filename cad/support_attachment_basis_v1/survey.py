"""Read actual mounting material and stored failed-curve boundary kinematics.
No new route, optimization, installed hardware, or old collision replay.
"""
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'cad/material_length_motion_v1'))
import material_model as a
m,g,c,p,np,cq=a.m,a.g,a.c,a.p,a.np,a.cq
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.OSD import OSD_ThreadPool

def distance(left,right):
    d=BRepExtrema_DistShapeShape(left.wrapped,right.wrapped);d.Perform();assert d.IsDone()
    return dict(distance_mm=d.Value(),point_on_first_mm=list(d.PointOnShape1(1).Coord()),point_on_second_mm=list(d.PointOnShape2(1).Coord()),solutions=d.NbSolution())

def bbox_distance(left,right):
    A=np.array(c.m.pack._bbox(left));B=np.array(c.m.pack._bbox(right));return float(np.linalg.norm(np.maximum(np.maximum(A[0]-B[1],B[0]-A[1]),0)))

def run():
    out=HERE/'reviews/survey.json';assert not out.exists();start=time.monotonic();data,pack,guide,wrist=a.s.prior.inputs()
    an=p.read(ROOT/'cad/material_length_motion_v1/reviews/analysis.json');source=data[0]
    for f in ['cad/material_length_motion_v1/reviews/analysis.json','cad/material_length_motion_v1/reviews/actual_common.json','cad/material_length_motion_v1/generated/actual_common/scene.json','cad/sheath_retention_v1/selected/reviews/check.json','cad/thumb_entry_bend_v1/reviews/static.json','cad/support_attachment_basis_v1/survey.py']:
        source[f]=m.drive.sha(ROOT/f)
    evidence=c.load(p.read(ROOT/'cad/material_length_motion_v1/generated/actual_common/scene.json')['parts'][:1],source)[0]
    order=sorted(pack,key=lambda z:bbox_distance(z['shape'],evidence['shape']))
    # Complete bbox lower bounds allow exact nearest-package branch and bound.
    tested=[];best=float('inf')
    for z in order:
        bound=bbox_distance(z['shape'],evidence['shape'])
        if bound>best:break
        d=distance(evidence['shape'],z['shape']);best=min(best,d['distance_mm']);tested.append(dict(name=z['name'],bbox_lower_bound_mm=bound,bbox_mm=c.m.pack._bbox(z['shape']),**d))
    structural=[z for z in pack if any(k in z['name'].lower() for k in ['rack','saddle','support','plate','spacer','clamp','adjuster'])]
    selected=sorted(structural,key=lambda z:bbox_distance(z['shape'],evidence['shape']))[:12]
    details=[dict(name=z['name'],bbox_mm=c.m.pack._bbox(z['shape']),**distance(evidence['shape'],z['shape'])) for z in selected]
    ports=p.read(ROOT/'cad/sheath_retention_v1/selected/reviews/check.json')['design']['ports'];Q=a.a.frame();entry=np.array([32.,-31.,0.]);axis=np.array([-1.,0.,0.]);terminal=[]
    for j in [0,1,2,3,4]:
        case=an['cases'][j];F=np.array(case['solution']['parent_F']);C=np.array(case['solution']['controls'])[3];T=F@Q;t=T[:3,:3]@axis;neutral=Q[:3,:3]@axis
        v=a.s.SPL.derivative()(1.)@C;acc=a.s.SPL.derivative(2)(1.)@C;normal=acc-v*(v@acc)/(v@v);speed=np.linalg.norm(v)
        terminal.append(dict(id=case['id'],angles_deg=case['angles_deg'],entry_world_mm=(T@np.r_[entry,1.])[:3],entry_tangent_world=t,entry_tangent_rotation_from_neutral_deg=np.degrees(np.arccos(np.clip(t@neutral,-1,1))),actual_entry_frame=T,stored_downstream_end_speed_mm=speed,stored_end_normal_acceleration_mm=np.linalg.norm(normal),stored_end_radius_mm=speed**2/np.linalg.norm(normal),actual_entry_axis_error=np.linalg.norm(v/speed-t),last_three_controls_mm=C[-3:],local_last_three_controls_mm=a.points(C[-3:],np.linalg.inv(T))))
    # Exact existing jaw/bolt/adjuster contact evidence, not a force/6DOF proof.
    names=['motor_housing_clamp_0_jaw_1','motor_housing_clamp_0_jaw_-1','motor_housing_clamp_0_bolt_0','motor_housing_clamp_0_nut_0','thumb_ip_barrel_adjuster_0']
    lookup={z['name']:z for z in pack};available=[n for n in names if n in lookup];contacts=[]
    for left,right in zip(available[:-1],available[1:]):contacts.append(dict(first=left,second=right,**distance(lookup[left]['shape'],lookup[right]['shape'])))
    report=dict(completed=True,source_sha256=source,packaging_rows=len(pack),structural_name_filter_count=len(structural),all_packaging_bbox_bounds_examined=len(pack),nearest_packaging_branch_bound_native_count=len(tested),nearest_packaging_mm=best,nearest_packaging_tested=tested,selected_structural_native_distances=details,actual_common_bbox_mm=c.m.pack._bbox(evidence['shape']),existing_motor_ports=[z for z in ports if z['body']=='forearm'],existing_contact_distances=contacts,upper_entry_boundary_kinematics=terminal,new_installed_hardware_rows=0,new_tube_SOLIDs=0,old_native_collision_groups_replayed=0,old_optimizer_or_Newton_replayed=0,actual_new_attachment_or_motion_approved=False,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),input_hash_guard=all(m.drive.sha(ROOT/f)==h for f,h in source.items()),elapsed_s=time.monotonic()-start)
    a.save(out,report);print('nearest_packaging_mm',best,'native_distances',len(tested)+len(details)+len(contacts),flush=True)
    print('structural',[(z['name'],round(z['distance_mm'],6)) for z in details],flush=True)
    print('entry_rotation_deg',[(z['id'],float(z['entry_tangent_rotation_from_neutral_deg'])) for z in terminal],flush=True)
if __name__=='__main__':run()
