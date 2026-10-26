"""Split housing clamps with positive capture of the terminal hardware.

Nominal contact is not a friction, pull-out load or manufacturing approval.
"""
import sys, math
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2]; HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'cad/distal_terminal_assembly_v1'))
import service as s
g,c,p,np,cq=s.g,s.c,s.p,s.np,s.cq
old=s.m.old;drive=s.m.drive

def clamp(tag,Q,body,R,lo,hi,bores,wing):
    ring=old.cy(R,lo,hi)
    for a,b,r in bores:ring=ring.cut(old.cy(r,a-.001,b+.001))
    # A 0.2 mm split is a closure allowance, not an accepted clearance target.
    rows=[]
    for side in [-1,1]:
        zlo,zhi=sorted([side*.1,side*R]);half=ring.intersect(old.bx([-R-1,lo-1,zlo],[R+1,hi+1,zhi]))
        for x in [-wing,wing]:
            arm=old.bx([min(0,x)-1.9,-8,zlo],[max(0,x)+1.9,-4,zhi])
            # Keep the housing/terminal bore open through every joining arm.
            for a,b,r in bores:arm=arm.cut(old.cy(r,a-.001,b+.001))
            half=half.fuse(arm)
        for x in [-wing,wing]:half=half.cut(old.zcyl(1.1,x,-6,-R-1,R+1,1))
        rows.append(dict(name=tag+'_jaw_'+str(side),shape=half.clean(),body=body,role='split_housing_jaw_positive_terminal_capture_friction_housing_grip_unqualified',color=[.16,.65,.69]))
    for j,x in enumerate([-wing,wing]):
        bolt=old.zcyl(1,x,-6,-R,R,1).fuse(old.zcyl(1.9,x,-6,R,R+1.2,1))
        pts=[cq.Vector(x+2.31*math.cos(k*math.pi/3),-6+2.31*math.sin(k*math.pi/3),-R-1.6) for k in range(6)]
        nut=cq.Solid.extrudeLinear(cq.Wire.makePolygon(pts,close=True),[],cq.Vector(0,0,1.6)).cut(old.zcyl(1,x,-6,-R-1.61,-R+.01,1))
        for part,sh in [('bolt',bolt),('nut',nut)]:rows.append(dict(name=tag+'_'+part+'_'+str(j),shape=sh,body=body,role='nominal_M2_smooth_envelope_threads_drive_torque_not_selected',color=[.5,.53,.57]))
    for row in rows:
        sh=row['shape'];assert sh.isValid() and len(sh.Solids())==1 and sh.Volume()>1e-6
        row['shape']=g.original.transform(sh,Q)
    return rows

def build():
    rows=[];ports=[];replace=[]
    # Motor head is captured between annular shoulders at z=-163 and -157.
    for i,x in enumerate([-1.1,18.9]):
        Q=np.eye(4);Q[:3,:3]=np.array([[1,0,0],[0,0,-1],[0,1,0]]);Q[:3,3]=[x,-243.5,-163]
        tag='motor_housing_clamp_'+str(i)
        rows+=clamp(tag,Q,'forearm',4.5,-12,7,[(-12,0,1.5),(0,6,3.5),(6,7,2.5)],6.5)
        ports.append(dict(tag=tag,body='forearm',Q=Q.tolist(),R=4.5,housing_contact_y_mm=[-12,0],grip_OD_mm=3.,captured_existing_part='thumb_ip_barrel_adjuster_'+str(i)))
    # Add a collar only to the exposed ferrule end; preserve its running bore.
    source=p.read(ROOT/'cad/thumb_sheath_terminal_v2/selected/generated/candidate_scene.json')['parts']
    originals=c.load([z for z in source if z['name'].startswith('distal_flanged_ferrule_')])
    for i in [0,1]:
        socket=np.asarray(g.original.layout()[i]['socket_transform']);Q=socket.copy()
        offset=np.array([0.,-24.,0.])+(np.array([14.,-17.,0.]) if i==1 else 0)
        Q[:3,3]=(socket@np.r_[offset,1.])[:3];Q=g.original.WORLD@Q
        name='distal_flanged_ferrule_'+str(i);base=next(z for z in originals if z['name']==name)
        collar=old.cy(2.6,0,2).cut(old.cy(1.55,-.01,2.01));sh=base['shape'].fuse(g.original.transform(collar,Q)).clean()
        assert sh.isValid() and len(sh.Solids())==1
        rows.append(dict(name=name+'_captured',shape=sh,body='thumb_mcp',role='same_ferrule_with_exposed_integral_axial_capture_collar',color=[.86,.63,.23]));replace.append(name)
        tag='distal_housing_clamp_'+str(i);rows+=clamp(tag,Q,'thumb_mcp',3.6,-11,4,[(-11,0,1.5),(0,2,2.6),(2,4,2.)],5.5)
        ports.append(dict(tag=tag,body='thumb_mcp',Q=Q.tolist(),R=3.6,housing_contact_y_mm=[-11,0],grip_OD_mm=3.,captured_existing_part=name+'_captured'))
    return rows,dict(replaced_distal_parts=replace,new_parts=len(rows),ports=ports,nominal_housing_OD_ID_mm=[3.,1.3],jaw_split_mm=.2,housing_radial_contact_mm=1.5,threaded_fasteners_are_smooth_envelopes=True,clamp_closure_and_housing_compression_not_simulated=True,pullout_force_verified=False,strain_relief_complete=False,whole_hand_drive_complete=False,hand_body_geometry_changed=False,motor_rack_electrical_positions_changed=False,source_STEP_equivalence_retested=False,inherited_OFFSET024_face_comparison_inconclusive=True)
