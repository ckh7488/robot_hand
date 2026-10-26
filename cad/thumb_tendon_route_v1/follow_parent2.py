"""Outside-parent route with a non-reversing wrist approach; R5 retained."""
from route_model import *
import route_model as initial


def geometry(q,entry_y=-8.,motor_angle_deg=0.,turn_y=5.,rise_y=25.):
    rows=initial.geometry(q,entry_y,motor_angle_deg,turn_y,rise_y)
    parentq=np.asarray(q,float).copy();parentq[19]=0.;O,R=v.frames(DESIGN,parentq)['thumb_ip']
    for i,(x,side) in enumerate(zip([-4.,4.],[-1.,1.])):
        row=rows[i];wrap=next(z for z in row['pieces'] if z['label']=='motor_wrap');jointwrap=next(z for z in row['pieces'] if z['label']=='joint_wrap')
        baseline=old.route(0.)[i];y0,z0=baseline['motor_contact_mm'][1:];yt,zt=baseline['joint_contact_mm'][1:]
        zp=z0+(entry_y-y0)*(zt-z0)/(yt-y0)
        K=O+R@np.array([x,-60.,zp+side*15.]);U=O+R@np.array([x,entry_y,zp+side*15.]);P=O+R@np.array([x,entry_y,zp])
        exit=wrap['z'];points=[exit,[exit[0],turn_y,exit[2]],[110.+i*10.,rise_y,exit[2]],
            [125.+i*10.,-15.,50.+i*10.],K,U,P,jointwrap['a']]
        free,guides=initial.fillet_route(points)
        row['pieces']=[row['pieces'][0],wrap,*free,jointwrap,row['pieces'][-1]]
        row['guide_loci']=guides;row['length_mm']=sum(z['length_mm'] for z in row['pieces'])
    return rows


def coordinated(q,entry_y=-8.,turn_y=5.,rise_y=25.):
    neutral=geometry(np.zeros(20),entry_y,0.,turn_y,rise_y);base=geometry(q,entry_y,0.,turn_y,rise_y)
    changes=np.array([a['length_mm']-z['length_mm'] for a,z in zip(base,neutral)])
    angle=math.degrees((changes[0]-changes[1])/20.);rows=geometry(q,entry_y,angle,turn_y,rise_y)
    residual=[a['length_mm']-z['length_mm'] for a,z in zip(rows,neutral)];assert abs(residual[0]-residual[1])<1e-7
    return rows,dict(motor_angle_deg=angle,neutral_lengths_mm=[z['length_mm'] for z in neutral],final_lengths_mm=[z['length_mm'] for z in rows],
        common_length_residual_mm=float(np.mean(residual)),differential_residual_mm=float(residual[0]-residual[1]),
        motor_rotation_alone_keeps_both_lengths=bool(max(abs(np.array(residual)))<1e-5),tensioner_installed=False,guide_loci_are_installed=False,force_equilibrium_solved=False)
