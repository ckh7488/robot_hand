"""Fixed-parent bench: exact two-circle exterior tangents and fixed wrap branches.

All coordinates are thumb-IP local mm. The motor axis is parallel to IP +X,
motor horn front is X=-6 and motor centre is (0,-D,0). This describes the
short free span on a one-joint test fixture, not a whole-hand Bowden route.
Import performs no work. Rope stiffness, friction and slack are not solved.
"""
from __future__ import annotations
import math

JOINT_RADIUS_MM=7.7
MOTOR_RADIUS_MM=10.
MOTOR_DISTANCE_MM=120.
TRACKS_X_MM=(-4.,4.)
JOINT_ANCHOR_DEG=0.
MOTOR_ANCHOR_DEG=(-210.,210.)
EXPLORATORY_RANGE_DEG=(-20.,85.)


def circle_point(x,yc,radius,angle):
    return [x,yc+radius*math.cos(angle),radius*math.sin(angle)]


def route(q_joint_deg,q_motor_deg=None,distance_mm=MOTOR_DISTANCE_MM):
    rj,rm=JOINT_RADIUS_MM,MOTOR_RADIUS_MM
    if q_motor_deg is None:q_motor_deg=(rj/rm)*q_joint_deg
    if not all(math.isfinite(v) for v in [q_joint_deg,q_motor_deg,distance_mm]) or distance_mm<=abs(rm-rj):
        raise ValueError('Finite angles and exterior-tangent circle spacing required')
    ny=(rm-rj)/distance_mm;nz=math.sqrt(1-ny*ny)
    rows=[]
    for i,(x,side) in enumerate(zip(TRACKS_X_MM,(-1,1))):
        a=math.atan2(side*nz,ny)
        aj=math.radians(JOINT_ANCHOR_DEG+q_joint_deg)
        am=math.radians(MOTOR_ANCHOR_DEG[i]+q_motor_deg)
        jm=(-side)*(aj-a)
        mm=(-side)*(a-am)
        if not (0<jm<2*math.pi and 0<mm<2*math.pi):
            raise ValueError('Selected joint/motor wrap branch lost contact or exceeds one turn')
        jc=circle_point(x,0,rj,a);mc=circle_point(x,-distance_mm,rm,a)
        straight=math.sqrt(distance_mm**2-(rm-rj)**2)
        rows.append(dict(track_x_mm=x,side=side,contact_angle_rad=a,
            joint_anchor_angle_rad=aj,motor_anchor_angle_rad=am,
            joint_anchor_mm=circle_point(x,0,rj,aj),motor_anchor_mm=circle_point(x,-distance_mm,rm,am),
            joint_contact_mm=jc,motor_contact_mm=mc,joint_wrap_rad=jm,motor_wrap_rad=mm,
            straight_length_mm=straight,total_working_length_mm=straight+rj*jm+rm*mm,
            fixed_tail_lengths_excluded=True,
            dL_dq_joint_mm_per_rad=(-side)*rj,dL_dq_motor_mm_per_rad=side*rm))
    return rows


def path_points(row,arc_steps=80):
    """Centreline display samples; not collision-free cable or sweep evidence."""
    if arc_steps<2:raise ValueError('Need at least two samples per arc')
    x=row['track_x_mm'];a=row['contact_angle_rad']
    motor_y=row['motor_contact_mm'][1]-MOTOR_RADIUS_MM*math.cos(a)
    am,aj=row['motor_anchor_angle_rad'],row['joint_anchor_angle_rad']
    motor=[circle_point(x,motor_y,MOTOR_RADIUS_MM,am+(a-am)*i/arc_steps) for i in range(arc_steps+1)]
    joint=[circle_point(x,0,JOINT_RADIUS_MM,a+(aj-a)*i/arc_steps) for i in range(arc_steps+1)]
    return motor+joint
