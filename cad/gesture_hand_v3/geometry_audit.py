"""Exact native-surface helpers for affine STEP and local thickness evidence.

Affine reconstruction can encode a plane as a BSpline. OCC tests the complete
underlying surface; selected vertices or a fitted sample plane are not used as
proof. These helpers do not change the supplied solids or the frozen v2 audit.
"""
import math
import numpy as np
import cadquery as cq
from OCP.BRep import BRep_Tool
from OCP.GeomLib import GeomLib_IsPlanarSurface

PLANE_TOL_MM=1e-4

def surface_plane(face,tolerance_mm=PLANE_TOL_MM):
    # This overload returns a copy transformed by the face's Location if needed.
    surface=BRep_Tool.Surface_s(face.wrapped)
    check=GeomLib_IsPlanarSurface(surface,float(tolerance_mm))
    if not check.IsPlanar():return None
    plane=check.Plan();p=plane.Location();d=plane.Axis().Direction()
    return dict(origin_mm=[p.X(),p.Y(),p.Z()],normal=[d.X(),d.Y(),d.Z()],
        method='OCC GeomLib_IsPlanarSurface on complete located native surface',tolerance_mm=tolerance_mm)

def planar_faces(shape,origin,normal):
    origin=np.asarray(origin,float);normal=np.asarray(normal,float)
    if origin.shape!=(3,) or normal.shape!=(3,) or not np.isfinite(origin).all() or not np.isfinite(normal).all() or np.linalg.norm(normal)==0:
        raise ValueError('Finite plane origin and nonzero normal required')
    normal=normal/np.linalg.norm(normal);result=[]
    for face in shape.Faces():
        plane=surface_plane(face)
        if plane is None:continue
        if (abs(np.dot(np.asarray(plane['origin_mm'])-origin,normal))<=PLANE_TOL_MM
                and np.linalg.norm(np.cross(plane['normal'],normal))<=1e-6):result.append(face)
    return result

def material_line_intervals(shape,start,end):
    """OCC edge/solid intersection intervals, in millimetres from start."""
    start=np.asarray(start,float);end=np.asarray(end,float);direction=end-start
    length=float(np.linalg.norm(direction))
    if length<=0:raise ValueError('Nonzero probe length required')
    direction/=length
    common=cq.Edge.makeLine(cq.Vector(*start),cq.Vector(*end)).intersect(shape,tol=1e-7)
    intervals=[]
    for edge in common.Edges():
        a=float(np.dot(np.asarray(edge.startPoint().toTuple())-start,direction))
        b=float(np.dot(np.asarray(edge.endPoint().toTuple())-start,direction))
        lo,hi=sorted((a,b))
        if hi-lo>1e-7:intervals.append([lo,hi])
    merged=[]
    for lo,hi in sorted(intervals):
        if merged and lo<=merged[-1][1]+1e-7:merged[-1][1]=max(merged[-1][1],hi)
        else:merged.append([lo,hi])
    return merged

def palm_thickness_screen(palm,mounts,thumb):
    """Declared exact line sections; no global/continuous wall minimum claim."""
    floor=[]
    for mount in mounts:
        yaw=math.radians(thumb['base_yaw_deg'] if mount['id'].startswith('thumb_') else 0.)
        c,s=math.cos(yaw),math.sin(yaw);x,y,_=mount['axis_point_mm']
        for dx,dy in [(-3,-3),(-3,3),(3,-3),(3,3)]:
            px=x+c*dx-s*dy;py=y+s*dx+c*dy
            intervals=material_line_intervals(palm,[px,py,-45],[px,py,-39.5])
            actual=[[lo-45,hi-45] for lo,hi in intervals]
            good=len(actual)==1 and abs(actual[0][0]+44)<=1e-4 and abs(actual[0][1]+40.5)<=1e-4
            floor.append(dict(mount=mount['id'],local_offset_mm=[dx,dy],xy_mm=[px,py],
                material_z_intervals_mm=actual,passed=good))
    web=[];cx,cy,_=thumb['root']
    for z in [-30.2,-23.,-14.]:
        for angle in range(0,360,15):
            a=math.radians(angle);u=np.array([math.cos(a),math.sin(a),0.])
            p=np.array([cx,cy,z])+16.2*u;q=p+25*u
            intervals=material_line_intervals(palm,p,q)
            web.append(dict(z_mm=z,angle_deg=angle,radial_domain_mm=[16.2,41.2],
                material_radial_intervals_mm=[[lo+16.2,hi+16.2] for lo,hi in intervals],
                remaining_material_lengths_mm=[hi-lo for lo,hi in intervals]))
    positive=[v for row in web for v in row['remaining_material_lengths_mm']]
    return dict(floor_probes=floor,floor_probe_count=len(floor),floor_3_5mm_preserved_in_sampled_sections=all(r['passed'] for r in floor),
        minimum_measured_floor_interval_mm=min((b-a for r in floor for a,b in r['material_z_intervals_mm']),default=None),
        thumb_surrounding_web_probes=web,web_probe_count=len(web),
        minimum_positive_sampled_web_interval_mm=min(positive,default=None),
        web_rays_without_material=sum(not r['remaining_material_lengths_mm'] for r in web),
        continuous_minimum_wall_thickness_mm=None,strength_approved=False,
        scope='40 exact floor line sections off the bores, and 72 radial web sections at three heights; open/empty rays retained. Cut exits meet open edges, so no positive global wall minimum or strength approval is claimed.')
