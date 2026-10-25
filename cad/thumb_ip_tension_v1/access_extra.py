"""Actual countersunk envelope and access to a captured adjustment screw."""
import math
import numpy as np
from common import bounds,overlap

LIMIT=1e-4

def check(cq,geo,rows,parts,d):
    name=d['part']; kind=d['kind']
    if name not in parts: raise ValueError('Unknown tool target')
    def vector(v):
        v=np.asarray(v,dtype=float)
        if v.shape!=(3,) or not np.isfinite(v).all(): raise ValueError('Invalid vector')
        return v
    def positive(v):
        v=float(v)
        if not math.isfinite(v) or v<=0: raise ValueError('Invalid dimension')
        return v
    def cylinder(p,q,r):
        p=vector(p); delta=vector(q)-p; height=positive(np.linalg.norm(delta))
        return cq.Solid.makeCylinder(positive(r),height,cq.Vector(*p),cq.Vector(*(delta/height)))
    fit=None
    if kind=='countersunk_screw':
        top=vector(d['top']); axis=vector(d['axis']); height=positive(d['head_height_mm']); length=positive(d['total_length_mm'])
        r0=positive(d['shaft_radius_mm']); r1=positive(d['head_radius_mm']); travel=positive(d['entry_travel_mm'])
        if abs(np.linalg.norm(axis)-1)>1e-9 or length<=height or r1<=r0 or not d['length_includes_head']: raise ValueError('Invalid countersunk screw geometry')
        cone=cq.Solid.makeCone(r0,r1,height,cq.Vector(*(top-height*axis)),cq.Vector(*axis))
        final=geo.boolean(cq,'fuse',cylinder(top-length*axis,top-height*axis,r0),cone)
        geo.require_solid(final,'countersunk final envelope')
        outside=geo.describe(geo.boolean(cq,'cut',parts[name],final))['volume_mm3']
        fit=dict(actual_part=name,outside_volume_mm3=outside,tolerance_mm3=LIMIT,passed=outside<=LIMIT)
        # For a monotonically widening conical head translated in +axis, its
        # exact outer swept volume is the final cone + a head-radius cylinder
        # from the head top to the entry position. A constant-radius cylinder
        # below the head top would incorrectly intersect a matching countersink.
        insertion=geo.boolean(cq,'fuse',final,cylinder(top,top+travel*axis,r1))
        envelopes=dict(actual_profile_outer_sweep=insertion,
            driver_exterior=cylinder(top-.5*axis,top+positive(d['driver_length_mm'])*axis,positive(d['driver_radius_mm'])))
    elif kind=='driver_access':
        axis=vector(d['tool_axis']); origin=vector(d['tool_origin_mm']); tip=vector(d['tool_tip_mm'])
        if abs(np.linalg.norm(axis)-1)>1e-9 or np.dot(origin-tip,axis)<=0: raise ValueError('Tool must pass through the capture cap')
        envelopes=dict(driver_through_cap=cylinder(tip,origin+positive(d['driver_length_mm'])*axis,positive(d['driver_radius_mm'])))
    else: raise ValueError('Unsupported extra access')
    excluded={name,*d.get('stage_excluded',[])}
    if not excluded<=set(parts): raise ValueError('Unknown exclusion')
    obstacles={n:s for n,s in parts.items() if n not in excluded}
    if not obstacles: raise ValueError('Empty obstacle set')
    hits=[]; count=0; boxes={n:bounds(s) for n,s in obstacles.items()}
    for label,shape in envelopes.items():
        box=bounds(shape)
        for other,obstacle in obstacles.items():
            count+=1
            if overlap(box,boxes[other]):
                v=geo.describe(geo.boolean(cq,'common',shape,obstacle))['volume_mm3']
                if v>LIMIT: hits.append(dict(envelope=label,obstacle=other,overlap_mm3=v))
    return dict(d,checks=count,collisions=hits,final_envelope_containment=fit,
        included_obstacles=sorted(obstacles),excluded_before_stage=sorted(excluded-{name}),
        continuous_sweep=True,passed=not hits and (fit is None or fit['passed']),
        scope='Saved part containment and conservative continuous exterior/tool access. Threading, driver internal fit, grip and torque remain unqualified.')
