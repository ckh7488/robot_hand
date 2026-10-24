"""Read saved native parts and measure declared nominal assembly interfaces.

This does not approve thread engagement, clip elasticity, printed tolerances,
strength, fatigue or fabrication release. No geometry is changed by this audit.
"""
from pathlib import Path
import argparse, json, math, traceback
import numpy as np
import cadquery as cq
from OCP.gp import gp_Trsf
from audit_gestures import load_v2, collect_inputs, read_json, write_json, progress
import geometry_audit

HERE=Path(__file__).resolve().parent; ROOT=HERE.parents[1]
AREA_TOL_MM2=.02
BOOLEAN_TOL_MM=1e-6


def checked_group(label, definitions, check):
    rows=[]
    progress('fabrication_group_started', group=label, total=len(definitions))
    for index, definition in enumerate(definitions, 1):
        rows.append(check(definition))
        if index%24==0 or index==len(definitions):
            progress('fabrication_group_progress', group=label, completed=index, total=len(definitions))
    return rows


def unit_vector(value):
    result=np.asarray(value,float)
    if result.shape!=(3,) or not np.isfinite(result).all() or np.linalg.norm(result)==0:
        raise ValueError('Expected a finite nonzero three-vector')
    return result/np.linalg.norm(result)


def local_contact_patch(common, definition):
    """Limit contact to the declared seat, not another co-planar component."""
    if 'patch_radius_mm' not in definition:
        raise ValueError('A localized seating patch radius is required')
    radius=float(definition['patch_radius_mm'])
    if not math.isfinite(radius) or radius<=0:raise ValueError('Invalid patch radius')
    origin=np.asarray(definition['plane_origin_mm'],float)
    normal=unit_vector(definition['normal'])
    if 'patch_centre_xy_mm' in definition:
        x,y=map(float,definition['patch_centre_xy_mm'])
        if not np.isfinite([x,y]).all() or abs(normal[2])<1e-8:
            raise ValueError('World-Z footprint requires a finite nonvertical plane')
        # Exact vertical cylinder footprint for posts/washer disks built along Z.
        z=origin[2]-(normal[0]*(x-origin[0])+normal[1]*(y-origin[1]))/normal[2]
        half=radius*np.linalg.norm(normal[:2])/abs(normal[2])+1.
        patch=cq.Solid.makeCylinder(radius,2*half,cq.Vector(x,y,z-half),cq.Vector(0,0,1))
        kind='world_Z_cylinder_footprint'
    else:
        # Other seats include rotated thumb and distal hinge faces: their patch
        # is a true disk in the declared plane, not a global-XY disk.
        wire=cq.Workplane(cq.Plane(origin=tuple(origin),normal=tuple(normal))).circle(radius).val()
        patch=cq.Face.makeFromWires(wire)
        kind='normal_plane_disk'
    return common.intersect(patch,tol=BOOLEAN_TOL_MM),kind

def positive_seat(v, parts, definition):
    row=dict(definition=definition, passed=False, distance_mm=None, distance_measured=False)
    try:
        a,b=definition['parts'];origin=definition['plane_origin_mm'];normal=definition['normal']
        aa=geometry_audit.planar_faces(parts[a],origin,normal)
        bb=geometry_audit.planar_faces(parts[b],origin,normal)
        if not aa or not bb:
            row.update(contact_area_mm2=0.,unlocalized_contact_area_mm2=0.,reason='Missing declared planar seating face')
            return row
        common=cq.Compound.makeCompound(aa).intersect(cq.Compound.makeCompound(bb),tol=BOOLEAN_TOL_MM)
        total=float(common.Area())
        local,kind=local_contact_patch(common,definition)
        area=float(local.Area())
        row.update(contact_area_mm2=area,unlocalized_contact_area_mm2=total,patch_kind=kind,
                   passed=math.isfinite(area) and area>AREA_TOL_MM2,
                   scope='Positive contact inside the declared local patch only; full support/load capacity not implied')
    except Exception as exc:row['error']=repr(exc)
    return row

def cylinder_check(v,parts,part,origin,axis,radius,span):
    surfaces=v.cylinder_surfaces(parts[part],origin,axis)
    matches=[s for s in surfaces if abs(s['radius_mm']-radius)<=1e-4]
    return dict(part=part,axis_origin_mm=origin,axis_direction=axis,radius_mm=radius,requested_span_mm=span,
        matching_faces=matches,passed=v.merged_span_covers([s['axial_span_mm'] for s in matches],span))

def matching_seats(seat_rows,first,second):
    return [row for row in seat_rows if set(row['definition']['parts'])=={first,second}]


def required_seats_present(rows):
    return bool(rows) and all(r.get('passed') is True for r in rows)


def cover_check(v,parts,d,seat_rows):
    x,y=d['centre_xy_mm'];origin=[x,y,0.];axis=[0,0,1]
    r=dict(id=d['id'],definition=d,cylinders=[],actual_bolt_clearance=[],seats=[],passed=False)
    try:
        r['cylinders']=[cylinder_check(v,parts,d['screw'],origin,axis,d['shaft_radius_mm'],d['shaft_span_z_mm']),
                       cylinder_check(v,parts,d['nut'],origin,axis,1.5,d['nut_span_z_mm'])]
        for name in ['Palm_tapered_chassis','Palm_shaped_front_cover',d['washer'],d['nut']]:
            volume=float(parts[d['screw']].intersect(parts[name],tol=1e-6).Volume())
            r['actual_bolt_clearance'].append(dict(part=name,material_overlap_mm3=volume,passed=volume<=1e-4))
        for a,b,z in [(d['screw'],d['washer'],d['washer_top_z_mm']),
                       (d['nut'],'Palm_tapered_chassis',d['nut_upper_seat_z_mm'])]:
            r['seats'].append(positive_seat(v,parts,dict(parts=[a,b],plane_origin_mm=[x,y,z],normal=axis,
                patch_radius_mm=3.3 if a==d['screw'] else 3.175)))
        underside=matching_seats(seat_rows,d['washer'],'Palm_shaped_front_cover')
        r['washer_underside_seats']=underside
        r['passed']=bool(required_seats_present(underside) and all(c['passed'] for key in ('cylinders','actual_bolt_clearance','seats') for c in r[key]))
        r['helical_thread_engagement_verified']=False
    except Exception as exc:r['error']=repr(exc)
    return r


def flat_key_check(v,parts,d):
    r=dict(definition=d,passed=False)
    try:
        n=unit_vector(d['plane_normal']);o=np.asarray(d['plane_origin_mm'],float)
        axis=unit_vector(d['axial_direction']);origin=np.asarray(d['axial_origin_mm'],float)
        if abs(np.dot(axis,n))>1e-6:raise ValueError('Key axis must lie in its flat plane')
        offset=float(d['mating_flat_offset_mm']);span=np.asarray(d['span_mm'],float)
        if not np.isfinite(offset) or span.shape!=(2,) or not np.isfinite(span).all() or span[1]<=span[0]:
            raise ValueError('Invalid key offset/span')
        own=geometry_audit.planar_faces(parts[d['part']],o,n)
        mate=geometry_audit.planar_faces(parts[d['mating_part']],o+n*offset,n)
        if not own or not mate:
            r['reason']='Missing declared flat key face';return r
        shifted=cq.Compound.makeCompound(mate).translate(tuple(-n*offset))
        common=cq.Compound.makeCompound(own).intersect(shifted,tol=BOOLEAN_TOL_MM)
        # Rotate the exact common faces into an axis-aligned local frame before
        # OCC finite bounds; unlike vertex-only min/max this also handles curves.
        rotation=np.vstack((axis,np.cross(n,axis),n));translation=-rotation@origin
        tr=gp_Trsf();tr.SetValues(*[float(x) for row in np.column_stack((rotation,translation)) for x in row])
        intervals=[]
        for face in common.Faces():
            if face.Area()<=1e-8:continue
            bounds=v.bounds(face.moved(cq.Location(tr)))
            intervals.append([float(bounds[0,0]),float(bounds[1,0])])
        area=float(common.Area())
        covered=v.merged_span_covers(intervals,span,tolerance=1e-4)
        r.update(own_face_area_mm2=sum(f.Area() for f in own),mating_face_area_mm2=sum(f.Area() for f in mate),
                 projected_common_area_mm2=area,projected_common_axial_intervals_mm=intervals,
                 continuous_projected_coverage=bool(covered),
                 passed=bool(math.isfinite(area) and area>AREA_TOL_MM2 and covered),
                 scope='Shared face projection after removing the declared nominal flat gap; continuous axial coverage, not loaded contact or key strength')
    except Exception as exc:r['error']=repr(exc)
    return r


def solid_gate(rows):
    failures=[r['name'] for r in rows if r.get('valid') is not True or r.get('solid_count')!=1]
    return dict(part_count=len(rows),invalid_or_non_single_solid_parts=failures,
                passed=bool(rows) and not failures)


def skin_fastener_check(v,parts,d,seat_rows):
    """Query saved M2 parts and named seats; no imported pass flags are used."""
    row=dict(id=d['id'],definition=d,passed=False,cylinders=[],actual_bolt_clearance=[],seats=[])
    try:
        axis=d['axis_direction'];origin=d['axis_origin_mm']
        tip=d.get('local_axis','Z')=='Y';suffix='y' if tip else 'z'
        if d.get('local_axis','Z') not in ('Y','Z'):raise ValueError('Unsupported attachment axis')
        row['cylinders']=[cylinder_check(v,parts,d['bolt'],origin,axis,d['shaft_radius_mm'],d['shaft_span_local_'+suffix+'_mm']),
                          cylinder_check(v,parts,d['nut'],origin,axis,d['shaft_radius_mm'],d['nut_span_local_'+suffix+'_mm'])]
        if tip:
            row['cylinders'].append(cylinder_check(v,parts,d['skin'],origin,axis,d['head_counterbore_radius_mm'],d['head_span_local_y_mm']))
        for name in [d['skin'],d['structure'],d['nut']]+([d['washer']] if not tip else []):
            volume=float(parts[d['bolt']].intersect(parts[name],tol=BOOLEAN_TOL_MM).Volume())
            row['actual_bolt_clearance'].append(dict(part=name,material_overlap_mm3=volume,passed=math.isfinite(volume) and volume<=1e-4))
        pairs=[(d['structure'],d['skin']),(d['nut'],d['structure'])]
        pairs += [(d['bolt'],d['skin'])] if tip else [(d['washer'],d['skin']),(d['bolt'],d['washer'])]
        coverage=[]
        for a,b in pairs:
            selected=matching_seats(seat_rows,a,b)
            # Two clamps can share skin/structure, so require the correct id too.
            selected=[r for r in selected if r['definition'].get('id','').startswith(d['id']+'_')]
            row['seats'].extend(selected)
            coverage.append(dict(parts=[a,b],seat_rows=len(selected),passed=required_seats_present(selected)))
        row['required_seat_coverage']=coverage
        row['passed']=bool(all(c['passed'] for key in ('cylinders','actual_bolt_clearance','required_seat_coverage') for c in row[key]))
        row['thread_engagement_driver_access_preload_strength_verified']=False
    except Exception as exc:row['error']=repr(exc)
    return row

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--cad-root',type=Path,default=HERE)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    if args.output.exists():raise FileExistsError('Use a new audit output')
    candidate=args.cad_root.resolve();repository=candidate.parents[1]
    report=dict(completed=False,manufacturing_approved=False,errors=[],parts=[],mesh_topology=[])
    before={}
    try:
        v=load_v2(repository);scene=read_json(candidate/'generated/scene.json');build=read_json(candidate/'generated/build_report.json')
        before=collect_inputs(repository,candidate,candidate/'poses.json',repository/'cad/accepted_form_v1',scene,build,read_json(candidate/'poses.json'))
        before[str(Path(__file__).resolve())]=v.digest(__file__)
        before[str(Path(geometry_audit.__file__).resolve())]=v.digest(geometry_audit.__file__)
        # A fresh in-memory module adapts plane recognition for affine BSpline
        # surfaces. The frozen v2 source itself is neither changed nor rewritten.
        v.planar_faces=geometry_audit.planar_faces
        report['plane_recognition']='geometry_audit.planar_faces; explicit runtime adapter also used by frozen v2 root checks'
        progress('fabrication_parts_loading', total=len(scene['hand']))
        parts=v.load_parts(scene,candidate,report)
        progress('fabrication_parts_loaded', completed=len(parts))
        report['solid_gate']=solid_gate(report['parts'])
        if not report['solid_gate']['passed']:raise ValueError('Invalid or disconnected saved part; nominal interface result withheld')
        progress('fabrication_root_interfaces_started')
        report['root_interfaces']=v.audit_interfaces(build['root_module'],parts)
        progress('fabrication_root_interfaces_completed')
        metadata=build['fabrication_repairs']
        if not metadata:raise ValueError('No fabrication repair metadata')
        report['cylinder_interfaces']=[]
        for d in metadata.get('cylinder_interfaces',[]):
            report['cylinder_interfaces'].append(cylinder_check(v,parts,d['part'],d['axis_origin_mm'],d['axis_direction'],d['radius_mm'],d['span_mm']))
        report['seating_interfaces']=checked_group('seating_interfaces',metadata.get('seating_interfaces',[]),lambda d:positive_seat(v,parts,d))
        report['flat_keys']=checked_group('flat_keys',metadata.get('flat_key_interfaces',[]),lambda d:flat_key_check(v,parts,d))
        report['actual_pin_clearance']=[]
        for d in metadata.get('clearance_shape_probes',[]):
            r=dict(definition=d,checks=[],passed=False)
            try:
                for name in d['against_parts']:
                    volume=float(parts[d['moving_part']].intersect(parts[name],tol=1e-6).Volume())
                    r['checks'].append(dict(part=name,material_overlap_mm3=volume,passed=volume<=d['maximum_intersection_mm3']))
                r['passed']=bool(r['checks']) and all(c['passed'] for c in r['checks'])
            except Exception as exc:r['error']=repr(exc)
            report['actual_pin_clearance'].append(r)
        report['cover_fasteners']=checked_group('cover_fasteners',metadata.get('cover_fasteners',[]),lambda d:cover_check(v,parts,d,report['seating_interfaces']))
        report['skin_fasteners']=checked_group('skin_fasteners',metadata.get('skin_fasteners',[]),lambda d:skin_fastener_check(v,parts,d,report['seating_interfaces']))
        groups=['cylinder_interfaces','seating_interfaces','flat_keys','actual_pin_clearance','cover_fasteners']
        if metadata.get('skin_fasteners'):groups.append('skin_fasteners')
        report['summary']={g:dict(total=len(report[g]),failed=sum(not r['passed'] for r in report[g])) for g in groups}
        report['summary']['all_declared_nominal_checks_passed']=bool(report['root_interfaces']['all_declared_checks_passed']
            and report['root_interfaces']['all_mount_nuts_seated_at_nominal_pose']
            and report['solid_gate']['passed'] and all(report[g] and all(r['passed'] for r in report[g]) for g in groups))
        report['unverified']=metadata.get('remaining',[])+['Pin retention is a custom nominal concept; spring installation/load/fit still unqualified.',
            'Near contact does not establish tolerance or loaded clearance; gesture transitions audited separately.']
        report['completed']=True
    except Exception as exc:report['errors'].append(dict(error=repr(exc),traceback=traceback.format_exc()))
    after={p:v.digest(p) if Path(p).is_file() else None for p in before}
    report['input_hash_guard']=dict(start=before,end=after,passed=bool(before) and before==after)
    report['completed']=bool(report['completed'] and report['input_hash_guard']['passed'])
    write_json(args.output,report)
    print(json.dumps(dict(completed=report['completed'],summary=report.get('summary'),errors=report['errors'])),flush=True)
    return 0 if report['completed'] else 1

if __name__=='__main__':raise SystemExit(main())
