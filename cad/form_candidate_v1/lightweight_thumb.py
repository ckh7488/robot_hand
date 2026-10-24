"""Hollow thenar/metacarpal form with physically yawed distal hinge bores.

build_thumb_metacarpal(length_mm=50, distal_yaw_deg=25) -> list of part dicts.
The list contract matches lightweight_digits: name, shape, color, role,
volume_mm3, valid, solid_count and limitations. Prefix names when assembling.

Local millimeters: proximal hinge center [0,0,0], axis +X; distal center
[0,L,0], axis Rz(yaw) @ +X. The child phalanx must be translated to [0,L,0]
and rotated by Rz(yaw). Geometry, including the physical fork and bores,
uses that same transform. No pin is supplied; add one once per joint.

The teal body is one connected hollow solid: broad tapered palm-side skin,
proximal eye/web and a rotated distal fork connected by side webs/crossbar.
Its nominal wall is 2.2mm. Three separate dark bushing envelopes have no
retention or manufacturing approval. This is a form/packaging candidate,
not certified fit, hand clearance, strength or tendon transmission.

Standalone: hidden pythonw lightweight_thumb.py --deps <CadQuery path>.
No subprocess or GUI is launched. The bounded check measures actual BRep
cylindrical bore axes as well as validity/connected-solid counts.
"""
from __future__ import annotations

if __name__ == '__main__':
    import argparse
    import sys
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--deps')
    options = parser.parse_args()
    if options.deps:
        sys.path.insert(0, options.deps)

import math
import cadquery as cq

TEAL = (0.22, 0.58, 0.64)
DARK = (0.27, 0.31, 0.37)
LIMITATIONS = [
    'Form/packaging construction candidate; no fabrication release.',
    'Nominal wall and bushing gaps are not a validated manufacturing tolerance.',
    'Pin retention, bearings, cap/skin fasteners and tendon routing remain undesigned.',
    'Distal central-eye clearance is a neutral envelope only, not swept-motion clearance.',
    'Palm mating, loaded strength and whole-hand assembly are not approved.',
]


def _cyl(radius, length, start):
    return cq.Solid.makeCylinder(radius, length, cq.Vector(*start), cq.Vector(1, 0, 0))


def _ring(radius, bore, length, x, y=0):
    return _cyl(radius, length, (x, y, 0)).cut(_cyl(bore, length+2, (x-1, y, 0))).clean()


def _profile(y, width, depth, cx=0):
    x, z, c = width/2, depth/2, 2.2
    points = [(-x+c, -z), (x-c, -z), (x, -z+c), (x, z-c),
              (x-c, z), (-x+c, z), (-x, z-c), (-x, -z+c)]
    return cq.Wire.makePolygon([cq.Vector(px+cx, y, pz) for px, pz in points], close=True)


def _loft(sections):
    return cq.Solid.makeLoft([_profile(*s) for s in sections], ruled=True)


def _box(width, length, depth, center):
    return cq.Workplane('XY').box(width, length, depth).val().translate(center)


def _web(x, width, yz):
    wire = cq.Wire.makePolygon([cq.Vector(x, y, z) for y, z in yz], close=True)
    return cq.Solid.extrudeLinear(wire, [], cq.Vector(width, 0, 0))


def _distal(shape, length, yaw):
    return shape.rotate((0, 0, 0), (0, 0, 1), yaw).translate((0, length, 0))


def _part(name, shape, color, role, **extra):
    shape = shape.clean()
    result = dict(name=name, shape=shape, color=color, role=role,
                  volume_mm3=float(shape.Volume()), valid=bool(shape.isValid()),
                  solid_count=len(shape.Solids()), limitations=LIMITATIONS.copy(), **extra)
    if not result['valid'] or result['solid_count'] != 1 or result['volume_mm3'] <= 0:
        raise ValueError('Invalid/disconnected thumb component: '+name)
    return result


def build_thumb_metacarpal(length_mm: float = 50, distal_yaw_deg: float = 25) -> list[dict]:
    """Return genuine hollow CAD bodies; caller owns global placement/palm fit.

    Tested default: L50/yaw25. Parameter bounds are construction guardrails,
    not evidence that every allowed combination has passed assembly review.
    """
    length, yaw = float(length_mm), float(distal_yaw_deg)
    if not math.isfinite(length) or not math.isfinite(yaw) or not 44 <= length <= 60 or not 0 <= yaw <= 35:
        raise ValueError('Construction range: length44..60 mm, yaw0..35 degrees')
    wall = 2.2
    sections = [(0, 36, 26, -2), (length*.24, 35, 26, -2.2),
                (length*.60, 29, 24, -1.0), (length-5, 22, 20, 0)]
    outer = _loft(sections)
    inner_sections = [(y, w-2*wall, d-2*wall, cx) for y, w, d, cx in sections]
    inner_sections[0] = (-1, *inner_sections[0][1:])
    inner_sections[-1] = (length-4, *inner_sections[-1][1:])
    body = outer.cut(_loft(inner_sections))
    # A wide lower web links the central proximal eye to the thenar shell's
    # actual lower wall. Neither the eye nor the distal fork is floating.
    eye = _ring(8, 3.4, 16, -8)
    proximal_web = _web(-8, 16, [(3, -7), (13, -12.6), (13, -10.2), (3, -5)])
    body = body.fuse(eye).fuse(proximal_web)

    # Build the ENTIRE fork in the child frame before yawing it, including
    # side webs, crossbar and physical clearance bores.
    fork = None
    for x in (-12, 9):
        side = _ring(8, 3.4, 3, x)
        rear_web = _box(3, 10, 12, (x+1.5, -8, 0))
        side = side.fuse(rear_web).cut(_cyl(3.4, 5, (x-1, 0, 0)))
        fork = side if fork is None else fork.fuse(side)
    crossbar = _box(24, 4, 4, (0, -12, -7))
    fork = fork.fuse(crossbar)
    body = body.fuse(_distal(fork, length, yaw))
    # Reserve the next lightweight_digits central eye's nominal envelope.
    # This clears the rotated front opening; it is not a motion sweep.
    eye_clearance = _distal(_cyl(8.4, 16.8, (-8.4, 0, 0)), length, yaw)
    body = body.cut(eye_clearance).clean()
    axis = (math.cos(math.radians(yaw)), math.sin(math.radians(yaw)), 0.0)
    result = [_part('thenar_shell', body, TEAL, 'connected_hollow_thenar_structure',
                    nominal_wall_mm=wall, proximal_joint_mm=(0, 0, 0),
                    proximal_axis=(1, 0, 0), distal_joint_mm=(0, length, 0),
                    distal_axis=axis, distal_yaw_deg=yaw,
                    distal_fork_bore_radius_mm=3.4,
                    central_eye_neutral_clearance=dict(radius_mm=8.4, axial_length_mm=16.8))]
    result.append(_part('proximal_bushing', _ring(3.35, 2.2, 16.4, -8.2),
                        DARK, 'separate_bushing_envelope'))
    for label, x in [('left', -12.2), ('right', 8.8)]:
        bushing = _distal(_ring(3.35, 2.2, 3.4, x), length, yaw)
        result.append(_part('distal_'+label+'_bushing', bushing, DARK,
                            'separate_yawed_bushing_envelope', axis=axis))
    return result


def verify_distal_axes(parts, length_mm=50, distal_yaw_deg=25):
    """Measure actual radius3.4 BRep cylinders of the structural body.

    This verifies orientation/axis location, not fits or load continuity.
    A connected-solid check independently rejects a detached fork.
    """
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Cylinder
    length, yaw = float(length_mm), float(distal_yaw_deg)
    wanted = (math.cos(math.radians(yaw)), math.sin(math.radians(yaw)), 0)
    matches = []
    for face in parts[0]['shape'].Faces():
        surface = BRepAdaptor_Surface(face.wrapped)
        if surface.GetType() != GeomAbs_Cylinder:
            continue
        cylinder = surface.Cylinder()
        if abs(cylinder.Radius()-3.4) > 1e-7:
            continue
        direction = cylinder.Axis().Direction()
        a = (direction.X(), direction.Y(), direction.Z())
        location = cylinder.Axis().Location()
        delta = (-location.X(), length-location.Y(), -location.Z())
        dot = sum(x*y for x, y in zip(delta, a))
        distance = math.sqrt(sum((d-dot*v)**2 for d, v in zip(delta, a)))
        alignment = abs(sum(x*y for x, y in zip(a, wanted)))
        center = face.Center()
        if center.y > length-18:
            axial_center = center.x*wanted[0] + (center.y-length)*wanted[1] + center.z*wanted[2]
            matches.append(dict(radius_mm=cylinder.Radius(), axis=list(a),
                                distance_to_distal_joint_axis_mm=distance,
                                absolute_direction_dot=alignment,
                                axis_error_deg=math.degrees(math.acos(min(1.0, alignment))),
                                face_axial_center_mm=axial_center))
    left = any(r['face_axial_center_mm'] < -8.5 for r in matches)
    right = any(r['face_axial_center_mm'] > 8.5 for r in matches)
    passed = left and right and all(r['distance_to_distal_joint_axis_mm'] < 1e-7 and
                                   r['axis_error_deg'] < 1e-5 for r in matches)
    return dict(passed=passed and len(parts[0]['shape'].Solids()) == 1,
                matching_cylindrical_faces=matches,
                maximum_axis_error_deg=max((r['axis_error_deg'] for r in matches), default=None),
                maximum_axis_distance_mm=max((r['distance_to_distal_joint_axis_mm'] for r in matches), default=None),
                both_fork_sides_found=left and right,
                required_axis=list(wanted), required_joint_mm=[0, length, 0],
                structural_body_solid_count=parts[0]['solid_count'],
                manufacturing_approved=False)


# Compatibility alias for the first integration draft; both names measure the
# same actual BRep cylinders and return the same report.
inspect_distal_bore_axes = verify_distal_axes


if __name__ == '__main__':
    import json
    parts = build_thumb_metacarpal()
    axes = verify_distal_axes(parts)
    assert axes['passed'], axes
    print(json.dumps(dict(parts=[{k: v for k, v in p.items() if k != 'shape'} for p in parts],
                          actual_brep_bore_check=axes, assembly_approved=False,
                          manufacturing_ready=False), indent=2), flush=True)
