"""Parametric hollow finger forms, in millimeters, for form_candidate_v1.

API
---
build_segment(length_mm, tip=False) -> list[dict]
Each dict contains name, shape (CadQuery Shape), color (RGB), volume_mm3,
valid, solid_count, role and limitations. Names are local: the caller must
prefix them with finger/segment identifiers before assembling a whole hand.

Coordinates: +Y is finger length, X is the hinge axis, X=Z=0 is the link
center. A normal segment has hinge centers at Y=0 and Y=length_mm. A tip
has its proximal hinge at Y=0 and its fingertip end at Y=length_mm, with no
distal hinge. Main mid-link skin occupies Y=5..L-5; hinge hardware extends
around the hinge centers. Root spacing and inter-link placement are owned
by the caller. NO axial scaling of vendor hardware is done here.

The main shell is a connected, genuinely hollow tapered CAD solid with
reinforcing bands and integral eye/fork features. Its nominal wall is
approximately 2 mm; sloped-wall minimum thickness is not certified.
Separate dark bushings and the yellow hollow fingertip cap are form and
packaging candidates, not validated fits, retained hardware or fabrication
instructions. Optional pin/pulley helpers are not automatically included,
so adjacent links do not receive duplicate shafts.

Standalone smoke check (hidden pythonw, no subprocesses are launched):
  lightweight_digits.py --deps <workspace CadQuery dependency directory>
This checks valid connected solids for representative lengths only. It
does not prove attachment, motion clearance, strength or tendon routing.
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

SHELL_COLOR = (0.22, 0.38, 0.56)
DARK_COLOR = (0.27, 0.31, 0.37)
TIP_COLOR = (0.91, 0.66, 0.25)
PIN_COLOR = (0.50, 0.54, 0.59)
LIMITATIONS = [
    'Visual/form construction candidate: no fabrication release.',
    'Nominal bearing and cap gaps are geometric assumptions, not measured fits.',
    'No axle retention, cap fastening, bearings, tendon wrap or load qualification.',
    'No inter-segment or whole-hand swept collision acceptance.',
]


def _cylinder(radius, length, origin):
    return cq.Solid.makeCylinder(radius, length, cq.Vector(*origin), cq.Vector(1, 0, 0))


def _ring(radius, bore, length, x_start, y):
    return _cylinder(radius, length, (x_start, y, 0)).cut(
        _cylinder(bore, length+2, (x_start-1, y, 0))).clean()


def _profile(y, width, depth, corner=1.4):
    """Octagonal section gives real edge transitions, not a rectangular bar."""
    x, z = width/2, depth/2
    c = min(corner, x/3, z/3)
    points = [(-x+c, -z), (x-c, -z), (x, -z+c), (x, z-c),
              (x-c, z), (-x+c, z), (-x, z-c), (-x, -z+c)]
    return cq.Wire.makePolygon([cq.Vector(px, y, pz) for px, pz in points], close=True)


def _loft(sections):
    return cq.Solid.makeLoft([_profile(*s) for s in sections], ruled=True)


def _web(x_start, width, yz_points):
    wire = cq.Wire.makePolygon([cq.Vector(x_start, y, z) for y, z in yz_points], close=True)
    return cq.Solid.extrudeLinear(wire, [], cq.Vector(width, 0, 0))


def _part(name, shape, color, role, **extra):
    shape = shape.clean()
    row = dict(name=name, shape=shape, color=color, role=role,
               volume_mm3=float(shape.Volume()), valid=bool(shape.isValid()),
               solid_count=len(shape.Solids()), limitations=LIMITATIONS.copy(), **extra)
    if not row['valid'] or row['solid_count'] != 1 or row['volume_mm3'] <= 0:
        raise ValueError('Invalid or disconnected candidate component: '+name)
    return row


def _hollow_skin(length, tip):
    start = 5.0
    end = length-4 if tip else length-5
    w0, w1 = (22.0, 20.0) if tip else (28.0, 26.0)
    d0, d1 = (18.0, 16.0) if tip else (22.0, 20.0)
    middle = (start+end)/2
    outer = _loft([(start, w0, d0), (middle, (w0+w1)/2, (d0+d1)/2), (end, w1, d1)])
    # Extend the cutter beyond both ends so the shell is open, not a solid box.
    inner = _loft([(start-1, w0-4, d0-4), (middle, (w0+w1)/2-4, (d0+d1)/2-4),
                   (end+1, w1-4, d1-4)])
    shell = outer.cut(inner)
    if not tip:
        # Two small raised collars change the actual section and reinforce the
        # hollow wall. They fuse to the shell; they are not painted stripes.
        for t in (0.30, 0.70):
            y = start+t*(end-start)
            width, depth = w0+t*(w1-w0), d0+t*(d1-d0)
            raised = _loft([(y-1.5, width+1.0, depth+1.0),
                            (y+1.5, width+1.0, depth+1.0)])
            shell = shell.fuse(raised.cut(inner))
    # The proximal central eye connects to the lower shell wall via a real
    # broad web. Its two bore bushings are separate solids in build_segment.
    eye = _ring(8.0, 3.4, 16.0, -8.0, 0)
    base_z = -d0/2
    bridge = _web(-8, 16, [(3, -7), (11, base_z-0.3),
                           (11, base_z+2.2), (3, -5)])
    shell = shell.fuse(eye).fuse(bridge)
    if not tip:
        # Outer fork rings leave 18 mm between their inner faces; the next
        # central eye is 16 mm wide. Nominal clearance is not assembly approval.
        for x in (-12.0, 9.0):
            fork = _ring(8.0, 3.4, 3.0, x, length)
            shell = shell.fuse(fork)
    return shell.clean()


def build_segment(length_mm: float, tip: bool = False) -> list[dict]:
    """Build unscaled hollow CAD solids in the local segment frame.

    Intended examples: 70 and 50 mm middle-finger links; 34 mm fingertip.
    Useful construction range: non-tip >= 36 mm, tip 28..45 mm.
    """
    length = float(length_mm)
    if not math.isfinite(length) or (tip and not 28 <= length <= 45) or (not tip and length < 36):
        raise ValueError('Expected non-tip length >=36 mm or tip length 28..45 mm')
    body = _hollow_skin(length, tip)
    parts = [_part('tip_core' if tip else 'segment_shell', body,
                   DARK_COLOR if tip else SHELL_COLOR, 'printed_hollow_structure',
                   start_joint_mm=(0, 0, 0),
                   end_joint_mm=None if tip else (0, length, 0),
                   fingertip_end_mm=(0, length, 0) if tip else None,
                   nominal_wall_mm=2.0)]
    parts.append(_part('proximal_bushing', _ring(3.35, 2.2, 16.4, -8.2, 0),
                       DARK_COLOR, 'separate_bushing_envelope'))
    if not tip:
        for label, x in [('left', -12.2), ('right', 8.8)]:
            parts.append(_part('distal_'+label+'_bushing', _ring(3.35, 2.2, 3.4, x, length),
                               DARK_COLOR, 'separate_bushing_envelope'))
        # A real, thin, separately colored fascia restores a segmented-machine
        # face without reinstating the heavy distal servo. It deliberately sits
        # above the raised bands; its attachment is not designed or approved.
        wires = []
        for y in (10.0, length-10.0):
            t = (y-5)/(length-10)
            surface_z = (22-2*t)/2
            wire = _profile(y, 21-2*t, 0.9, 0.2)
            wires.append(wire.translate((0, 0, surface_z+0.7+0.45)))
        fascia = cq.Solid.makeLoft(wires, ruled=True)
        parts.append(_part('dark_faceplate', fascia, DARK_COLOR,
                           'separate_appearance_fascia', thickness_mm=0.9,
                           attachment_status='Appearance envelope only; floating gap, fasteners and retention not designed'))
    else:
        cap_length = 14.0
        # Rounded on every outside edge; the open rear cavity makes a genuine
        # hollow fingertip, with a thicker closed pad at its distal end.
        cap = cq.Workplane('XY').box(26, cap_length, 22).edges().fillet(2.0).val()
        cap = cap.translate((0, length-cap_length/2, 0))
        cavity_length = cap_length-2
        cavity = cq.Workplane('XY').box(22, cavity_length, 18).edges('|Y').fillet(1.0).val()
        cavity = cavity.translate((0, length-3-cavity_length/2, 0))
        cap = cap.cut(cavity).clean()
        parts.append(_part('rounded_tip_cap', cap, TIP_COLOR, 'separate_hollow_tip_cap',
                           nominal_side_wall_mm=2.0, nominal_front_pad_mm=3.0,
                           attachment_status='Cap retention and assembly clearances not validated'))
    return parts


def build_joint_pin(length_mm=27.0, radius_mm=2.0, y_mm=0.0) -> dict:
    """Optional separate shaft envelope; not automatically added per link."""
    return _part('joint_pin', _cylinder(radius_mm, length_mm, (-length_mm/2, y_mm, 0)),
                 PIN_COLOR, 'unretained_pin_envelope')


def build_joint_pulley(radius_mm=8.0, width_mm=3.0, x_center_mm=15.5, y_mm=0.0) -> dict:
    """Optional ungrooved ring envelope; no tendon groove/hub approval."""
    return _part('joint_pulley', _ring(radius_mm, 2.2, width_mm,
                                      x_center_mm-width_mm/2, y_mm),
                 DARK_COLOR, 'ungrooved_pulley_envelope')


if __name__ == '__main__':
    import json
    report = []
    for length, tip in [(70, False), (50, False), (34, True), (32, True)]:
        pieces = build_segment(length, tip)
        report.append(dict(length_mm=length, tip=tip,
                           parts=[{k: v for k, v in p.items() if k != 'shape'} for p in pieces]))
    print(json.dumps(dict(status='bounded solid construction checks only', cases=report,
                          assembly_approved=False, manufacturing_ready=False), indent=2), flush=True)
