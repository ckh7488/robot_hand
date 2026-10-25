"""Small XL330/HNX330 motor-side bench coupon; millimetres, output axis +X.

Inert on import.  CAD is created only by explicitly calling build_motor_module
with a CadQuery module.  No exports, subprocesses, installation, or hardware I/O.
The missing metal-horn central retaining screw is NOT replaced by invented CAD.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path


HERE = Path(__file__).resolve().parent
_LEGACY = "cad/modular_roots_v9/parts/"
_ROTOR_SHA = "6210cadf17f10112cfa28d1b9a4c97bb7575b910381ed92469743beb6b2b4fa5"
_METAL_SHA = "ba3b0cb98bc66ee69d3c57891c3c169f54e2e270ffc235427220c1d5dfa6b692"
_SPEC = {
    "schema": "thumb_ip_drive_motor.v1",
    "units": "mm, degree; right-handed; output +X; metal horn front X=0",
    "official_horn": {
        "model": "HNX330-N101", "sku": "903-0314-000",
        "diameter_mm": 16.0, "overall_length_mm": 5.6,
        "flange_thickness_mm": 2.7, "mount_pcd_mm": 12.0,
        "mount_threads": "4 x M2x0.4 TAP THRU",
        "central_bore_diameter_mm": 2.6, "front_recess_diameter_mm": 5.6,
        "front_recess_depth_mm": 1.65,
        "chosen_configuration_retainer": "BHS M2.6x06 TAP K for XL330",
        "retainer_exact_geometry_available": False,
        "native_to_motor": [[0, 1, 0, -2.7], [-1, 0, 0, 0],
                            [0, 0, 1, 0], [0, 0, 0, 1]],
    },
    "spool": {
        "x_span_mm": [0.0, 14.0], "groove_x_mm": [2.0, 10.0],
        "cord_center_radius_mm": 10.0, "cord_diameter_mm": 0.6,
        "groove_cut_major_radius_mm": 10.05, "groove_cut_minor_radius_mm": 0.35,
        "barrel_radius_mm": 10.2, "flange_radius_mm": 11.0,
        "flange_spans_x_mm": [[0.0, 0.8], [5.6, 6.4], [13.2, 14.0]],
        "central_tool_bore_diameter_mm": 5.8,
        "mount_hole_diameter_mm": 2.2, "mount_pcd_mm": 12.0,
        "mount_screw": "Accu SSC-M2-16-A2 nominal geometry from supplier data, not vendor CAD",
        "mount_screw_head_diameter_mm": 3.8, "mount_screw_head_height_mm": 2.0,
        "mount_screw_socket_af_mm": 1.5, "mount_screw_socket_depth_mm": 1.0,
        "mount_underhead_x_mm": 14.0, "mount_tip_x_mm": -2.0,
        "horn_nominal_thread_engagement_mm": 2.0,
        "radial_land_between_groove_bottom_and_mount_hole_mm": 2.6,
        "anchor_azimuth_deg": [150.0, 210.0],
        "anchor_tail_tangent_sign": [-1.0, 1.0],
        "anchor_tail_tangent_per_axial_mm": 2.0,
        "anchor_tail_hole_diameter_mm": 0.8,
        "anchor_clamp_offset_x_mm": 2.5, "anchor_boss_radius_mm": 3.6,
        "anchor_boss_axis_span_from_cord_mm": [-4.0, 4.0],
        "anchor_clamp_axis": "anchor radial direction, exactly perpendicular to its tangent tail",
        "anchor_nominal_thread_surround_mm": 2.6,
        "anchor_boss_spans_x_mm": [[.9, 8.1], [8.9, 16.1]],
        "anchor_screw_nominal": "Accu SSF-M2-3-A2 nominal DIN913/ISO4026 envelope, not vendor CAD",
        "anchor_screw_length_mm": 3.0,
        "anchor_screw_tip_diameter_mm": 1.0,
        "anchor_screw_socket_af_mm": .9,
        "anchor_screw_socket_depth_mm": .8,
        "material_assumption": "machined aluminium, alloy and finish undecided",
        "status": "exploratory dimensions, not qualified tendon retention or strength",
    },
    "bench": {
        "bracket_box_mm": [-38.0, -32.0, -15.8, 46.0, 49.0, 2.8],
        "existing_frame_bottom_z_mm": -13.0,
        "existing_base_screw_underhead_z_mm": -11.8,
        "existing_base_nut_z_span_mm": [-17.4, -15.8],
        "mount_xy_mm": [[-34.0, -28.0], [-34.0, 13.0],
                        [4.0, -28.0], [-4.0, 13.0]],
        "post_radius_mm": 3.5, "post_z_span_mm": [-18.8, -15.8],
        "baseplate_box_mm": [-50.0, -42.5, -21.8, 70.0, 70.0, 3.0],
        "bench_bolt_clearance_diameter_mm": 3.4,
        "bench_screw_nominal": "M3x12 nominal socket screw, vendor SKU unselected",
        "bench_nut_nominal": "M3 AF5.5 x2.4 nominal, vendor SKU unselected",
        "bench_to_table_attachment": "not modelled; this is a removable local fixture base",
    },
}


def design_spec():
    return deepcopy(_SPEC)


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _path(repo, relative, hashes, expected=None):
    repo = Path(repo).resolve()
    p = (repo / relative).resolve()
    if not p.is_relative_to(repo) or not p.is_file():
        raise ValueError(f"Missing or outside-repository source: {relative}")
    value = _sha(p)
    if expected and value != expected:
        raise ValueError(f"Source hash mismatch: {relative}")
    hashes[p.relative_to(repo).as_posix()] = value
    return p


def _bounds(shape):
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    box = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape.wrapped, box, False, False)
    return list(box.Get())


def _describe(shape):
    return {"volume_mm3": shape.Volume(), "bounds_mm": _bounds(shape),
            "valid": shape.isValid(), "solid_count": len(shape.Solids())}


def _rigid(cq, shape, matrix):
    from OCP.gp import gp_Trsf
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    transform = gp_Trsf()
    transform.SetValues(*[float(matrix[r][c]) for r in range(3) for c in range(4)])
    return cq.Shape.cast(BRepBuilderAPI_Transform(shape.wrapped, transform, True).Shape())


def identify_vendor_parts(cq, repo):
    """Identify by measured geometry, not by trusting the STEP solid order."""
    hashes = {}
    root = Path(repo).resolve()
    rel = "cad/thumb_ip_drive_v1/source_docs/"
    provenance = _path(root, rel + "provenance.json", hashes)
    source_rows = json.loads(provenance.read_text(encoding="utf-8-sig"))["files"]
    for row in source_rows:
        p = _path(root, rel + row["file"], hashes, row["sha256"])
        if p.stat().st_size != row["bytes"]:
            raise ValueError("Supplier byte-size mismatch")
    rotor_path = _path(root, _LEGACY + "M077_rotor.step", hashes, _ROTOR_SHA)
    metal_path = _path(root, rel + "HNX330-N101.step", hashes, _METAL_SHA)
    solids = cq.importers.importStep(str(rotor_path)).val().Solids()
    if len(solids) != 3:
        raise ValueError("Expected the frozen three-solid rotor reference")
    wanted = {"removed_front_plastic_horn": (577.0803040541065, -5.600004431893, 0.0),
              "removed_old_central_screw": (68.80325061522328, -9.6, -.034552430519),
              "preserved_rear_idler": (464.6753430966254, -29.0, -24.85)}
    found, evidence = {}, {}
    for index, solid in enumerate(solids):
        d = _describe(solid)
        matches = [name for name, (v, xmin, xmax) in wanted.items()
                   if abs(d["volume_mm3"] - v) < 1e-5
                   and abs(d["bounds_mm"][0] - xmin) < 2e-5
                   and abs(d["bounds_mm"][3] - xmax) < 2e-5]
        if len(matches) != 1 or matches[0] in found or not d["valid"]:
            raise ValueError("Rotor solid could not be uniquely identified")
        name = matches[0]
        found[name] = solid
        evidence[name] = {"source_solid_index": index, **d}
    metal_native = cq.importers.importStep(str(metal_path)).val()
    if len(metal_native.Solids()) != 1 or not metal_native.isValid():
        raise ValueError("Metal horn is not one valid vendor solid")
    metal = _rigid(cq, metal_native, _SPEC["official_horn"]["native_to_motor"])
    b = _bounds(metal)
    if abs(b[0] + 5.6) > 2e-5 or abs(b[3]) > 2e-5:
        raise ValueError("Metal horn datum transform does not put front at X0")
    evidence["new_metal_horn"] = {**_describe(metal), "transform":
                                  _SPEC["official_horn"]["native_to_motor"]}
    return {"shapes": {**found, "new_metal_horn": metal},
            "identification": evidence, "input_hashes": hashes}


def _cylinder(cq, radius, start, direction, length):
    return cq.Solid.makeCylinder(radius, length, cq.Vector(*start), cq.Vector(*direction))


def _drill(cq, shape, a, b, radius):
    d = [b[i] - a[i] for i in range(3)]
    length = math.sqrt(sum(v*v for v in d))
    return shape.cut(_cylinder(cq, radius, a, [v/length for v in d], length))


def _hex_prism(cq, center, axis, af, length):
    # Explicit orthonormal face basis; across-flats is not the circumscribed diameter.
    axis = tuple(axis)
    helper = (0., 1., 0.) if abs(axis[1]) < .9 else (0., 0., 1.)
    cross = lambda a, b: (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
    u = cross(axis, helper)
    norm = math.sqrt(sum(v*v for v in u)); u = tuple(v/norm for v in u)
    v = cross(axis, u)
    rr = af / math.sqrt(3.)
    pts = [cq.Vector(*[center[k] + rr*(u[k]*math.cos(i*math.pi/3) + v[k]*math.sin(i*math.pi/3))
                       for k in range(3)]) for i in range(6)]
    wire = cq.Wire.makePolygon(pts + [pts[0]])
    return cq.Solid.extrudeLinear(wire, [], cq.Vector(*[a*length for a in axis]))


def _socket_screw_x(cq, y, z):
    # Nominal assembly envelope: helical threads and vendor-specific fillets absent.
    shaft = _cylinder(cq, 1., (-2., y, z), (1, 0, 0), 16.)
    head = _cylinder(cq, 1.9, (14., y, z), (1, 0, 0), 2.)
    return shaft.fuse(head).cut(_hex_prism(cq, (15., y, z), (1, 0, 0), 1.5, 1.1)).clean()


def _spool(cq):
    spec = _SPEC["spool"]
    shape = _cylinder(cq, spec["barrel_radius_mm"], (0, 0, 0), (1, 0, 0), 14)
    for lo, hi in spec["flange_spans_x_mm"]:
        shape = shape.fuse(_cylinder(cq, 11, (lo, 0, 0), (1, 0, 0), hi-lo))
    anchors = []
    for gx, deg, sign in zip(spec["groove_x_mm"], spec["anchor_azimuth_deg"], spec["anchor_tail_tangent_sign"]):
        a = math.radians(deg); er = (math.cos(a), math.sin(a)); et = (-sign*math.sin(a), sign*math.cos(a))
        start = [gx, 10*er[0], 10*er[1]]
        # Each short tail runs tangentially out of its own groove; it does not
        # tunnel underneath the other groove.  .8 mm drilling is exploratory.
        # Centreline exit from the circular boss, measured along its cross-section.
        exit_dx = 2.5 + 3.6/math.sqrt(5.)
        end = [gx+exit_dx, 10*er[0]+2*exit_dx*et[0], 10*er[1]+2*exit_dx*et[1]]
        center = [gx+2.5, 10*er[0]+5.*et[0], 10*er[1]+5.*et[1]]
        wire_unit = [1/math.sqrt(5), 2*et[0]/math.sqrt(5), 2*et[1]/math.sqrt(5)]
        direction = [0., er[0], er[1]]
        boss_start = [center[k]-4.*direction[k] for k in range(3)]
        shape = shape.fuse(_cylinder(cq, 3.6, boss_start, direction, 8.))
        # A flat end is tangent to the entire straight cord only when its axis
        # is perpendicular to the cord, not merely radial to the spool.
        tip = [center[k]+.3*direction[k] for k in range(3)]
        anchors.append({"groove_x_mm": gx, "azimuth_deg": deg,
                        "tail_start_mm": start, "tail_exit_mm": end,
                        "screw_axis_outward": list(direction), "screw_tip_mm": tip,
                        "screw_outer_mm": [tip[k]+3*direction[k] for k in range(3)],
                        "cord_center_at_clamp_mm": center,
                        "cord_unit_direction": wire_unit,
                        "screw_axis_dot_cord_axis": sum(direction[k]*wire_unit[k] for k in range(3)),
                        "thread_model": "M2 major-diameter envelope; tap drill/helix not a machining file"})
    # Complete U-grooves are cut after the thick bosses; their actual bounds
    # and relation to both tracks are inspected in the separate local audit.
    for gx in spec["groove_x_mm"]:
        shape = shape.cut(cq.Solid.makeTorus(10.05, .35, (gx, 0, 0), (1, 0, 0)))
    shape = _drill(cq, shape, (-.1, 0, 0), (14.1, 0, 0), 2.9)
    for y, z in [(6., 0.), (0., 6.), (-6., 0.), (0., -6.)]:
        shape = _drill(cq, shape, (-.1, y, z), (14.1, y, z), 1.1)
    screws = []
    for anchor in anchors:
        a, b = anchor["tail_start_mm"], anchor["tail_exit_mm"]
        # Extend both drill endpoints to guarantee an open passage into the groove/front.
        d = [b[k]-a[k] for k in range(3)]; norm = math.sqrt(sum(x*x for x in d))
        aa = [a[k]-.6*d[k]/norm for k in range(3)]
        bb = [b[k]+5.*d[k]/norm for k in range(3)]
        shape = _drill(cq, shape, aa, bb, .4)
        axis = anchor["screw_axis_outward"]; tip = anchor["screw_tip_mm"]
        outer = [tip[k] + 4.*axis[k] for k in range(3)]
        shape = _drill(cq, shape, tip, outer, 1.)
        # Supplier SSF-M2-3-A2: flat tip diameter1, 90deg relief, AF.9 and
        # socket depth.8..1.7. Use the minimum depth, omit its unspecified
        # detailed top relief, and preserve a conservative major envelope.
        shoulder = [tip[k]+.5*axis[k] for k in range(3)]
        screw = cq.Solid.makeCone(.5, 1., .5, cq.Vector(*tip), cq.Vector(*axis))
        screw = screw.fuse(_cylinder(cq, 1., shoulder, axis, 2.5))
        socket_start = [tip[k]+2.2*axis[k] for k in range(3)]
        screw = screw.cut(_hex_prism(cq, socket_start, axis, .9, .9)).clean()
        screws.append(screw)
    return shape.clean(), anchors, screws


def _bench(cq):
    s = _SPEC["bench"]
    def box(vals):
        x, y, z, dx, dy, dz = vals
        return cq.Solid.makeBox(dx, dy, dz, cq.Vector(x, y, z))
    bracket = box(s["bracket_box_mm"])
    for x, y in s["mount_xy_mm"]:
        bracket = bracket.fuse(_cylinder(cq, 3.5, (x, y, -18.8), (0, 0, 1), 3.))
    a = math.sqrt(32.)
    motor_holes = [(sx*a-14.5, -7.5+sy*(7.5+a)) for sx in (-1, 1) for sy in (-1, 1)]
    for x, y in motor_holes:
        bracket = _drill(cq, bracket, (x, y, -16.), (x, y, -12.9), 1.1)
    base = box(s["baseplate_box_mm"])
    bolts, nuts = [], []
    for x, y in s["mount_xy_mm"]:
        bracket = _drill(cq, bracket, (x, y, -19.), (x, y, -12.9), 1.7)
        base = _drill(cq, base, (x, y, -21.9), (x, y, -18.7), 1.7)
        screw = _cylinder(cq, 1.5, (x, y, -25.), (0, 0, 1), 12.).fuse(
            _cylinder(cq, 2.75, (x, y, -13.), (0, 0, 1), 3.))
        screw = screw.cut(_hex_prism(cq, (x, y, -11.3), (0, 0, 1), 2.5, 1.4)).clean()
        nut = _hex_prism(cq, (x, y, -24.2), (0, 0, 1), 5.5, 2.4)
        nut = _drill(cq, nut, (x, y, -24.3), (x, y, -21.7), 1.5)
        bolts.append(screw); nuts.append(nut)
    return bracket.clean(), base, bolts, nuts, motor_holes


def build_motor_module(cq, repo, config=None):
    """Return unexported actual shapes, measured part identity and explicit limitations.

    Current dimensions are a frozen single candidate.  Reject silent parameter
    overrides; a changed route/radius must be reviewed as another source revision.
    """
    if config not in (None, {}):
        raise ValueError("This bounded candidate does not accept dimension overrides")
    repo = Path(repo).resolve()
    vendor = identify_vendor_parts(cq, repo)
    hashes = vendor["input_hashes"]
    _path(repo, Path(__file__).resolve().relative_to(repo).as_posix(), hashes)
    parts = []
    def add(name, shape, body, role, source, **metadata):
        d = _describe(shape)
        expected_solids = 12 if name == "drive_m077_stator" else 1
        if not d["valid"] or d["solid_count"] != expected_solids or d["volume_mm3"] <= 0:
            raise ValueError(f"Invalid candidate shape: {name}")
        parts.append({"name": name, "shape": shape, "body": body, "role": role,
                      "source": source, **d, **metadata})
    fixed, moving = "bench_motor_frame", "bench_motor_output"
    for filename, role in [("M077_stator", "vendor_motor_body"),
                           ("S101_relief", "legacy_modified_vendor_body_frame")]:
        relative = _LEGACY + filename + ".step"
        p = _path(repo, relative, hashes)
        add("drive_"+filename.lower(), cq.importers.importStep(str(p)).val(), fixed, role, relative)
    add("drive_rear_idler", vendor["shapes"]["preserved_rear_idler"], fixed,
        "legacy_passive_rear_idler_preserved_neutral", _LEGACY + "M077_rotor.step",
        display_constraint_only=True, drive_torque_path=False)
    add("drive_metal_horn", vendor["shapes"]["new_metal_horn"], moving,
        "official_replacement_metal_horn", "cad/thumb_ip_drive_v1/source_docs/HNX330-N101.step")
    for filename in ("M2_body_TAP8", "M2_base_screws", "M2_base_nuts"):
        relative = _LEGACY + filename + ".step"
        p = _path(repo, relative, hashes)
        for i, shape in enumerate(cq.importers.importStep(str(p)).val().Solids()):
            add(f"drive_{filename.lower()}_{i+1}", shape, fixed,
                "legacy_nominal_hardware_envelope", relative)
    spool, anchors, anchor_screws = _spool(cq)
    add("drive_dual_groove_spool", spool, moving, "custom_machined_spool", "motor_adapter.py::_spool")
    for i, (y, z) in enumerate(((6., 0.), (0., 6.), (-6., 0.), (0., -6.))):
        add(f"drive_spool_mount_m2x16_{i+1}", _socket_screw_x(cq, y, z), moving,
            "nominal_hardware_not_vendor_cad", "cad/thumb_ip_drive_v1/source_docs/motor_fasteners/accu_SSC-M2-16-A2.pdf",
            thread_pitch_mm=.4, helical_thread_modelled=False, vendor_sku_verified=True)
    for i, shape in enumerate(anchor_screws):
        add(f"drive_tendon_clamp_m2x3_{i+1}", shape, moving,
            "nominal_flat_tip_set_screw_not_vendor_cad", "cad/thumb_ip_drive_v1/source_docs/motor_fasteners/accu_SSF-M2-3-A2.pdf",
            driver_geometry_modelled=True, vendor_sku_verified=True, socket_depth_mm=.8,
            thread_pitch_mm=.4, helical_thread_modelled=False, top_relief_modelled=False)
    bracket, base, bolts, nuts, motor_holes = _bench(cq)
    add("drive_bench_bracket", bracket, fixed, "custom_bench_bracket", "motor_adapter.py::_bench")
    add("drive_bench_baseplate", base, fixed, "custom_fixture_baseplate", "motor_adapter.py::_bench")
    for i, (bolt, nut) in enumerate(zip(bolts, nuts)):
        add(f"drive_bench_m3x12_{i+1}", bolt, fixed, "nominal_hardware_not_vendor_cad", "M3x12 envelope")
        add(f"drive_bench_m3_nut_{i+1}", nut, fixed, "nominal_hardware_not_vendor_cad", "M3 nut envelope")
    interfaces = {
        "datum": _SPEC["units"], "motor_axis": [1., 0., 0.],
        "horn_front_plane": {"origin_mm": [0., 0., 0.], "normal": [1., 0., 0.]},
        "groove_planes_x_mm": [2., 10.], "cord_center_radius_mm": 10.,
        "cord_diameter_mm": .6, "anchors": anchors,
        "spool_to_horn": {"type": "four M2 threaded fasteners and planar seat",
                          "bolt_yz_mm": [[6., 0.], [0., 6.], [-6., 0.], [0., -6.]],
                          "thread_engagement_mm": 2., "screw_tip_x_mm": -2.,
                          "central_tool_bore_diameter_mm": 5.8,
                          "central_retainer_missing": True},
        "motor_to_bench": {"chain": ["M077 body", "S101_relief", "existing body TAP screws",
                                      "existing M2x6 and nuts", "bench bracket", "M3x12 and nuts", "baseplate"],
                           "base_hole_xy_mm": [list(p) for p in motor_holes],
                           "frame_bracket_seat_z_mm": -13., "bracket_base_seat_z_mm": -18.8},
        "intentional_models": ["Horn M2 holes may use tap-minor rather than screw-major envelopes",
                               "Anchor screw threads use a major-diameter bore, not a tap drill",
                               "Cord preload, helix contact and friction are absent"],
    }
    # These are bounded regions to CHECK against the actual common shape, not
    # permission to suppress every collision between a screw and its neighbour.
    thread_regions = []
    for i, (y, z) in enumerate(((6., 0.), (0., 6.), (-6., 0.), (0., -6.))):
        thread_regions.append({"pair": ["drive_metal_horn", f"drive_spool_mount_m2x16_{i+1}"],
                               "start_mm": [-2., y, z], "axis": [1., 0., 0.],
                               "length_mm": 2., "radius_mm": 1.,
                               "reason": "vendor R.8 tap-minor geometry versus nominal M2 major envelope"})
    for i, (start_x, y) in enumerate(((-27.3, 7.5), (-27.3, -22.5),
                                     (-9.7, 7.5), (-9.7, -22.5))):
        thread_regions.append({"pair": ["drive_m077_stator", f"drive_m2_body_tap8_{i+1}"],
                               "start_mm": [start_x, y, -8.], "axis": [1., 0., 0.],
                               "length_mm": 8., "radius_mm": .95,
                               "reason": "frozen legacy body TAP shaft envelope; vendor hole geometry preserved"})
    interfaces["declared_thread_overlap_regions"] = thread_regions
    limitations = [
        "Central BHS M2.6x06 TAP K exact geometry/drive/torque remains unverified and is not an installed part; axial horn retention is incomplete.",
        "The old plastic horn and its old retainer were removed; their source geometry remains in identification evidence only, not double-installed.",
        "Horn spline nominal placement is not a tolerance-fit or vendor-approved assembly test.",
        "The unused rear idler is displayed at its neutral angle only; its possible independent idle rotation is not constrained or coupled to the spool.",
        "4 x SSC-M2-16-A2 have 2 mm nominal horn thread engagement; thread runout, tolerance stack, torque and stripping strength are unverified.",
        "R10, .6 mm cord, .8 mm tail drill and flat-tip M2 clamps are exploratory. Tail kinking, crushing, slip and cyclic fatigue require a physical coupon test.",
        "SSF-M2-3-A2 clamps use supplier nominal tip and AF.9 socket with minimum .8 depth; detailed top relief, thread helix, tool access and physical holding force remain unverified.",
        "Legacy body/frame hardware is reused as nominal geometry. Bench fasteners are nominal envelopes and fixture-to-table attachment is not provided.",
        "This module does not establish tendon routing to the joint, tensioning travel, dynamic performance, safe motor load or 50/100 ms tracking.",
    ]
    changed = [p for p, h in hashes.items() if _sha(repo/p) != h]
    if changed:
        raise RuntimeError(f"Motor module inputs changed during build: {changed}")
    return {"schema": "thumb_ip_drive_motor_module.v1", "parts": parts,
            "design_spec": design_spec(), "identification": vendor["identification"],
            "interfaces": interfaces, "limitations": limitations,
            "input_hashes": hashes, "inputs_unchanged": True,
            "part_row_count": len(parts), "native_solid_count": sum(p["solid_count"] for p in parts),
            "vendor_compound_exception": {"drive_m077_stator": 12},
            "central_retention_complete": False}
