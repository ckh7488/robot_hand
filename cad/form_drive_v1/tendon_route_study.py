"""Bounded middle-finger routing geometry study; NOT a complete transmission.

Only NumPy and the Python standard library are required. No CAD, dynamics,
hardware or subprocess is started. Run with the project's hidden Python runner.

get_routes(repo_root=None) returns four route dictionaries for later CAD lines.
`points` are body-local anchors/control points, using the new model's canonical
body names. `neutral_display_path_world_mm` is a sampled neutral drawing curve,
NOT a verified fixed-length sheath or a collision-free manufactured cable.

The exposed segment consists of a straight tangent plus a selected circular
wrap to a child-fixed termination. The radius is a nominal cable CENTRELINE
radius, not a verified grooved pulley. Its finite-difference Jacobian describes
ONLY this exposed segment. A constant-length Bowden sheath is a separate ideal
assumption. Actual motor-to-joint length/Jacobian remain unknown.

CLI: tendon_route_study.py [--root REPO] [--output report.json]
The sole generated artifact defaults to generated/tendon_route_study.json.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import platform
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
JOINTS = ("middle_mcp", "middle_pip", "middle_dip")
MOTORS = ("middle_pip", "middle_dip")
RADIUS_MM = 8.0
SPOOL_RADIUS_MM = 8.0
FD_STEP_RAD = 1e-6
OLD_A = np.array([[1., 1., 0.], [1., 1., 1.]])
IDEAL_A = np.array([[0., 1., 0.], [0., 0., 1.]])
LIMITATIONS = [
    "Only middle MCP/PIP/DIP flexion is sampled. Spread, wrist, arm, other fingers and the oblique thumb are excluded.",
    "The form thumb's 25 degree distal-axis turn requires a separate spatial route; a neutral -15 degree global direction does not remain fixed during opposition.",
    "No motor placement is connected to these routes. Palm entry points are placeholders, not installed motor outputs.",
    "The four neutral Bezier sheath curves are drawing candidates. Their pose-dependent deformation is not an inextensible-sheath equilibrium solution.",
    "Constant sheath centreline length and zero compression are ideal assumptions, not properties established by this computation.",
    "Cable offset within a bent sheath, friction, backlash, slack, stretch, tension, curvature limits, sheath compression and termination compliance are unmeasured.",
    "An 8 mm centreline circle is not an installed grooved pulley. Wrap retention, groove/hub geometry, axle support, cable fastening, slip and load capacity are unverified.",
    "Lines and arcs may pass through existing material. No CAD collision, swept clearance, assembly or installation-path check is performed.",
    "Upstream cancellation in the exposed segment follows from its parent-attached terminal. It does not prove cancellation in the omitted sheath or complete motor path.",
    "No delay, settling, dynamic tracking, torque-speed feasibility or 50/100 ms acceptance is calculated. No prior latency result transfers to this geometry.",
]


def _load_design(root):
    path = Path(root) / "cad/form_candidate_v1/design.json"
    design = json.loads(path.read_text(encoding="utf-8"))
    if design.get("units") != "mm":
        raise ValueError("Expected form coordinates in mm")
    candidates = [f for f in design["fingers"] if f["name"] == "middle"]
    if len(candidates) != 1:
        raise ValueError("Expected exactly one middle finger")
    finger = candidates[0]
    origin = np.asarray(finger["root"], dtype=float)
    lengths = np.asarray(finger["links"], dtype=float)
    if (origin.shape != (3,) or lengths.shape != (3,)
            or not np.all(np.isfinite(origin)) or not np.all(np.isfinite(lengths))
            or np.any(lengths <= 0) or np.any(lengths[:2] <= 40)):
        raise ValueError("Invalid middle-finger dimensions for this route candidate")
    return design, origin, lengths


def _rx(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1., 0., 0.], [0., c, -s], [0., s, c]])


def body_frames(q_rad, root=ROOT):
    """Body origins at proximal hinges, mm; spread and wrist held at neutral."""
    _, origin, lengths = _load_design(root)
    q = np.asarray(q_rad, dtype=float)
    if q.shape != (3,) or not np.all(np.isfinite(q)):
        raise ValueError("Expected three finite MCP/PIP/DIP radians")
    frames = {"palm": (np.eye(3), np.zeros(3))}
    R, p = np.eye(3), origin.copy()
    for i, name in enumerate(JOINTS):
        if i:
            p = p + R @ np.array([0., lengths[i-1], 0.])
        R = R @ _rx(q[i])
        frames[name] = (R.copy(), p.copy())
    return frames


def _world(point, frames):
    R, p = frames[point["body"]]
    return R @ np.asarray(point["point_local_mm"], dtype=float) + p


def _point(name, body, xyz, role):
    return dict(name=name, body=body, point_local_mm=np.asarray(xyz, dtype=float).tolist(), role=role)


def _bezier(points, samples=97):
    t = np.linspace(0., 1., samples)[:, None]
    p = np.asarray(points)
    return (1-t)**3*p[0] + 3*(1-t)**2*t*p[1] + 3*(1-t)*t*t*p[2] + t**3*p[3]


def _polyline_length(points):
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def _circle_contact(route, q_rad, frames, *, fixed_terminal_world=None, with_path=False):
    """Exact planar tangent and signed continuous wrap; no shortest-arc switch.

    The entire assembly may have arbitrary world rotation. The current study's
    sheath exit and pulley groove are coplanar in the parent frame. Off-plane
    inputs fail rather than inventing a planar projection or a fleet angle.
    """
    parent_R, parent_p = frames[route["parent_body"]]
    centre_local = np.asarray(route["pulley_centre_parent_mm"])
    terminal = _world(route["points"][3], frames)
    if fixed_terminal_world is not None:
        terminal = np.asarray(fixed_terminal_world, dtype=float)
    rel = parent_R.T @ (terminal-parent_p) - centre_local
    if abs(rel[0]) > 1e-8:
        raise ValueError("Sheath terminal is outside the candidate pulley plane")
    d = float(np.linalg.norm(rel[1:]))
    radius = route["cable_centreline_radius_mm"]
    if d <= radius:
        raise ValueError("No exterior tangent from sheath terminal to pulley")
    # Follow the continuous branch around the neutral terminal direction pi.
    alpha_s = math.pi + math.atan2(-rel[2], -rel[1])
    beta = math.acos(radius/d)
    sign = route["length_sign"]
    tangent_angle = alpha_s + sign*beta - (2*math.pi if sign > 0 else 0.)
    anchor_angle = float(q_rad[route["joint_index"]])
    wrap = sign*(anchor_angle-tangent_angle)
    if not 0. < wrap < 2*math.pi:
        raise ValueError("Selected wrap lost contact or exceeds one turn")
    radial = np.array([0., radius*math.cos(tangent_angle), radius*math.sin(tangent_angle)])
    tangent = parent_R @ (centre_local+radial) + parent_p
    anchor = _world(route["points"][5], frames)
    expected_anchor = parent_R @ (centre_local + np.array(
        [0., radius*math.cos(anchor_angle), radius*math.sin(anchor_angle)])) + parent_p
    if np.linalg.norm(anchor-expected_anchor) > 1e-8:
        raise ValueError("Child-fixed anchor and pulley transform disagree")
    straight = float(np.linalg.norm(tangent-terminal))
    result = dict(straight_length_mm=straight, wrap_angle_rad=wrap,
                  wrapped_length_mm=radius*wrap, exposed_length_mm=straight+radius*wrap,
                  tangent_radius_error_mm=abs(float(np.linalg.norm(radial))-radius),
                  tangent_dot_error_mm2=abs(float(np.dot(parent_R @ radial, tangent-terminal))),
                  child_anchor_error_mm=float(np.linalg.norm(anchor-expected_anchor)))
    if with_path:
        angles = np.linspace(tangent_angle, anchor_angle, 49)
        arc_local = np.column_stack([np.zeros_like(angles), radius*np.cos(angles), radius*np.sin(angles)])
        arc_world = (arc_local+centre_local) @ parent_R.T + parent_p
        result["display_path_world_mm"] = np.vstack([terminal, arc_world]).tolist()
    return result


def get_routes(repo_root=None):
    """Return four route candidates, with body-local points and neutral curves.

    In points[0:4], the first/last points are candidate sheath terminals and
    the interior points control a drawing Bezier only. points[4] is the neutral
    contact point (not a fixed cable attachment); points[5] is child-fixed.
    Canonical body names match sim/form_drive_v1/model_adapter.py.
    """
    root = Path(repo_root) if repo_root is not None else ROOT
    _, origin, lengths = _load_design(root)
    neutral = body_frames(np.zeros(3), root)
    routes = []
    for joint_index, parent in [(1, "middle_mcp"), (2, "middle_pip")]:
        parent_length = float(lengths[joint_index-1])
        joint = JOINTS[joint_index]
        for sign, label in [(1., "plus"), (-1., "minus")]:
            x = sign*4.
            centre = np.array([x, parent_length, 0.])
            terminal = centre + [0., -20., 0.]
            beta = math.acos(RADIUS_MM/20.)
            angle = math.pi+sign*beta-(2*math.pi if sign > 0 else 0.)
            tangent = centre + [0., RADIUS_MM*math.cos(angle), RADIUS_MM*math.sin(angle)]
            exit_direction = (tangent-terminal)/np.linalg.norm(tangent-terminal)
            points = [
                _point("proximal_sheath_entry", "palm", [origin[0]+sign*12, 0, -24],
                       "placeholder_entry_not_connected_to_any_motor"),
                _point("proximal_curve_control", "palm", [origin[0]+sign*24, 70, -14],
                       "drawing_control_not_a_physical_anchor"),
                _point("distal_curve_control", parent, terminal-30*exit_direction,
                       "drawing_control_not_a_physical_anchor"),
                _point("distal_sheath_exit", parent, terminal,
                       "candidate_parent_fixed_compression_reaction_terminal"),
                _point("neutral_circle_tangent", parent, tangent,
                       "neutral_contact_location_not_fixed_tendon_material"),
                _point("distal_tendon_termination", joint, [x, RADIUS_MM, 0.],
                       "candidate_child_fixed_cable_end_no_fastener_designed"),
            ]
            route = dict(route_id=joint+"_"+label, driven_joint=joint, joint_index=joint_index,
                         parent_body=parent, child_body=joint, length_sign=sign,
                         color_rgb=[0.88, 0.32, 0.2] if sign > 0 else [0.18, 0.55, 0.84],
                         points=points, pulley_centre_parent_mm=centre.tolist(),
                         pulley_axis_parent=[1., 0., 0.], cable_centreline_radius_mm=RADIUS_MM,
                         sheath_exit_direction_parent=exit_direction.tolist(),
                         wrap_winding="increasing_yz_angle" if sign > 0 else "decreasing_yz_angle",
                         manufactured_route=False, material_clearance_checked=False)
            sheath = _bezier([_world(p, neutral) for p in points[:4]])
            contact = _circle_contact(route, np.zeros(3), neutral, with_path=True)
            route["neutral_sheath_drawing_length_mm"] = _polyline_length(sheath)
            route["neutral_exposed_length_mm"] = contact["exposed_length_mm"]
            route["neutral_display_path_world_mm"] = np.vstack(
                [sheath, np.asarray(contact["display_path_world_mm"])[1:]]).tolist()
            route["display_curve_scope"] = "Neutral sheath Bezier followed by exact straight tangent and sampled circular wrap; no CAD clearance or fixed sheath-length proof"
            routes.append(route)
    return routes


def exposed_lengths(q_rad, routes, repo_root=ROOT):
    frames = body_frames(q_rad, repo_root)
    return np.array([_circle_contact(route, q_rad, frames)["exposed_length_mm"] for route in routes])


def exposed_jacobian(q_rad, routes, repo_root=ROOT, step_rad=FD_STEP_RAD):
    q = np.asarray(q_rad, dtype=float)
    return np.column_stack([(exposed_lengths(q+np.eye(3)[j]*step_rad, routes, repo_root)
                             - exposed_lengths(q-np.eye(3)[j]*step_rad, routes, repo_root))/(2*step_rad)
                            for j in range(3)])


def _paired(matrix):
    return 0.5*(matrix[::2]-matrix[1::2]), 0.5*(matrix[::2]+matrix[1::2])


def _pose_specs():
    return [
        ("pip_only_90", [0., 0., 0.], [0., 90., 0.]),
        ("dip_only_90", [0., 0., 0.], [0., 0., 90.]),
        ("pip_dip_together_90", [0., 0., 0.], [0., 90., 90.]),
        ("mcp_0_to_60_distals_held_45", [0., 45., 45.], [60., 45., 45.]),
        ("pip_0_to_60_dip_held_45", [30., 0., 45.], [30., 60., 45.]),
    ]


def _hashes(root):
    paths = [Path(__file__).resolve(), root/"cad/form_candidate_v1/design.json",
             root/"cad/form_candidate_v1/build.py", root/"cad/form_candidate_v1/lightweight_digits.py"]
    return {p.relative_to(root).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}


def build_report(repo_root=ROOT):
    root = Path(repo_root).resolve()
    inputs_before = _hashes(root)
    design, origin, lengths = _load_design(root)
    routes = get_routes(root)
    poses = []
    max_fd_difference = max_analytic_difference = max_common = 0.
    min_wrap = float("inf")
    max_tangent_error = 0.
    for name, q0, q1 in _pose_specs():
        rows = []
        for u in np.linspace(0., 1., 17):
            degrees = (1-u)*np.array(q0)+u*np.array(q1)
            q = np.deg2rad(degrees)
            frames = body_frames(q, root)
            contacts = [_circle_contact(r, q, frames) for r in routes]
            J = exposed_jacobian(q, routes, root)
            refined = exposed_jacobian(q, routes, root, FD_STEP_RAD/2)
            H, common = _paired(J)
            curve_lengths = [_polyline_length(_bezier([_world(p, frames) for p in r["points"][:4]])) for r in routes]
            max_fd_difference = max(max_fd_difference, float(np.max(np.abs(J-refined))))
            max_analytic_difference = max(max_analytic_difference, float(np.max(np.abs(H-RADIUS_MM*IDEAL_A))))
            max_common = max(max_common, float(np.max(np.abs(common))))
            min_wrap = min(min_wrap, min(c["wrap_angle_rad"] for c in contacts))
            max_tangent_error = max(max_tangent_error, max(c["tangent_dot_error_mm2"] for c in contacts))
            rows.append(dict(q_deg=degrees.tolist(), exposed_lengths_mm=[c["exposed_length_mm"] for c in contacts],
                             exposed_jacobian_mm_per_rad=J.tolist(), paired_exposed_jacobian_mm_per_rad=H.tolist(),
                             common_mode_jacobian_mm_per_rad=common.tolist(),
                             sheath_drawing_curve_lengths_mm=curve_lengths,
                             minimum_selected_wrap_deg=math.degrees(min(c["wrap_angle_rad"] for c in contacts))))
        poses.append(dict(name=name, samples=17, q0_deg=q0, q1_deg=q1, rows=rows))

    # Kinematic demand only: exact minimum-jerk rate, not integration of a robot.
    demands = []
    for name, q0, q1 in _pose_specs():
        duration = .150
        u = np.linspace(0., 1., 65)  # Includes u=.5, the exact rate maximum.
        blend = 10*u**3-15*u**4+6*u**5
        blend_rate = (30*u**2-60*u**3+30*u**4)/duration
        q0, delta = np.deg2rad(q0), np.deg2rad(np.array(q1)-q0)
        q = q0[None, :]+blend[:, None]*delta
        qdot = blend_rate[:, None]*delta
        exposed_A = np.array([_paired(exposed_jacobian(a, routes, root))[0]/SPOOL_RADIUS_MM for a in q])
        omega_exposed = np.einsum("tij,tj->ti", exposed_A, qdot)
        old_omega, ideal_omega = qdot @ OLD_A.T, qdot @ IDEAL_A.T
        rpm = lambda omega: (np.max(np.abs(omega), axis=0)*60/(2*math.pi)).tolist()
        draws = np.array([_paired(exposed_lengths(a, routes, root))[0] for a in q])
        demands.append(dict(name=name, duration_s=duration, command="minimum_jerk_10u3_minus15u4_plus6u5",
                            q0_deg=np.rad2deg(q0).tolist(), q1_deg=np.rad2deg(q0+delta).tolist(), samples=65,
                            old_coefficient_one_motor_peak_rpm=rpm(old_omega),
                            pure_fixed_sheath_ideal_motor_peak_rpm=rpm(ideal_omega),
                            exposed_only_equivalent_spool_peak_rpm=rpm(omega_exposed),
                            exposed_pair_draw_change_mm=(draws[-1]-draws[0]).tolist(),
                            actual_motor_peak_rpm=None,
                            scope="Equivalent exposed-end demand only; equals motor demand ONLY with the separately declared ideal constant sheath and no omitted path change"))

    # Negative control: moving only the endpoint label to the palm must expose
    # upstream coupling; a hardcoded zero Jacobian would fail this check.
    negative_route = routes[0]
    neutral = body_frames(np.zeros(3), root)
    fixed_terminal = _world(negative_route["points"][3], neutral)
    negative_rows = []
    for angle in np.linspace(0., 60., 17):
        q = np.deg2rad([angle, 45., 0.])
        local = _circle_contact(negative_route, q, body_frames(q, root))
        fixed = _circle_contact(negative_route, q, body_frames(q, root), fixed_terminal_world=fixed_terminal)
        negative_rows.append(dict(mcp_deg=float(angle), parent_terminal_exposed_length_mm=local["exposed_length_mm"],
                                  palm_fixed_terminal_exposed_length_mm=fixed["exposed_length_mm"]))
    fixed_change = negative_rows[-1]["palm_fixed_terminal_exposed_length_mm"]-negative_rows[0]["palm_fixed_terminal_exposed_length_mm"]
    local_variation = float(np.ptp([r["parent_terminal_exposed_length_mm"] for r in negative_rows]))
    checks = dict(scope="Geometry/arithmetic checks, not physical route validation",
                  finite_difference_step_rad=FD_STEP_RAD, refined_step_rad=FD_STEP_RAD/2,
                  max_finite_difference_refinement_change_mm_per_rad=max_fd_difference,
                  max_paired_exposed_vs_local_circular_formula_mm_per_rad=max_analytic_difference,
                  max_common_mode_exposed_derivative_mm_per_rad=max_common,
                  max_tangent_dot_error_mm2=max_tangent_error,
                  minimum_selected_wrap_deg=math.degrees(min_wrap),
                  finite_difference_tolerance_mm_per_rad=1e-5,
                  parent_terminal_mcp_invariance_variation_mm=local_variation,
                  negative_control_palm_fixed_terminal_length_change_mm=fixed_change,
                  negative_control_rows=negative_rows,
                  arithmetic_checks_passed=bool(max_fd_difference < 1e-5 and max_analytic_difference < 1e-5
                      and max_common < 1e-5 and max_tangent_error < 1e-8 and min_wrap > 0
                      and local_variation < 1e-8 and abs(fixed_change) > 1.))
    if not checks["arithmetic_checks_passed"]:
        raise ValueError("Routing arithmetic failed: "+json.dumps(checks))
    inputs_after = _hashes(root)
    if inputs_before != inputs_after:
        raise RuntimeError("An input changed during the routing study")
    selected_q = np.deg2rad([30., 45., 45.])
    selected_A = _paired(exposed_jacobian(selected_q, routes, root))[0]/SPOOL_RADIUS_MM
    all_curve_lengths = np.asarray([row["sheath_drawing_curve_lengths_mm"] for pose in poses for row in pose["rows"]])
    sheath_drawing_summary = []
    for i, route in enumerate(routes):
        nominal = route["neutral_sheath_drawing_length_mm"]
        sheath_drawing_summary.append(dict(route_id=route["route_id"], neutral_length_mm=nominal,
            minimum_sampled_drawing_length_mm=float(all_curve_lengths[:, i].min()),
            maximum_sampled_drawing_length_mm=float(all_curve_lengths[:, i].max()),
            maximum_absolute_change_from_neutral_mm=float(np.max(np.abs(all_curve_lengths[:, i]-nominal))),
            interpretation="This changing drawing curve is not the deformation of a proven fixed-length sheath. Values are NOT motor stroke or compression."))
    return dict(schema_version="form_drive_v1.tendon_route_study.v1", status="Exposed-end geometry and separate sheath assumptions only",
                actual_complete_transmission_validated=False, latency_evaluated=False,
                joint_names=list(JOINTS), motor_order=list(MOTORS), leg_order=[r["route_id"] for r in routes],
                coordinate_convention=dict(lengths="mm", angles="rad in APIs; explicitly named deg in report",
                    hinge_axes="+X, positive right-hand rotation, spread and wrist neutral",
                    body_frames="Canonical middle_mcp/proximal, middle_pip/middle, middle_dip/distal; body origins at proximal hinges",
                    middle_root_world_mm=origin.tolist(), middle_lengths_mm=lengths.tolist(),
                    neutral_pip_world_mm=(origin+[0, lengths[0], 0]).tolist(),
                    neutral_dip_world_mm=(origin+[0, lengths[0]+lengths[1], 0]).tolist()),
                route_candidates=routes, pose_sets=poses, speed_demand_comparisons=demands,
                sheath_drawing_variation=sheath_drawing_summary,
                length_model=dict(exposed="L_straight + radius * selected_wrap_angle; both terminal and circle centre are parent-attached, cable end is child-attached",
                    sheath_drawing="Cubic Bezier through declared controls; samples are for a neutral drawing and sensitivity illustration only",
                    ideal_sheath="Centreline length constant, compression zero, cable offset/curvature effects zero; these are NOT verified",
                    actual_sheath_length_and_compression_model=None, actual_total_motor_cable_length=None,
                    actual_motor_to_joint_jacobian=None,
                    excluded_motor_to_proximal_sheath_entry="Unrouted; actual fixed motor spool frame is not used",
                    warning="Do not add a changing drawing Bezier to the exposed segment and call it a physical cable route; do not replace its unknown physics with zero without labeling the ideal assumption"),
                transmission_comparison=dict(column_joints=list(JOINTS), row_motors=list(MOTORS),
                    nominal_spool_radius_mm=SPOOL_RADIUS_MM, nominal_joint_cable_centreline_radius_mm=RADIUS_MM,
                    old_upstream_coefficient_one_A_motor_rad_per_joint_rad=OLD_A.tolist(),
                    ideal_fixed_sheath_A_motor_rad_per_joint_rad=IDEAL_A.tolist(),
                    exposed_only_equivalent_spool_A_at_q_deg_30_45_45=selected_A.tolist(),
                    old_normalized_generalized_torque_Nm_per_motor_Nm=OLD_A.T.tolist(),
                    ideal_fixed_sheath_normalized_generalized_torque_Nm_per_motor_Nm=IDEAL_A.T.tolist(),
                    exposed_only_normalized_torque_matrix_conditional=selected_A.T.tolist(),
                    sign_convention="Positive spool angle pays out plus leg and takes in minus leg. H=(dL_plus-dL_minus)/2; A=H/r_spool. With zero common-mode derivative, tau_joint=A.T*tau_motor; positive torque corresponds to T_minus>T_plus.",
                    legacy_leg_name_warning="These plus/minus labels describe cable-length increase/decrease. They are opposite to the pinned legacy cable_forces tension-leg labels for positive extension error. Any force connection needs an explicit sign/name mapping; no such connection is made here.",
                    general_virtual_work="For each cable tau_joint=-J_length.T*tension, converting mm to m. Common-mode length derivatives create additional preload loads and must not be discarded.",
                    scope="Two distal motors and three flexion coordinates only. Torque mapping is normalized ideal virtual work, not available torque, tension, full force balance or a motor-selection result.",
                    arbitrary_coefficient_0_3_has_geometry_support=False),
                checks=checks, limitations=LIMITATIONS,
                environment=dict(python=sys.version.split()[0], numpy=np.__version__, platform=platform.platform()),
                input_sha256=inputs_before, input_hash_guard=dict(before=inputs_before, after=inputs_after, passed=True))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = build_report(args.root)
    out = args.output or args.root/"cad/form_drive_v1/generated/tendon_route_study.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False)+"\n", encoding="utf-8")
    print(json.dumps(dict(output=str(out), arithmetic_checks_passed=report["checks"]["arithmetic_checks_passed"],
                         sampled_poses=sum(p["samples"] for p in report["pose_sets"]),
                         actual_complete_transmission_validated=False, latency_evaluated=False)), flush=True)


if __name__ == "__main__":
    main()
