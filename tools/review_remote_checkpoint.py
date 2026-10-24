"""Read-only remote-drive contract review; never a manufacturing/performance release.

No candidate generator is imported. With --geometry, re-read individual saved
STEP solids and independently aggregate their mass properties into joint frames.
Mass/density assumptions remain assumptions even when the geometry is consistent.
"""
from pathlib import Path
import argparse
import hashlib
import importlib.util
import json
import math
import subprocess
import sys

import numpy as np


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def finite_tree(obj, path="root"):
    if isinstance(obj, float):
        assert math.isfinite(obj), f"Nonfinite value: {path}"
    elif isinstance(obj, dict):
        for key, value in obj.items():
            finite_tree(value, path + "." + str(key))
    elif isinstance(obj, list):
        for i, value in enumerate(obj):
            finite_tree(value, path + f"[{i}]")


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


def frame(position=(0, 0, 0), R=None):
    out = np.eye(4)
    out[:3, 3] = position
    if R is not None:
        out[:3, :3] = R
    return out


def expected_frames(design):
    out = {}
    for f in design["hand"]["fingers"]:
        n = f["name"]
        names = ([f"{n}_{j}" for j in ["spread", "mcp", "pip", "dip"]]
                 if n != "thumb" else
                 ["thumb_base_sweep", "thumb_opposition", "thumb_mcp", "thumb_ip"])
        rx, ry, rz = np.deg2rad(f["base_rpy"])
        R = rotation([0, 0, 1], rz) @ rotation([0, 1, 0], ry) @ rotation([1, 0, 0], rx)
        T = frame(f["root"], R)
        for j, name in enumerate(names):
            if j == 1:
                T = T @ frame([0, 0, design["hand"]["root_axis_offset_z"]])
            elif j >= 2:
                T = T @ frame([0, f["links"][j - 2], 0])
                if n == "thumb" and j == 2:
                    T = T @ frame(R=rotation([0, 0, 1], math.radians(f["mcp_mount_yaw"])))
            out[name] = {"frame": T.copy(), "parent": "palm" if j == 0 else names[j - 1],
                         "axis": [0, 0, -1] if j == 0 else [1, 0, 0]}
    return out


def tensor_valid(tensor, label):
    I = np.asarray(tensor)
    assert I.shape == (3, 3), label
    assert np.max(abs(I - I.T)) < 1e-11, f"Asymmetric inertia: {label}"
    values = np.linalg.eigvalsh(I)
    assert values[0] > 0, f"Nonpositive inertia: {label}"
    assert values[2] <= values[0] + values[1] + 1e-10, f"Impossible principal inertia: {label}"
    return values


def part_properties_from_step(root, directory, p, cache):
    import cadquery as cq
    from OCP.GProp import GProp_GProps
    from OCP.BRepGProp import BRepGProp

    source = p.get("source")
    if source is None:
        filename = directory / "generated/parts" / (p["name"] + ".step")
        placement = None  # Individual candidate STEP was exported in world frame.
    else:
        if source == "V9 vendor XL330-M077 STEP":
            source = "cad/modular_roots_v9/parts/M077_stator.step"
        elif source == "visual rotor; mass already included in motor":
            source = "cad/modular_roots_v9/parts/M077_rotor.step"
        filename = (root / source.replace("\\", "/")).resolve()
        assert filename.is_relative_to(root), f"External source: {source}"
        placement = np.asarray(p["transform_world"])
    if filename not in cache:
        shape = cq.importers.importStep(str(filename)).val()
        assert shape.isValid(), str(filename)
        g = GProp_GProps()
        BRepGProp.VolumeProperties_s(shape.wrapped, g)
        center, inertia = g.CentreOfMass(), g.MatrixOfInertia()
        cache[filename] = (float(g.Mass()), np.array([center.X(), center.Y(), center.Z()]),
                           np.array([[inertia.Value(i + 1, j + 1) for j in range(3)] for i in range(3)]))
    volume, center, geometric_inertia = cache[filename]
    assert volume > 0, str(filename)
    tensor = geometric_inertia * p["mass_kg"] / volume * 1e-6
    if placement is not None:
        R = placement[:3, :3]
        center = R @ center + placement[:3, 3]
        tensor = R @ tensor @ R.T
    return volume, center, tensor


def review_physical_envelope(root, directory, parts):
    """Reread physical geometry including zero-added-mass motor rotor visuals."""
    import cadquery as cq
    from OCP.gp import gp_Trsf
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib

    source_cache, boxes, new_part_counts = {}, [], []
    for part in parts:
        if part["route"]:
            continue
        source = part.get("source")
        if source is None:
            path = directory / "generated/parts" / (part["name"] + ".step")
        else:
            source = {"V9 vendor XL330-M077 STEP": "cad/modular_roots_v9/parts/M077_stator.step",
                      "visual rotor; mass already included in motor": "cad/modular_roots_v9/parts/M077_rotor.step"}.get(source, source)
            path = (root / source.replace("\\", "/")).resolve()
            assert path.is_relative_to(root), source
        if path not in source_cache:
            source_cache[path] = cq.importers.importStep(str(path)).val()
        shape = source_cache[path]
        if source is not None:
            transform = gp_Trsf()
            transform.SetValues(*[float(x) for x in np.asarray(part["transform_world"])[:3, :4].ravel()])
            shape = cq.Shape.cast(BRepBuilderAPI_Transform(shape.wrapped, transform, True).Shape())
        else:
            solid_count = len(shape.Solids())
            new_part_counts.append({"name": part["name"], "solid_count": solid_count})
            assert solid_count == 1, f"New manufactured component is not one connected solid: {part['name']} ({solid_count} solids)"
        bounds = Bnd_Box()
        BRepBndLib.AddOptimal_s(shape.wrapped, bounds, False, False)
        coordinates = bounds.Get()
        boxes.append((coordinates[:3], coordinates[3:]))
    minimum = np.min([row[0] for row in boxes], axis=0)
    maximum = np.max([row[1] for row in boxes], axis=0)
    declared = read(directory / "reviews/geometry.json").get("physical_assembly_bounds_mm")
    if declared is not None:
        assert np.max(abs(minimum - declared["min"])) < 1e-5, "Assembly minimum bounds differ from saved solids"
        assert np.max(abs(maximum - declared["max"])) < 1e-5, "Assembly maximum bounds differ from saved solids"
    return {"physical_components_checked": len(boxes), "min_mm": minimum.tolist(), "max_mm": maximum.tolist(),
            "extent_mm": (maximum - minimum).tolist(), "declared_bounds_present": declared is not None,
            "new_component_solid_counts": new_part_counts,
            "new_components_not_single_solid": [row for row in new_part_counts if row["solid_count"] != 1],
            "scope": "Saved physical STEP solids at neutral pose, exact-surface bounds without triangulation, including motor rotors; cable envelopes excluded. "
                     "This is an assembly envelope, not a swept volume, attachment proof or manufacturing release."}


def review_additive_metrics(metrics, label, names):
    """CP005 audit without importing the producer's evaluator; CP004 is unchanged."""
    if "event_classification_counts" not in metrics:
        return None
    categories = ["reached", "early_crossing", "preexisting_level", "never_reached"]
    counts = dict.fromkeys(categories, 0)
    cutoff = metrics["early_crossing_cutoff_ms"]
    for event in metrics["events"]:
        difference = event["command_end_deg"] - event["command_start_deg"]
        assert abs(abs(difference) - event["motion_amplitude_deg"]) < 1e-8, label
        assert event["direction"] * difference > 0, label
        threshold = event["command_start_deg"] + event["fraction"] * difference
        assert abs(threshold - event["threshold_deg"]) < 1e-8, label
        assert abs(event["actual_start_deg"] - event["command_start_deg"] - event["actual_start_error_deg"]) < 1e-8, label
        past = bool(event["direction"] * (event["actual_start_deg"] - threshold) >= 0)
        assert past == event["initially_past_threshold"], label
        identifiable = event["actual_event_s"] is not None
        assert identifiable == event["timing_identifiable"], label
        expected = ("preexisting_level" if past else "never_reached" if not identifiable else
                    "early_crossing" if event["lag_ms"] < cutoff else "reached")
        assert event["classification"] == expected, label
        legacy = "missing" if not identifiable else "anticipatory" if expected == "early_crossing" else "reached"
        assert event["status"] == legacy, label
        if past:
            assert not identifiable and event["lag_ms"] is None, label
        if not past:
            assert event["later_directional_crossing_s"] is None, label
        counts[expected] += 1
    assert counts == metrics["event_classification_counts"], label
    assert metrics["indeterminate_events"] == counts["preexisting_level"] + counts["never_reached"], label
    assert metrics["indeterminate_events"] == metrics["missing_events"], label

    params = metrics["hold_quality_parameters"]
    quality = metrics["hold_quality"]
    assert metrics["unsettled_quality_holds"] == sum(x["status"] == "unsettled" for x in quality), label
    groups = {}
    metadata = metrics["command_generation"]
    for row in quality:
        assert row["joint"] in names and row["motion_amplitude_deg"] >= 0, label
        expected_band = max(params["minimum_tolerance_deg"], min(params["maximum_tolerance_deg"],
                            params["relative_amplitude_fraction"] * row["motion_amplitude_deg"]))
        assert abs(row["tolerance_deg"] - expected_band) < 1e-10, label
        assert row["max_hold_error_deg"] + 1e-10 >= abs(row["final_error_deg"]), label
        if row["status"] == "settled":
            assert row["settle_after_command_end_ms"] is not None, label
            assert abs(row["final_error_deg"]) <= expected_band + 1e-10, label
        else:
            assert row["status"] == "unsettled" and row["settle_after_command_end_ms"] is None, label
        assert row["joint"] not in groups.setdefault(row["move"], set()), label
        groups[row["move"]].add(row["joint"])
    assert all(joints == set(names) for joints in groups.values()), f"Incomplete stationary-axis hold coverage: {label}"
    expected_holds = {i for i, move in enumerate(metadata["moves"])
                      if move["hold_end_s"] - move["end_s"] >= params["minimum_observed_hold_s"]}
    assert set(groups) == expected_holds, f"Missing qualifying hold: {label}"
    motion = metrics["motion_excursions"]
    for row in motion:
        assert row["joint"] in names and row["endpoint_overshoot_deg"] >= 0 and row["wrong_way_excursion_deg"] >= 0, label
        assert row["window_end_s"] > row["window_start_s"], label
    for aggregate, field in [("max_endpoint_overshoot_deg", "endpoint_overshoot_deg"),
                             ("max_wrong_way_excursion_deg", "wrong_way_excursion_deg")]:
        assert abs(metrics[aggregate] - max((x[field] for x in motion), default=0.0)) < 1e-10, label
    joint_rows = metrics["per_joint"]
    for prefix, per_joint_prefix in [("dynamic_raw", "raw"), ("task_raw", "task_raw"), ("projection_loss", "projection_loss")]:
        max_field = prefix + ("_max_error_deg" if prefix.endswith("raw") else "_max_deg")
        per_max = per_joint_prefix + ("_max_error_deg" if per_joint_prefix.endswith("raw") else "_max_deg")
        assert abs(metrics[max_field] - max(x[per_max] for x in joint_rows)) < 1e-8, label
        rms = math.sqrt(sum(x[per_joint_prefix + "_rmse_deg"] ** 2 for x in joint_rows) / len(joint_rows))
        assert abs(metrics[prefix + "_rmse_deg"] - rms) < 1e-8, label
    # Task error = dynamic error + reference projection error; verify norm bounds.
    for dynamic, task, projection in [("dynamic_raw_max_error_deg", "task_raw_max_error_deg", "projection_loss_max_deg"),
                                      ("dynamic_raw_rmse_deg", "task_raw_rmse_deg", "projection_loss_rmse_deg")]:
        a, b, c = metrics[dynamic], metrics[task], metrics[projection]
        assert abs(a - c) - 1e-8 <= b <= a + c + 1e-8, label
    assert set(metrics["commanded_joint_names"]) | set(metrics["held_joint_names"]) == set(names), label
    assert not set(metrics["commanded_joint_names"]) & set(metrics["held_joint_names"]), label
    expected_events, expected_excursions = set(), set()
    for mi, move in enumerate(metadata["moves"]):
        for j, name in enumerate(names):
            if abs(move["feasible_q1_deg"][j] - move["feasible_q0_deg"][j]) >= .1:
                expected_excursions.add((mi, name))
                expected_events.update((mi, name, fraction) for fraction in [.1, .5, .9])
    actual_events = {(row["move"], row["joint"], row["fraction"]) for row in metrics["events"]}
    assert actual_events == expected_events and len(actual_events) == len(metrics["events"]), label
    actual_excursions = {(row["move"], row["joint"]) for row in motion}
    assert actual_excursions == expected_excursions and len(actual_excursions) == len(motion), label
    for row in metrics["events"]:
        move, j = metadata["moves"][row["move"]], names.index(row["joint"])
        assert abs(row["command_start_deg"] - move["feasible_q0_deg"][j]) < 1e-8, label
        assert abs(row["command_end_deg"] - move["feasible_q1_deg"][j]) < 1e-8, label
        # An inverse minimum-jerk solve independently checks reference event time.
        lo, hi = 0., 1.
        for unused in range(50):
            s = (lo + hi) / 2
            if s ** 3 * (10 + s * (-15 + 6 * s)) < row["fraction"]:
                lo = s
            else:
                hi = s
        expected_time = move["start_s"] + (lo + hi) / 2 * (move["end_s"] - move["start_s"])
        assert abs(row["command_event_s"] - expected_time) <= metadata["physics_dt_s"], label
    for row in quality + motion:
        move, j = metadata["moves"][row["move"]], names.index(row["joint"])
        assert abs(row["motion_amplitude_deg"] - abs(move["feasible_q1_deg"][j] - move["feasible_q0_deg"][j])) < 1e-8, label
    return {"classification_counts": counts, "hold_rows_checked": len(quality), "all_axis_holds": len(groups),
            "motion_excursions_checked": len(motion), "projection_loss_max_deg": metrics["projection_loss_max_deg"],
            "task_raw_max_error_deg": metrics["task_raw_max_error_deg"]}


def reconstruct_commands(metadata, times):
    """Evaluate the declared endpoint/time contract, independent of scenario()."""
    assert metadata["kind"] == "piecewise_minimum_jerk" and metadata["angle_units"] == "deg"
    arrays = []
    for prefix in ["requested", "feasible"]:
        result = np.repeat(np.asarray(metadata["initial_" + prefix + "_deg"])[None, :], len(times), axis=0)
        last_end = -math.inf
        for move in metadata["moves"]:
            start, end, hold = move["start_s"], move["end_s"], move["hold_end_s"]
            assert last_end <= start + 1e-10 and start < end <= hold, "Inconsistent segment time metadata"
            last_end = end
            mask = times >= start
            s = np.clip((times[mask] - start) / (end - start), 0, 1)
            blend = s ** 3 * (10 + s * (-15 + 6 * s))
            q0, q1 = np.asarray(move[prefix + "_q0_deg"]), np.asarray(move[prefix + "_q1_deg"])
            result[mask] = q0 + blend[:, None] * (q1 - q0)
        arrays.append(result)
    return arrays


def review_command_contracts(root, report, names):
    if not any("command_generation" in value for value in report["cases"].values()):
        return None
    signatures, rows = {}, []
    all_runs = dict(report["cases"])
    all_runs.update({"sensitivity/" + key: value for key, value in report.get("sensitivities", {}).items()})
    for key, metrics in all_runs.items():
        meta = metrics["command_generation"]
        dt = meta["physics_dt_s"]
        assert dt > 0 and meta["time_end_s"] > dt, key
        count = round(meta["time_end_s"] / dt) + 1
        assert abs((count - 1) * dt - meta["time_end_s"]) < 1e-8, key
        time = np.arange(count) * dt
        requested, feasible = reconstruct_commands(meta, time)
        assert requested.shape == feasible.shape == (count, len(names)), key
        policy = meta["reference_policy"]
        expected = requested.copy()
        if policy == "virtual16_nonthumb_DIP_equals_0.8_PIP":
            expected[:, [3, 7, 11, 15]] = .8 * expected[:, [2, 6, 10, 14]]
        else:
            assert policy == "independent20", key
        assert np.max(abs(feasible - expected)) < 1e-9, key
        projection = feasible - requested
        assert abs(float(np.max(abs(projection))) - metrics["projection_loss_max_deg"]) < 1e-8, key
        assert abs(float(np.sqrt(np.mean(projection ** 2))) - metrics["projection_loss_rmse_deg"]) < 1e-8, key
        for j, row in enumerate(metrics["per_joint"]):
            assert row["joint"] == names[j], key
            assert abs(float(np.max(abs(projection[:, j]))) - row["projection_loss_max_deg"]) < 1e-8, key
            assert abs(float(np.sqrt(np.mean(projection[:, j] ** 2))) - row["projection_loss_rmse_deg"]) < 1e-8, key
        summary = {"case": key, "reconstructed_command_samples": count,
                   "projection_loss_max_deg": metrics["projection_loss_max_deg"]}
        if not key.startswith("sensitivity/"):
            variant, case = key.split("/", 1)
            signature = {"initial_requested_deg": meta["initial_requested_deg"], "time_end_s": meta["time_end_s"],
                         "moves": [{k: v for k, v in move.items() if not k.startswith("feasible_")} for move in meta["moves"]]}
            if case in signatures:
                assert signature == signatures[case], f"Requested task changed between variants: {case}"
            signatures[case] = signature
            displayed = read(root / "sim/remote_drive_v1/results" / f"{variant}_{case}_trajectory.json")
            finite_tree(displayed, key + "_trajectory")
            assert displayed["joint_names"] == names and displayed["command_generation"] == meta, key
            assert displayed["variant"] == variant and displayed["case"] == case, key
            sample_time = np.asarray(displayed["time_s"])
            assert sample_time[0] == 0 and np.all(np.diff(sample_time) > 0), key
            rq, fq = reconstruct_commands(meta, sample_time)
            assert np.max(abs(rq - displayed["requested_deg"])) < 1e-3, key
            assert np.max(abs(fq - displayed["command_deg"])) < 1e-3, key
            actual = np.asarray(displayed["actual_deg"])
            assert actual.shape == rq.shape, key
            # Decimated/rounded samples only give lower bounds on full-rate peaks.
            assert float(np.max(abs(actual - rq))) <= metrics["task_raw_max_error_deg"] + 1e-3, key
            assert float(np.max(abs(actual - fq))) <= metrics["dynamic_raw_max_error_deg"] + 1e-3, key
            for event in metrics["events"]:
                start = meta["moves"][event["move"]]["start_s"]
                samples = np.flatnonzero(abs(sample_time - start) < 1e-9)
                if len(samples):
                    j = names.index(event["joint"])
                    assert abs(actual[samples[0], j] - event["actual_start_deg"]) < 1e-3, key
            if case == "independent_distal":
                assert np.max(abs(requested[:, [3, 7, 11, 15]] - .8 * requested[:, [2, 6, 10, 14]])) > 1.0, key
                assert (metrics["projection_loss_max_deg"] > 1.0) == (variant == "remote16"), key
            summary["display_samples_checked"] = len(sample_time)
        rows.append(summary)
    return {"cases_with_identical_requested_tasks": len(signatures), "reconstructed_runs": rows,
            "scope": "Independent command reconstruction from committed endpoints/times, exact full-rate projection-loss arithmetic, "
                     "and rounded display-trace consistency. Decimated actual angles do not revalidate fine-rate events or sustained settling."}


def review_model_test_evidence(root):
    evidence = read(root / "sim/remote_drive_v1/results/model_consistency.json")
    finite_tree(evidence, "model_consistency")
    paths = {name: root / "sim/remote_drive_v1" / name for name in ["model.py", "evaluate.py", "test_model.py"]}
    paths["simulation_contract.json"] = root / "cad/remote_drive_v1/simulation_contract.json"
    for name, value in evidence["source_hashes"].items():
        assert digest(paths[name]) == value, f"Stale consistency-test evidence: {name}"
    small = evidence["details"].get("small5_time_step_quality")
    if small is not None:
        assert small["coarse_dt_s"] == .000125 and small["fine_dt_s"] == .0000625
        assert small["declared_max_difference_limit_deg"] == .02
        assert small["hold_quality_rows"] == 80
        assert small["hold_quality_classifications_stable"] == (not small["changed_hold_classifications"])
        if evidence["passed"]:
            assert small["full_trajectory_max_difference_deg"] <= small["declared_max_difference_limit_deg"]
            assert small["hold_quality_classifications_stable"]
    return {"producer_test_passed": evidence["passed"], "producer_tests_run": evidence["tests"],
            "source_hashes_current": True, "small_motion_numerical_evidence": small,
            "scope": "Freshness and stated-threshold consistency of producer test results; tests are not rerun by this reviewer."}


def review_simulation(root, contract, design):
    path = root / "sim/remote_drive_v1/results/report.json"
    report = read(path)
    finite_tree(report, "simulation_report")
    for name, expected_hash in report["source_hashes"].items():
        assert digest(root / name) == expected_hash, f"Stale simulation source: {name}"
    assert "cad/remote_drive_v1/design.json" in report["source_hashes"], "CAD parameters missing from simulation hashes"
    assert "cad/remote_drive_v1/simulation_contract.json" in report["source_hashes"]
    names = [j["name"] for j in contract["joints"]]
    aliases = {"thumb_spread": "thumb_base_sweep", "thumb_mcp": "thumb_opposition",
               "thumb_pip": "thumb_mcp", "thumb_dip": "thumb_ip"}
    bodies = {b["name"]: b for b in contract["bodies"]}
    remote = report["provenance"]["remote20"]
    assert remote["actuator_count"] == remote["physical_joint_count"] == 20
    for key, props in remote["body_mass_properties"].items():
        body = bodies[aliases.get(key, key)]
        assert abs(props["mass_kg"] - body["mass_kg"]) < 1e-12, key
        assert np.allclose(np.asarray(props["com_local_m"]) * 1000, body["com_local_mm"], atol=1e-9), key
        assert np.allclose(props["inertia_com_kg_m2"], body["inertia_com_local_kg_m2"], atol=1e-12), key
    assert len(remote["body_mass_properties"]) == 20
    config = remote["parameters"]
    assert config["cable_preload_N"] == design["remote_motors"]["preload_N"]
    assert abs(config["motor_spool_radius_m"] * 1000 - design["remote_motors"]["spool_radius_mm"]) < 1e-12
    assert abs(config["joint_pulley_radius_m"] * 1000 - design["links"]["pulley_radius_mm"]) < 1e-12
    A = np.eye(20)
    for name, body in bodies.items():
        if "transmission" not in body:
            continue
        row = names.index(name)
        route = body["transmission"]
        A[row] = 0
        for upstream, coefficient in route["motor_angle_coefficients"].items():
            A[row, names.index(upstream)] = coefficient * route["joint_radius_mm"] / route["spool_radius_mm"]
    assert np.allclose(A, remote["motor_angle_from_joint_angle_matrix"], atol=1e-12), "CAD and simulation transmission differ"

    summaries, coverage, additions = [], set(), {}
    runs = dict(report["cases"])
    runs.update({"sensitivity/" + key: value for key, value in report.get("sensitivities", {}).items()})
    for key, metrics in runs.items():
        events = metrics["events"]
        assert metrics["event_count"] == len(events), key
        missing = sum(e["status"] == "missing" for e in events)
        assert missing == metrics["missing_events"], key
        assert sum(e["status"] == "anticipatory" for e in events) == metrics["anticipatory_events"], key
        lags = [e["lag_ms"] for e in events if e["lag_ms"] is not None]
        for event in events:
            assert event["joint"] in names and event["direction"] in [-1, 1], key
            if key.startswith("remote20/"):
                coverage.add((event["joint"], event["direction"]))
            if event["lag_ms"] is not None:
                measured = (event["actual_event_s"] - event["command_event_s"]) * 1000
                assert abs(measured - event["lag_ms"]) < 1e-7, key
        if lags:
            for field, expected in [("lag_median_ms", np.median(lags)),
                                    ("lag_p95_ms", np.percentile(lags, 95)),
                                    ("valid_event_max_ms", max(lags))]:
                assert abs(metrics[field] - expected) < 1e-8, (key, field)
        if missing:
            assert metrics["maximum_including_missing_ms"] is None, key
            assert not metrics["simulated_100ms_event_budget_met"], key
        if metrics["simulated_100ms_event_budget_met"]:
            assert events and not missing and metrics["anticipatory_events"] == 0 and max(lags) <= 100, key
        if metrics["simulated_50ms_event_target_met"]:
            assert metrics["simulated_100ms_event_budget_met"] and max(lags) <= 50, key
        errors = [j["raw_max_error_deg"] for j in metrics["per_joint"]]
        assert len(errors) == 20 and abs(max(errors) - metrics["raw_max_error_deg"]) < 1e-8, key
        summaries.append({"case": key, "event_count": len(events), "missing_events": missing,
                          "valid_event_max_ms": metrics["valid_event_max_ms"],
                          "simulated_100ms_event_budget_met": metrics["simulated_100ms_event_budget_met"]})
        additive = review_additive_metrics(metrics, key, names)
        if additive is not None:
            additions[key] = additive
    assert coverage == {(name, sign) for name in names for sign in [-1, 1]}, "Incomplete bidirectional full-hand cases"
    return {"source_hash_count": len(report["source_hashes"]),
            "remote20_bidirectional_joint_coverage": len(coverage) // 2,
            "CAD_mass_and_transmission_match": True, "event_summary_reaggregation": summaries,
            "additive_metric_review": additions,
            "command_contract_review": review_command_contracts(root, report, names),
            "model_test_evidence_review": review_model_test_evidence(root),
            "scope": "Checks committed event arithmetic/coverage and CAD-to-simulation data consistency; "
                     "does not independently replay or calibrate the dynamics engine."}


def review_model_frames(root, contract):
    import mujoco

    sys.dont_write_bytecode = True
    path = root / "sim/remote_drive_v1/model.py"
    spec = importlib.util.spec_from_file_location("_reviewed_remote_model", path)
    candidate = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(candidate)
    system = candidate.build_model("remote20")
    model, data = system["model"], system["data"]
    aliases = {"thumb_base_sweep": "thumb_spread", "thumb_opposition": "thumb_mcp",
               "thumb_mcp": "thumb_pip", "thumb_ip": "thumb_dip"}
    rows = contract["joints"]
    bodies = {b["name"]: b for b in contract["bodies"]}
    assert list(candidate.NAMES) == [r["name"] for r in rows]
    rng = np.random.default_rng(20261024)
    configurations = [np.zeros(20)]
    ranges = np.deg2rad(np.asarray([r["range_deg"] for r in rows]))
    for unused in range(12):
        configurations.append(ranges[:, 0] + rng.uniform(.15, .85, 20) * np.diff(ranges, axis=1).ravel())
    max_position = max_rotation = max_axis = max_com = max_inertia = 0.0
    for q in configurations:
        mujoco.mj_resetData(model, data)
        data.qpos[:20] = q
        mujoco.mj_forward(model, data)
        expected = {"palm": np.eye(4)}
        for i, row in enumerate(rows):
            relative = np.asarray(row["frame_parent"]).copy()
            relative[:3, 3] /= 1000
            world = expected[row["parent_joint"]] @ relative @ frame(R=rotation(row["axis_local"], q[i]))
            name = row["name"]
            expected[name] = world
            body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, aliases.get(name, name))
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            assert body_id >= 0 and joint_id >= 0
            actual_R = data.xmat[body_id].reshape(3, 3)
            max_position = max(max_position, float(np.max(abs(data.xpos[body_id] - world[:3, 3]))))
            max_rotation = max(max_rotation, float(np.max(abs(actual_R - world[:3, :3]))))
            max_axis = max(max_axis, float(np.max(abs(data.xaxis[joint_id] - world[:3, :3] @ row["axis_local"]))))
            props = bodies[name]
            expected_com = world[:3, :3] @ (np.asarray(props["com_local_mm"]) / 1000) + world[:3, 3]
            max_com = max(max_com, float(np.max(abs(data.xipos[body_id] - expected_com))))
            expected_I = world[:3, :3] @ props["inertia_com_local_kg_m2"] @ world[:3, :3].T
            principal_R = data.ximat[body_id].reshape(3, 3)
            actual_I = principal_R @ np.diag(model.body_inertia[body_id]) @ principal_R.T
            max_inertia = max(max_inertia, float(np.max(abs(actual_I - expected_I))))
            assert abs(model.body_mass[body_id] - props["mass_kg"]) < 1e-10
    assert max_position < 1e-9 and max_rotation < 1e-9 and max_axis < 1e-9
    assert max_com < 1e-9 and max_inertia < 1e-10
    return {"poses": len(configurations), "joints_per_pose": len(rows),
            "max_position_error_m": max_position, "max_rotation_matrix_error": max_rotation,
            "max_axis_error": max_axis, "max_COM_error_m": max_com,
            "max_world_inertia_error_kg_m2": max_inertia,
            "scope": "Independent CAD parent-frame/axis composition versus compiled MuJoCo model at neutral and 12 non-neutral poses; "
                     "includes body COM and rotated inertia, not integrated trajectory or physical calibration."}


def review_parent_bores(directory, contract, parts, design):
    """Measure actual parent/child bore and shaft axes near each distal hinge.

    Geometric mismatches are reported as construction failures, not made into
    exceptions that would hide the rest of the checkpoint's numerical review.
    Nominal axis agreement does not establish tolerance, bearing or assembly fit.
    """
    import cadquery as cq
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Cylinder

    radius = design["links"]["joint_bore_radius_mm"]
    cache, measurements = {}, []

    def measure(candidates, wanted_radius, origin, expected_axis):
        measured = []
        for part in candidates:
            path = directory / "generated/parts" / (part["name"] + ".step")
            if path not in cache:
                shape = cq.importers.importStep(str(path)).val()
                cylinders = []
                for face in shape.Faces():
                    surface = BRepAdaptor_Surface(face.wrapped)
                    if surface.GetType() != GeomAbs_Cylinder:
                        continue
                    cylinder = surface.Cylinder()
                    axis = cylinder.Axis()
                    p, d = axis.Location(), axis.Direction()
                    cylinders.append((cylinder.Radius(), np.array([p.X(), p.Y(), p.Z()]), np.array([d.X(), d.Y(), d.Z()])))
                cache[path] = cylinders
            for cylinder_radius, center, axis in cache[path]:
                if abs(cylinder_radius - wanted_radius) > 1e-5:
                    continue
                distance = float(np.linalg.norm(np.cross(origin - center, axis)))
                if distance > .25:
                    continue
                angle = math.degrees(math.acos(float(np.clip(abs(axis @ expected_axis), 0, 1))))
                measured.append((angle, distance, part["name"]))
        best = min(measured) if measured else None
        return {"radius_mm": wanted_radius, "nearby_cylindrical_faces": len(measured),
                "axis_angle_error_deg": None if best is None else best[0],
                "axis_line_distance_mm": None if best is None else best[1],
                "best_component": None if best is None else best[2],
                "status": "no_nearby_bore_identified" if best is None else
                          "misaligned" if best[0] > .1 or best[1] > .001 else "aligned_at_neutral"}

    for joint in contract["joints"]:
        parents = [p for p in parts if p["body"] == joint["parent_joint"] and "_light_link_" in p["name"]]
        if not parents:
            continue
        T = np.asarray(joint["frame_world"])
        expected_axis = T[:3, :3] @ joint["axis_local"]
        origin = T[:3, 3]
        parent = measure(parents, radius, origin, expected_axis)
        children = [p for p in parts if p["body"] == joint["name"] and "_light_link_" in p["name"]]
        shafts = [p for p in parts if p["body"] == joint["name"] and "_shaft_" in p["name"]]
        measurements.append({"joint": joint["name"], "parent": joint["parent_joint"],
                             "bore_radius_mm": radius, "nearby_cylindrical_bore_faces": parent["nearby_cylindrical_faces"],
                             "axis_angle_error_deg": parent["axis_angle_error_deg"],
                             "axis_line_distance_mm": parent["axis_line_distance_mm"],
                             "best_parent_component": parent["best_component"], "status": parent["status"],
                             "child_bore": measure(children, radius, origin, expected_axis),
                             "shaft": measure(shafts, design["links"]["pin_radius_mm"], origin, expected_axis)})
    return {"joints_examined": len(measurements),
            "misaligned_parent_bores": sum(x["status"] == "misaligned" for x in measurements),
            "unidentified_parent_bores": sum(x["status"] == "no_nearby_bore_identified" for x in measurements),
            "misaligned_child_bores": sum(x["child_bore"]["status"] == "misaligned" for x in measurements),
            "misaligned_shafts": sum(x["shaft"]["status"] == "misaligned" for x in measurements),
            "unidentified_child_bores": sum(x["child_bore"]["status"] == "no_nearby_bore_identified" for x in measurements),
            "unidentified_shafts": sum(x["shaft"]["status"] == "no_nearby_bore_identified" for x in measurements),
            "measurements": measurements,
            "scope": "Saved STEP parent/child cylindrical bores and shaft nominal axes near distal hinge anchors at neutral pose only. "
                     "Agreement within0.1deg/0.001mm is a numerical geometry check, not manufacturing tolerance approval. "
                     "Aligned does not validate bearing fits, pin retention, horn coupling, swept clearance or assembly feasibility."}


def review(root, geometry, simulation, model_frames):
    directory = root / "cad/remote_drive_v1"
    design = read(directory / "design.json")
    baseline = read(root / "cad/modular_roots_v9/design.json")
    contract = read(directory / "simulation_contract.json")
    placements = read(directory / "generated/placements_mass_inertia.json")
    for label, obj in [("design", design), ("contract", contract), ("placements", placements)]:
        finite_tree(obj, label)
    preserved = ["palm_width", "palm_length", "root_axis_offset_z", "fingers"]
    for key in preserved:
        assert design["hand"][key] == baseline["hand"][key], f"Changed baseline geometry: {key}"
    assert contract["actuator_count"] == 20
    joints = {x["name"]: x for x in contract["joints"]}
    bodies = {x["name"]: x for x in contract["bodies"]}
    assert len(joints) == len(contract["joints"]) == 20
    assert len(bodies) == len(contract["bodies"]) == 21
    expected = expected_frames(design)
    assert set(joints) == set(expected)
    assert set(bodies) == set(expected) | {"palm"}
    max_frame_error = 0.0
    for name, e in expected.items():
        j = joints[name]
        assert j["parent_joint"] == bodies[name]["parent_joint"] == e["parent"]
        assert np.array_equal(j["axis_local"], e["axis"]), name
        for matrix in [j["frame_world"], bodies[name]["frame_world"]]:
            max_frame_error = max(max_frame_error, float(np.max(abs(np.asarray(matrix) - e["frame"]))))
        parent_frame = np.eye(4) if e["parent"] == "palm" else expected[e["parent"]]["frame"]
        assert np.allclose(parent_frame @ np.asarray(j["frame_parent"]), e["frame"], atol=1e-9), name
        assert j["range_deg"][0] < j["range_deg"][1], name
    assert max_frame_error < 1e-9, max_frame_error

    parts = placements["parts"]
    assert len({p["name"] for p in parts}) == len(parts)
    motor_names = [p["name"] for p in parts if p["name"].endswith("_motor")]
    assert len(motor_names) == 20, motor_names
    remote = [p for p in parts if "_remote_" in p["name"] and p["name"].endswith("_motor")]
    assert len(remote) == 10
    for p in remote:
        expected_body = "thumb_opposition" if p["name"].startswith("thumb_") else "palm"
        assert p["body"] == expected_body, p["name"]

    cache, remeasured, rows = {}, [], []
    max_volume = max_com = max_tensor = 0.0
    for p in parts:
        assert p["body"] in bodies, p["name"]
        assert p["mass_kg"] >= 0, p["name"]
        if p["mass_kg"] == 0:
            continue
        if geometry:
            volume, com, tensor = part_properties_from_step(root, directory, p, cache)
            volume_error = abs(volume - p["volume_mm3"])
            com_error = float(np.max(abs(com - np.asarray(p["com_world_mm"]))))
            tensor_error = float(np.max(abs(tensor - np.asarray(p["inertia_com_world_kg_m2"]))))
            max_volume = max(max_volume, volume_error)
            max_com = max(max_com, com_error)
            max_tensor = max(max_tensor, tensor_error)
            assert volume_error <= max(1e-4, abs(volume) * 1e-7), p["name"]
            assert com_error < 1e-5, p["name"]
            assert tensor_error < 1e-9, p["name"]
            remeasured.append(p["name"])
        else:
            com = np.asarray(p["com_world_mm"])
            tensor = np.asarray(p["inertia_com_world_kg_m2"])
        tensor_valid(tensor, p["name"])
        rows.append((p, com, tensor))

    max_mass = max_body_com = max_body_tensor = 0.0
    for name, body in bodies.items():
        members = [(p, c, I) for p, c, I in rows if p["body"] == name]
        assert {p["name"] for p, c, I in members} == set(body["component_names"]), name
        mass = sum(p["mass_kg"] for p, c, I in members)
        assert mass > 0, name
        com = sum((p["mass_kg"] * c for p, c, I in members), np.zeros(3)) / mass
        inertia = np.zeros((3, 3))
        for p, c, I in members:
            delta = (c - com) / 1000
            inertia += I + p["mass_kg"] * (delta @ delta * np.eye(3) - np.outer(delta, delta))
        frame_world = np.asarray(body["frame_world"])
        R = frame_world[:3, :3]
        local_com = R.T @ (com - frame_world[:3, 3])
        local_inertia = R.T @ inertia @ R
        tensor_valid(local_inertia, name)
        max_mass = max(max_mass, abs(mass - body["mass_kg"]))
        max_body_com = max(max_body_com, float(np.max(abs(local_com - np.asarray(body["com_local_mm"])))))
        max_body_tensor = max(max_body_tensor, float(np.max(abs(local_inertia - np.asarray(body["inertia_com_local_kg_m2"])))))
    assert max_mass < 1e-9 and max_body_com < 1e-5 and max_body_tensor < 1e-9
    assert abs(sum(p["mass_kg"] for p in parts) - placements["hand_mass_kg"]) < 1e-10

    frozen = read(root / "AUDIT_EVIDENCE_INDEX.json")
    frozen_count = 0
    for group in ["geometry_sha256", "code_sha256", "report_sha256"]:
        for relative, expected_hash in frozen[group].items():
            assert digest(root / relative) == expected_hash, relative
            frozen_count += 1
    generated = read(directory / "generated/build_summary.json")
    assert generated["design_sha256"] == digest(directory / "design.json"), "CAD build is stale relative to design.json"
    assert generated["generator_sha256"] == digest(directory / "generate.py"), "CAD build is stale relative to generate.py"
    git_flags = {"creationflags": subprocess.CREATE_NO_WINDOW} if sys.platform == "win32" else {}
    sha = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, **git_flags).decode().strip()
    dirty = subprocess.check_output(["git", "status", "--porcelain"], cwd=root, **git_flags).decode().strip()
    result = {"target_sha": sha, "clean_checkout_at_review": not dirty,
            "review_tool_sha256": digest(Path(__file__)),
            "review_tool_within_target_checkout": Path(__file__).resolve().is_relative_to(root),
            "frozen_hashes_checked": frozen_count, "preserved_dimensions": preserved,
            "joint_count": len(joints), "motor_count": len(motor_names),
            "relocated_motor_count": len(remote), "joint_frame_max_error_mm_or_unitless": max_frame_error,
            "body_count": len(bodies), "massive_component_count": len(rows),
            "saved_STEP_remeasured_component_count": len(remeasured), "unique_STEP_files_read": len(cache),
            "part_max_volume_error_mm3": max_volume, "part_max_com_error_mm": max_com,
            "part_max_inertia_error_kg_m2": max_tensor, "body_max_mass_error_kg": max_mass,
            "body_max_com_error_mm": max_body_com, "body_max_inertia_error_kg_m2": max_body_tensor,
            "hand_assumed_mass_kg": placements["hand_mass_kg"], "numerical_contract_consistency_passed": True,
            "manufacturing_release": False, "physical_latency_validated": False,
            "scope": ("Independent saved STEP mass-property remeasurement and contract aggregation; "
                      if geometry else "Recorded component-to-body contract aggregation; no STEP remeasurement; ") +
                     "preserved geometry inputs, declared joint frames, motor body membership, positive inertias and frozen evidence. "
                     "Complete mating-bore/pin alignment and assembly feasibility are not established. "
                     "No whole-range collision, cable-routing validation, motor calibration or hardware performance approval."}
    if simulation:
        result["simulation_report_review"] = review_simulation(root, contract, design)
    if model_frames:
        result["compiled_model_frame_review"] = review_model_frames(root, contract)
    if geometry:
        result["actual_parent_bore_review"] = review_parent_bores(directory, contract, parts, design)
        result["physical_assembly_envelope_review"] = review_physical_envelope(root, directory, parts)
        result["mechanical_blockers_detected"] = [x for x in result["actual_parent_bore_review"]["measurements"]
                                                 if any(y != "aligned_at_neutral" for y in
                                                        [x["status"], x["child_bore"]["status"], x["shaft"]["status"]])]
        result["mechanical_blocker_list_scope"] = "The legacy field above lists nominal hinge-axis defects only; an empty list is not construction approval."
        result["unresolved_construction_scope"] = [
            "Rigid motor-to-body attachment is a dynamics assumption; new raised or rotated motor plates require physical supports, fasteners and verified load paths.",
            "Nominal bore and pin axes do not establish fits, retention, horn engagement, structural strength or access for assembly.",
            "Continuous swept clearance and functional tendon routing, wraps, tensioners and terminals remain unvalidated."]
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--output", type=Path)
    parser.add_argument("--geometry", action="store_true", help="Reread saved STEP using CadQuery/OCP")
    parser.add_argument("--simulation", action="store_true", help="Audit committed report arithmetic and CAD integration")
    parser.add_argument("--model-frames", action="store_true", help="Check compiled MuJoCo frames/inertias at 13 poses")
    args = parser.parse_args()
    result = review(args.root.resolve(), args.geometry, args.simulation, args.model_frames)
    text = json.dumps(result, indent=2, allow_nan=False)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")
    print(text)
