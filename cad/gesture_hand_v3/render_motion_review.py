"""Render actual saved CAD poses and three geometric paths, without simulation.

Run in a hidden Python process after the assembly, pose catalog and path catalog
are frozen. This script starts no child processes, opens no GUI and requires a
new output directory. GIF playback is an arbitrary preview, not physical time.
"""
from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import sys

for _key in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_key, "1")
sys.dont_write_bytecode = True

import numpy as np
from PIL import Image, ImageDraw

import poses
import render_gestures as render

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
PATHS = (("open__fist", "open", "fist", "보 → 바위"),
         ("open__thumb_up", "open", "thumb_up", "보 → 엄지척"),
         ("fist__thumb_up", "fist", "thumb_up", "바위 → 엄지척"))
NOTICE = "각도경로 미리보기 · 재생시간은 실물속도 아님 · 연속충돌 보증 아님"


class RepoInputs(render.Inputs):
    """Keep every source key reproducible from a repository-relative path."""

    def add(self, path, expected=None):
        path = Path(path).resolve()
        if not path.is_relative_to(ROOT):
            raise ValueError("A render input is outside the repository: " + str(path))
        return super().add(path, expected)


def repository_path(value):
    path = Path(value)
    path = path.resolve() if path.is_absolute() else (ROOT / path).resolve()
    if not path.is_relative_to(ROOT):
        raise ValueError("Source path escapes the repository: " + str(value))
    return path


def load_paths(path, catalog, inputs):
    document = render.read_json(inputs.add(path))
    for key in ("joint_names", "kinematic_design", "reference"):
        if document[key] != catalog[key]:
            raise ValueError("Core-path and pose catalogs disagree: " + key)
    if document.get("units") != "degrees":
        raise ValueError("Core-path angles must use degrees")
    expected_ids = ["open", "fist", "thumb_up"]
    if [g["id"] for g in document["gestures"]] != expected_ids:
        raise ValueError("Expected the three named core gestures, in canonical order")
    for gesture in document["gestures"]:
        if not np.array_equal(gesture["angles_deg"], poses.pose(gesture["id"], catalog)):
            raise ValueError("Core-path endpoint differs from canonical pose: " + gesture["id"])
    if [p["id"] for p in document["transitions"]] != [p[0] for p in PATHS]:
        raise ValueError("Expected exactly the three forward core paths")
    for transition, (ident, source, target, _) in zip(document["transitions"], PATHS):
        if (transition["source"], transition["target"]) != (source, target):
            raise ValueError("Core-path source/target mismatch: " + ident)
        points = transition["waypoints"]
        if not 2 <= len(points) <= 257:
            raise ValueError("A bounded preview requires 2..257 waypoints")
        fractions = np.array([p["fraction"] for p in points], dtype=float)
        if (not np.isfinite(fractions).all() or fractions[0] != 0. or
                fractions[-1] != 1. or np.any(np.diff(fractions) <= 0)):
            raise ValueError("Path fractions must increase strictly from zero to one")
        for point in points:
            poses.validate_angles(point["angles_deg"], catalog["kinematic_design"])
        if (points[0]["angles_deg"] != poses.pose(source, catalog).tolist() or
                points[-1]["angles_deg"] != poses.pose(target, catalog).tolist()):
            raise ValueError("Path endpoint angles differ from named gestures: " + ident)
    return document


def at_fraction(transition, fraction):
    """Copy exact waypoints; interpolate only inside a declared geometric leg."""
    points = transition["waypoints"]
    f = float(fraction)
    if not math.isfinite(f) or not 0 <= f <= 1:
        raise ValueError("Preview fraction is outside [0, 1]")
    for index, point in enumerate(points):
        if f == float(point["fraction"]):
            return dict(fraction=f, angles_deg=list(point["angles_deg"]),
                        waypoint_index=index, phase=point.get("phase"),
                        segment_index=max(0, index - 1),
                        segment_fraction=0. if index == 0 else 1.)
    for index, (a, b) in enumerate(zip(points, points[1:])):
        if a["fraction"] < f < b["fraction"]:
            u = (f - a["fraction"]) / (b["fraction"] - a["fraction"])
            q = (1. - u) * np.asarray(a["angles_deg"]) + u * np.asarray(b["angles_deg"])
            return dict(fraction=f, angles_deg=q.tolist(), waypoint_index=None,
                        phase="interpolated", segment_index=index, segment_fraction=float(u))
    raise ValueError("No enclosing path segment")


def fill_fractions(fractions, count):
    """Retain authored fractions and split the largest gaps deterministically."""
    result = sorted(set(float(f) for f in fractions))
    while len(result) < count:
        index = max(range(len(result) - 1), key=lambda i: (result[i + 1] - result[i], -i))
        midpoint = (result[index] + result[index + 1]) / 2
        if midpoint == result[index] or midpoint == result[index + 1]:
            raise ValueError("Path fractions cannot support another distinct preview sample")
        result.insert(index + 1, midpoint)
    return result


def preview_samples(transition, requested_frames):
    fractions = [p["fraction"] for p in transition["waypoints"]]
    frames = [at_fraction(transition, f) for f in fill_fractions(fractions, requested_frames)]
    # A 33-waypoint authored fist produces 33 frames, not a silently shortened path.
    represented = {p["waypoint_index"] for p in frames if p["waypoint_index"] is not None}
    if represented != set(range(len(fractions))):
        raise ValueError("A GIF omitted an authored waypoint")
    if len(fractions) <= 5:
        contact_fractions = fill_fractions(fractions, 5)
    else:
        # The contact sheet is a selected illustration, never the complete path.
        indices = [round(k * (len(fractions) - 1) / 4) for k in range(5)]
        contact_fractions = [fractions[i] for i in indices]
    contact = [at_fraction(transition, f) for f in contact_fractions]
    return frames, contact


def audit_attachment(path, paths_file, inputs):
    if not path.exists():
        return dict(audit_attached=False, reason="optional_report_absent",
                    continuous_collision_guarantee=False, rendering_is_collision_test=False)
    report = render.read_json(inputs.add(path))
    guard = report.get("input_hash_guard", {})
    if (report.get("completed") is not True or guard.get("passed") is not True or
            not guard.get("start") or guard.get("start") != guard.get("end")):
        raise ValueError("Present transition audit is incomplete or has a failed input guard")
    if repository_path(report["poses_source"]) != paths_file:
        raise ValueError("Transition audit used a different path catalog")
    guarded = {repository_path(k): v for k, v in guard["start"].items()}
    if paths_file not in guarded or guarded[paths_file] != render.sha(paths_file):
        raise ValueError("Transition audit does not bind the current path-catalog hash")
    for source, expected in guarded.items():
        inputs.add(source, expected)
    if (report.get("mode") != "full" or
            [p["id"] for p in report.get("transitions", [])] != [p[0] for p in PATHS]):
        raise ValueError("Transition audit does not cover the three forward paths")
    cases = report.get("cases", {})
    return dict(audit_attached=True, path=path.relative_to(ROOT).as_posix(),
                sha256=render.sha(path), completed=report["completed"],
                poses_source=paths_file.relative_to(ROOT).as_posix(),
                poses_sha256=render.sha(paths_file), thresholds=report.get("thresholds"),
                summary=report.get("summary"), report_errors=report.get("errors", []),
                unique_case_count=len(cases),
                cases_with_reported_collision=sum(bool(c["collision_count"]) for c in cases.values()),
                cases_with_operation_failure=sum(bool(c["operation_failures"]) for c in cases.values()),
                continuous_collision_guarantee=False, rendering_is_collision_test=False,
                meaning="Attached sampled CAD audit only; its failures remain visible. The rendered frames are not its sampling grid.")


def image_text(draw, position, text, max_width, size, minimum=13):
    """Fit one caption line instead of clipping a longer Korean label."""
    selected = render.font(size)
    while size > minimum and draw.textbbox((0, 0), text, font=selected)[2] > max_width:
        size -= 1
        selected = render.font(size)
    if draw.textbbox((0, 0), text, font=selected)[2] > max_width:
        raise ValueError("Caption does not fit its declared panel")
    draw.text(position, text, fill=render.INK, font=selected)


def make_sheet(tiles, columns, size, title, subtitle, footer):
    rows = (len(tiles) + columns - 1) // columns
    head, label_height, foot = 94, 42, 68
    width = columns * size
    result = Image.new("RGB", (width, head + rows * (size + label_height) + foot), render.BG)
    draw = ImageDraw.Draw(result)
    image_text(draw, (22, 12), title, width - 44, 30)
    image_text(draw, (22, 53), subtitle, width - 44, 20)
    for index, (label, tile) in enumerate(tiles):
        x, y = (index % columns) * size, head + (index // columns) * (size + label_height)
        image_text(draw, (x + 13, y + 8), label, size - 26, 23)
        if tile.size != (size, size):
            tile = tile.resize((size, size), Image.Resampling.LANCZOS)
        result.paste(tile, (x, y + label_height))
    image_text(draw, (22, result.height - foot + 12), footer, width - 44, 20)
    return result


def gif_frame(tile, label, sample, size):
    canvas = Image.new("RGB", (size, size + 112), render.BG)
    draw = ImageDraw.Draw(canvas)
    image_text(draw, (14, 10), label + " · 각도경로 미리보기", size - 28, 24)
    image_text(draw, (14, 43), "경로 위치 " + format(sample["fraction"] * 100, ".1f") + "% · 사선", size - 28, 18)
    canvas.paste(tile.resize((size, size), Image.Resampling.LANCZOS), (0, 70))
    image_text(draw, (14, size + 79), "재생시간은 실물속도 아님 · 연속충돌 보증 아님", size - 28, 17)
    return canvas


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cad-root", type=Path, default=HERE)
    parser.add_argument("--poses", type=Path, default=HERE / "poses.json")
    parser.add_argument("--paths", type=Path, default=HERE / "core_paths.json")
    parser.add_argument("--audit-report", type=Path, default=HERE / "reviews/core_transitions.json")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--tile-size", type=int, default=850)
    parser.add_argument("--gif-size", type=int, default=640)
    parser.add_argument("--gif-frames", type=int, default=31)
    parser.add_argument("--frame-ms", type=int, default=120)
    args = parser.parse_args()
    if not 400 <= args.tile_size <= 1200 or not 320 <= args.gif_size <= 850:
        parser.error("tile-size must be 400..1200; gif-size must be 320..850")
    if not 21 <= args.gif_frames <= 31 or not 40 <= args.frame_ms <= 500:
        parser.error("gif-frames must be 21..31; frame-ms must be 40..500")
    output = args.output.resolve()
    if os.path.lexists(args.output):
        parser.error("Use a new output directory; previous evidence is preserved")
    if output.is_relative_to(args.cad_root.resolve()):
        parser.error("Output must be outside the CAD input directory")

    inputs = RepoInputs()
    for source in (__file__, poses.__file__, render.__file__):
        inputs.add(source)
    catalog = poses.load_catalog(inputs.add(args.poses))
    for relative, expected in catalog["reference"]["source_sha256"].items():
        inputs.add(repository_path(relative), expected)
    paths_file = inputs.add(args.paths)
    path_catalog = load_paths(paths_file, catalog, inputs)
    candidate = render.load_candidate(args.cad_root.resolve(), catalog, inputs)
    audit = audit_attachment(args.audit_report.resolve(), paths_file, inputs)
    old = poses.validate_angles(catalog["reference"]["authored_fist_deg"], candidate["design"]).copy()
    old[16:] = [-10., 20., 40., 0.]
    new = poses.pose("thumb_up", catalog)
    comparison = [dict(id="GESTURE001_previous", label="이전 · 마디를 굽힌 엄지", angles_deg=old.tolist()),
                  dict(id="current_thumb_up", label="현재 · 뿌리부터 편 엄지", angles_deg=new.tolist())]
    plans = []
    for transition, (_, _, _, label) in zip(path_catalog["transitions"], PATHS):
        frames, contact = preview_samples(transition, args.gif_frames)
        for sample in frames + contact:
            poses.validate_angles(sample["angles_deg"], candidate["design"])
        plans.append(dict(id=transition["id"], label=label, method=transition.get("method"),
                          source_status=transition.get("status"), waypoints=transition["waypoints"],
                          frames=frames, contact_frames=contact,
                          every_authored_waypoint_retained_in_gif=True,
                          exceeds_requested_frame_count_to_preserve_waypoints=len(frames) > args.gif_frames,
                          contact_sheet_is_selected_preview_only=True))
    fit_poses = comparison + [sample for plan in plans for sample in plan["frames"] + plan["contact_frames"]]
    cameras = {name: render.view_fit([candidate], fit_poses, normal, 1.)
               for name, normal in render.VIEWS.items()}
    shared_scale = max(camera["parallel_scale_mm"] for camera in cameras.values())
    for camera in cameras.values():
        camera["parallel_scale_mm"] = shared_scale
        camera["same_scale_across_all_views_and_paths"] = True

    import mujoco
    output.mkdir(parents=True, exist_ok=False)
    outputs = []
    records = []
    try:
        tiles = []
        for view in cameras:
            for gesture in comparison:
                image, bounds = render.render_one(candidate, gesture["angles_deg"], view, cameras, args.tile_size, mujoco)
                tiles.append((render.VIEW_LABELS[view] + " · " + gesture["label"], image))
                records.append(dict(kind="comparison", pose=gesture["id"], view=view,
                                    angles_deg=gesture["angles_deg"], foreground_bounds_px=bounds))
        name = "thumb_up_previous_vs_current.png"
        make_sheet(tiles, 2, args.tile_size, "엄지척 · 이전 자세와 현재 후보",
                   "같은 저장 CAD·카메라·축척 / 엄지는 곧게 펴고, 검지 자세를 함께 조정",
                   "정적 그림은 경로·접촉·구동 성능을 판정하지 않습니다.").save(output / name)
        outputs.append(name)
        for view in cameras:
            tiles = []
            for plan in plans:
                for sample in plan["contact_frames"]:
                    image, bounds = render.render_one(candidate, sample["angles_deg"], view, cameras, args.tile_size, mujoco)
                    tiles.append((plan["label"] + " · " + format(sample["fraction"] * 100, ".1f") + "%", image))
                    records.append(dict(kind="contact_sheet", path=plan["id"], view=view,
                                        sample=sample, foreground_bounds_px=bounds))
            name = "core_paths_" + view + ".png"
            make_sheet(tiles, 5, 430, "세 전환 · 각 경로의 대표 다섯 자세 · " + render.VIEW_LABELS[view],
                       "각 행은 한 경로 / 전체 경유점은 GIF와 보고서에 보존 / 모든 경로에 같은 카메라·축척",
                       NOTICE).save(output / name)
            outputs.append(name)
        for plan in plans:
            frames = []
            for sample in plan["frames"]:
                image, bounds = render.render_one(candidate, sample["angles_deg"], "oblique", cameras, args.tile_size, mujoco)
                frames.append(gif_frame(image, plan["label"], sample, args.gif_size))
                records.append(dict(kind="gif", path=plan["id"], view="oblique",
                                    sample=sample, foreground_bounds_px=bounds))
            name = plan["id"] + ".gif"
            frames[0].save(output / name, save_all=True, append_images=frames[1:],
                           duration=args.frame_ms, loop=0, disposal=2, optimize=False)
            # A nonuniform geometric sample list is deliberately not a timing law.
            plan["gif"] = dict(file=name, frame_count=len(frames), frame_duration_ms=args.frame_ms,
                               displayed_cycle_ms=len(frames) * args.frame_ms,
                               timing="Uniform display-only frame duration, unrelated to physical joint speed or acceleration")
            outputs.append(name)
            print(plan["id"] + ": rendered " + str(len(frames)) + " preview frames", flush=True)
        unchanged = inputs.verify()
        if not all(unchanged.values()):
            raise ValueError("Inputs changed during rendering: " + str([k for k, ok in unchanged.items() if not ok]))
        report = dict(schema_version="gesture_hand_v3.motion_render.v1", status="completed",
                      inputs_unchanged=True, source_sha256=inputs.hashes,
                      output_sha256={name: render.sha(output / name) for name in outputs},
                      cad_stage=candidate["stage"], candidate_part_count=len(candidate["parts"]),
                      comparison_poses=comparison, current_minus_previous_angles_deg=(new - old).tolist(),
                      comparison_scope="Same current saved CAD; GESTURE001 angles reconstructed from original fist first16 and thumb[-10,20,40,0]",
                      path_catalog=paths_file.relative_to(ROOT).as_posix(), paths=plans, renders=records,
                      audit_attachment=audit, cameras=cameras,
                      renderer=dict(name="MuJoCo/Pillow", mujoco_version=mujoco.__version__, off_screen=True,
                                    simulation_steps=0, render_size_px=args.tile_size, gif_hand_size_px=args.gif_size,
                                    opaque_parts=True, no_parts_hidden=True, synthetic_geometry=False),
                      limitations=["Actual saved CAD triangles are moved by FK; no generative or replacement geometry.",
                                   "GIF time is arbitrary display time, not measured or simulated robot speed.",
                                   "Every authored waypoint is retained; interpolation is piecewise linear in joint angles.",
                                   "Five contact-sheet frames are selected illustrations, not a collision sampling certificate.",
                                   "A matching sampled collision report is attached only when present and hash-validated.",
                                   "Neither these images nor a sampled audit prove continuous clearance, cable routing, motor performance or manufacturing readiness.",
                                   "No reverse-direction animation or dynamics run is performed."])
        render.write_json(output / "render_report.json", report)
        print("Completed actual-CAD motion preview; all repository input hashes unchanged.", flush=True)
    except Exception as error:
        render.write_json(output / "render_failure.json", dict(status="failed", error=repr(error),
                          source_sha256=inputs.hashes, completed_outputs=outputs))
        raise
    finally:
        if "_render_state" in candidate:
            candidate["_render_state"]["renderer"].close()


if __name__ == "__main__":
    main()
