"""Render the wrist-directed thumb candidate using unchanged saved CAD meshes.

No dynamics, collision calculation, source mutation or child process is started.
Run with pythonw in a hidden process; --output must name a new directory.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import importlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
for _key in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_key, "1")
sys.dont_write_bytecode = True

import numpy as np
from PIL import Image, ImageDraw

NOTICE = "각도경로 미리보기 · 재생시간은 실물속도 아님 · 연속충돌 보증 아님"
ARROW_COLOR = (166, 74, 13)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class Inputs:
    def __init__(self, repo):
        self.repo = Path(repo).resolve()
        self.hashes = {}

    def add(self, path, expected=None):
        path = Path(path).resolve()
        if not path.is_relative_to(self.repo) or not path.is_file():
            raise ValueError("Input is missing or outside the repository: " + str(path))
        key = path.relative_to(self.repo).as_posix()
        actual = sha(path)
        if expected is not None and actual != expected:
            raise ValueError("Stale input: " + key)
        if key in self.hashes and self.hashes[key] != actual:
            raise ValueError("Input changed while loading: " + key)
        self.hashes[key] = actual
        return path

    def verify(self):
        return {key: (self.repo / key).is_file() and sha(self.repo / key) == value
                for key, value in self.hashes.items()}


def load_profile(repo, inputs):
    source = inputs.add(repo / "cad/thumb_wrist_v1/wrist_profile.py")
    spec_path = inputs.add(repo / "cad/thumb_wrist_v1/profile.json")
    specification = json.loads(spec_path.read_text(encoding="utf-8"))
    base = (repo / specification["base_candidate"]).resolve()
    for relative in ("poses.py", "poses.json", "design.json", "generated/scene.json",
                     "render_gestures.py"):
        inputs.add(base / relative)
    spec = importlib.util.spec_from_file_location("_thumb_wrist_render_profile", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    loaded = module.load(repo)
    renderer = importlib.import_module("render_gestures")
    if Path(loaded["poses"].__file__).resolve() != base / "poses.py":
        raise ValueError("Profile imported poses from a different checkout")
    if Path(renderer.__file__).resolve() != base / "render_gestures.py":
        raise ValueError("Renderer imported from a different checkout")
    if renderer.ROOT.resolve() != repo:
        raise ValueError("Renderer's repository root differs from --repo")
    for relative, expected in loaded["catalog"]["reference"]["source_sha256"].items():
        inputs.add(repo / relative, expected)
    return loaded, renderer


def preview_frames(document, poses, design):
    """Keep all 33 finger samples, then interpolate only the final IP leg."""
    if len(document["transitions"]) != 1:
        raise ValueError("Expected one explicit two-phase transition")
    path = document["transitions"][0]
    points = path["waypoints"]
    if len(points) != 34:
        raise ValueError("Expected 33 authored finger frames plus the thumb target")
    fractions = np.asarray([p["fraction"] for p in points], float)
    if (not np.isfinite(fractions).all() or fractions[0] != 0 or fractions[-1] != 1
            or np.any(np.diff(fractions) <= 0)):
        raise ValueError("Preview fractions must strictly increase from zero to one")
    frames = []
    for index, point in enumerate(points[:-1]):
        q = poses.validate_angles(point["angles_deg"], design)
        if not np.array_equal(q[16:], np.zeros(4)):
            raise ValueError("The authored finger phase moved the thumb")
        frames.append(dict(fraction=float(point["fraction"]), angles_deg=q.tolist(),
                           waypoint_index=index, phase=point["phase"]))
    a, b = points[-2:]
    qa = np.asarray(a["angles_deg"], float)
    qb = poses.validate_angles(b["angles_deg"], design)
    if not np.array_equal(qa[:19], qb[:19]) or qb[19] != -15.:
        raise ValueError("The final leg must change only thumb IP to -15 degrees")
    count = max(1, int(math.ceil(abs(qb[19] - qa[19]) / 3.)))
    for k in range(1, count + 1):
        u = k / count
        q = qb.copy() if k == count else qa + u * (qb - qa)
        poses.validate_angles(q, design)
        frames.append(dict(fraction=float(b["fraction"] if k == count else
                           a["fraction"] + u * (b["fraction"] - a["fraction"])),
                           angles_deg=q.tolist(), waypoint_index=33 if k == count else None,
                           phase=b["phase"], interpolation_fraction=u))
    if {f["waypoint_index"] for f in frames if f["waypoint_index"] is not None} != set(range(34)):
        raise ValueError("Preview omitted an authored waypoint")
    endpoints = {g["id"]: g["angles_deg"] for g in document["gestures"]}
    if (frames[0]["angles_deg"] != endpoints[path["source"]] or
            frames[-1]["angles_deg"] != endpoints[path["target"]]):
        raise ValueError("Preview endpoint differs from named profile gesture")
    return frames


def camera_fit(renderer, assembly, samples, normal, annotation_points):
    camera = renderer.view_fit([assembly], samples, normal, 1.)
    right, up = np.asarray(camera["right"]), np.asarray(camera["up"])
    n = np.cross(right, up)
    basis = np.column_stack((right, up, n))
    low, high = np.asarray(camera["projected_union_bounds_mm"])
    marks = np.asarray(annotation_points) @ basis
    low = np.minimum(low, marks.min(axis=0))
    high = np.maximum(high, marks.max(axis=0))
    focus = basis @ ((low + high) / 2)
    distance = max(float(np.linalg.norm(high - low)), 1.) * 3
    camera.update(focus=focus.tolist(), position=(focus + n * distance).tolist(),
                  parallel_scale_mm=float(max((high - low)[:2]) * .60),
                  projected_union_bounds_mm=[low.tolist(), high.tolist()],
                  framing_margin_fraction=.10, annotation_points_in_fit=True)
    return camera


def project(point, camera, size):
    delta = np.asarray(point) - camera["focus"]
    px_per_mm = size / (2 * camera["parallel_scale_mm"])
    return np.array([(size - 1) / 2 + float(delta @ camera["right"]) * px_per_mm,
                     (size - 1) / 2 - float(delta @ camera["up"]) * px_per_mm])


def annotate(tile, camera, wrist, arrow_tip):
    """Screen overlay only: a mechanical datum and its projected -Y direction."""
    image = tile.copy()
    draw = ImageDraw.Draw(image)
    a, b = project(wrist, camera, tile.width), project(arrow_tip, camera, tile.width)
    if min(*a, *b, tile.width - 1 - a[0], tile.height - 1 - a[1],
           tile.width - 1 - b[0], tile.height - 1 - b[1]) < 14:
        raise ValueError("Wrist annotation would be clipped")
    vector = (b - a) / np.linalg.norm(b - a)
    side = np.array([-vector[1], vector[0]])
    draw.line([tuple(a), tuple(b)], fill=ARROW_COLOR, width=max(3, tile.width // 200))
    draw.polygon([tuple(b), tuple(b - 12 * vector + 6 * side),
                  tuple(b - 12 * vector - 6 * side)], fill=ARROW_COLOR)
    draw.ellipse((a[0] - 5, a[1] - 5, a[0] + 5, a[1] + 5),
                 fill=ARROW_COLOR, outline=(255, 255, 255), width=2)
    return image, dict(wrist_datum_px=a.tolist(), wrist_direction_arrow_tip_px=b.tolist(),
                       overlay_is_not_CAD_geometry=True, depth_occlusion_not_applied=True)


def text_fit(renderer, draw, xy, value, width, size):
    while size > 12 and draw.textbbox((0, 0), value, font=renderer.font(size))[2] > width:
        size -= 1
    font = renderer.font(size)
    if draw.textbbox((0, 0), value, font=font)[2] > width:
        raise ValueError("Caption exceeds available width")
    draw.text(xy, value, font=font, fill=renderer.INK)


def sheet(renderer, tiles, columns, size, title, subtitle):
    rows = math.ceil(len(tiles) / columns)
    header, label, footer = 100, 43, 102
    result = Image.new("RGB", (columns * size, header + rows * (size + label) + footer), renderer.BG)
    draw = ImageDraw.Draw(result)
    text_fit(renderer, draw, (22, 13), title, result.width - 44, 31)
    text_fit(renderer, draw, (22, 58), subtitle, result.width - 44, 22)
    for i, (caption, image) in enumerate(tiles):
        x, y = (i % columns) * size, header + (i // columns) * (size + label)
        text_fit(renderer, draw, (x + 14, y + 8), caption, size - 28, 24)
        result.paste(image, (x, y + label))
    text_fit(renderer, draw, (22, result.height - 87),
             "주황 점: 손목 부착판 기준 · 화살표: 팔목 방향(-Y) · 엄지 끝이 손목을 직접 가리킨다는 뜻은 아님",
             result.width - 44, 21)
    text_fit(renderer, draw, (22, result.height - 47),
             "기존 349개 CAD 형상 유지 · IP 음각은 시험 범위 · 실물 가동/제작 승인 아님",
             result.width - 44, 21)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=HERE.parents[1])
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    repo, output = args.repo.resolve(), args.output.resolve()
    if os.path.lexists(args.output) or output.exists():
        parser.error("Use a new output directory; previous evidence is preserved")
    if not Path(__file__).resolve().is_relative_to(repo):
        parser.error("Run the render.py inside the selected --repo checkout")
    inputs = Inputs(repo)
    inputs.add(__file__)
    loaded, renderer = load_profile(repo, inputs)
    base, poses, catalog = loaded["base"], loaded["poses"], loaded["catalog"]
    if output.is_relative_to(base):
        parser.error("Output must not be placed inside the frozen base CAD directory")
    assembly = renderer.load_candidate(base, catalog, inputs)
    if len(assembly["parts"]) != 349:
        raise ValueError("Expected the unchanged 349-part saved hand")
    original, design = loaded["original_design"], loaded["design"]
    comparison_design = copy.deepcopy(design)
    comparison_design["joint_ranges_deg"]["thumb_ip"] = original["joint_ranges_deg"]["thumb_ip"]
    if comparison_design != original or assembly["design"] != original:
        raise ValueError("Profile changed geometry/axes rather than the explicit IP range")
    assembly["design"] = design
    gestures = loaded["document"]["gestures"]
    if [g["id"] for g in gestures] != ["open", "fingers_folded", "thumb_wrist"]:
        raise ValueError("Unexpected profile gesture order")
    frames = preview_frames(loaded["document"], poses, design)
    wrist = np.asarray(loaded["direction"]["wrist_reference_mm"], float)
    direction = np.asarray(loaded["direction"]["wrist_direction_world"], float)
    if not np.array_equal(direction, [0, -1, 0]):
        raise ValueError("The annotation must use the declared hand-longitudinal -Y direction")
    arrow_tip = wrist + 30 * direction
    compare = gestures[1:]
    cameras = {name: camera_fit(renderer, assembly, compare, normal, [wrist, arrow_tip])
               for name, normal in renderer.VIEWS.items()}
    scale = max(c["parallel_scale_mm"] for c in cameras.values())
    for camera in cameras.values():
        camera["parallel_scale_mm"] = scale
        camera["same_scale_for_comparison_and_candidate"] = True
    gif_cameras = {"oblique": camera_fit(renderer, assembly, frames, renderer.VIEWS["oblique"], [wrist, arrow_tip])}
    gif_assembly = dict(assembly)
    import mujoco
    output.mkdir(parents=True, exist_ok=False)
    completed = []
    records = []
    report_path = output / "render_report.json"
    try:
        cache, tiles = {}, []
        for view in cameras:
            for gesture in compare:
                image, bounds = renderer.render_one(assembly, gesture["angles_deg"], view, cameras, 900, mujoco)
                annotated, overlay = annotate(image, cameras[view], wrist, arrow_tip)
                cache[view, gesture["id"]] = annotated
                label = "끝마디 0° · 엄지 유지" if gesture["id"] == "fingers_folded" else "끝마디만 팔목 쪽으로 15°"
                tiles.append((renderer.VIEW_LABELS[view] + " · " + label, annotated))
                records.append(dict(view=view, pose=gesture["id"], angles_deg=gesture["angles_deg"],
                                    foreground_bounds_px=bounds, annotation=overlay))
        sheet(renderer, tiles, 2, 900, "엄지 끝마디만 팔목 방향으로 젖히기",
              "같은 손·네 손가락 주먹·카메라·축척 / 엄지 기저와 중간마디는 편손 위치 유지").save(output / "comparison.png")
        completed.append("comparison.png")
        wristward_mm = -float(loaded["direction"]["tip_delta_world_mm"][1])
        sheet(renderer, [("현재 후보 · 사선", cache["oblique", "thumb_wrist"])], 1, 900,
              "엄지 끝마디만 팔목 쪽으로 15°",
              f"손 길이 방향으로 끝점이 팔목 쪽 약 {wristward_mm:.1f}mm 이동").save(output / "candidate.png")
        completed.append("candidate.png")
        animated = []
        for index, sample in enumerate(frames):
            image, bounds = renderer.render_one(gif_assembly, sample["angles_deg"], "oblique", gif_cameras, 640, mujoco)
            image, overlay = annotate(image, gif_cameras["oblique"], wrist, arrow_tip)
            canvas = Image.new("RGB", (640, 784), renderer.BG)
            draw = ImageDraw.Draw(canvas)
            phase = "엄지 유지 · 네 손가락만 쥠" if sample["angles_deg"][19] == 0 else "끝마디만 팔목 쪽으로 젖힘"
            text_fit(renderer, draw, (14, 9), phase, 612, 24)
            text_fit(renderer, draw, (14, 43), f"그림 {index + 1}/{len(frames)} · 경로 {sample['fraction'] * 100:.1f}% · IP {sample['angles_deg'][19]:g}°", 612, 18)
            canvas.paste(image, (0, 72))
            text_fit(renderer, draw, (14, 720), "주황 점: 손목 기준 · 화살표: 팔목 방향(-Y)", 612, 18)
            text_fit(renderer, draw, (14, 751), NOTICE, 612, 16)
            animated.append(canvas)
            records.append(dict(kind="motion", frame=index, sample=sample,
                                foreground_bounds_px=bounds, annotation=overlay))
        durations = [120] * len(animated)
        durations[-1] = 800
        animated[0].save(output / "motion.gif", save_all=True, append_images=animated[1:],
                         duration=durations, loop=0, optimize=False, disposal=2)
        completed.append("motion.gif")
        unchanged = inputs.verify()
        if not all(unchanged.values()):
            raise ValueError("Inputs changed: " + str([k for k, v in unchanged.items() if not v]))
        report = dict(schema_version="thumb_wrist_v1.render.v1", status="completed", completed=True,
            inputs_unchanged=True, source_sha256=inputs.hashes,
            output_sha256={name: sha(output / name) for name in completed},
            candidate_part_count=349, geometry_changed=False, canonical_pose_files_modified=False,
            original_thumb_ip_range_deg=original["joint_ranges_deg"]["thumb_ip"],
            exploratory_thumb_ip_range_deg=design["joint_ranges_deg"]["thumb_ip"],
            explicit_override="Only the in-memory render angle range changes; original saved CAD and pose sources remain unchanged.",
            gestures=gestures, direction=loaded["direction"], cameras=cameras, motion_cameras=gif_cameras,
            annotation=dict(wrist_reference_mm=wrist.tolist(), arrow_tip_mm=arrow_tip.tolist(),
                            arrow_length_mm=30, world_direction=[0, -1, 0], overlay_only=True,
                            reference_meaning=loaded["direction"]["reference_meaning"]),
            motion=dict(transition=loaded["document"]["transitions"][0], frames=frames,
                        authored_finger_frames_preserved=33, all_profile_waypoints_preserved=True,
                        frame_count=len(frames), duration_ms=durations,
                        timing_is_physical=False, maximum_extra_ip_step_deg=3),
            renderer=dict(name="MuJoCo/Pillow", mujoco_version=mujoco.__version__,
                          off_screen=True, simulation_steps=0, opaque_parts=True, no_parts_hidden=True,
                          actual_saved_meshes=True, annotations_are_screen_overlays=True,
                          still_tile_size_px=900, gif_tile_size_px=640),
            renders=records, limitations=[
                "Images show exact source mesh shapes under rigid FK; rendering is not collision testing.",
                "The -15-degree IP target is outside the original declared range; physical hinge ROM is unapproved.",
                "The thumb tip moves wristward but does not directly point at the wrist datum.",
                "The 33 accepted finger waypoints are retained; GIF playback and added IP interpolation are not measured physical timing.",
                "No full-path, continuous clearance, strength, motor, tendon, or latency approval is inferred."])
        renderer.write_json(report_path, report)
        print(json.dumps(dict(status="completed", output=str(output), frames=len(frames), inputs_unchanged=True)), flush=True)
    except Exception as error:
        renderer.write_json(report_path, dict(schema_version="thumb_wrist_v1.render.v1", status="failed", completed=False,
            error=repr(error), inputs_unchanged=all(inputs.verify().values()), source_sha256=inputs.hashes,
            output_sha256={name: sha(output / name) for name in completed}))
        raise
    finally:
        for item in (assembly, gif_assembly):
            if "_render_state" in item:
                item["_render_state"]["renderer"].close()


if __name__ == "__main__":
    main()
