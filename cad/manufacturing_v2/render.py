"""Render the saved MANUFACTURE002 assembly and its actual retention parts.

No CAD is built, cut or replaced. The close-up separates existing meshes only
for display. Run after the guarded build has completed, in a hidden process.
This module starts no child process and performs no dynamics steps.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys

for _key in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_key, "1")
sys.dont_write_bytecode = True

import numpy as np
from PIL import Image, ImageDraw

HERE = Path(__file__).resolve().parent
DIGITS = ("index", "middle", "ring", "little", "thumb")
PIN = "thumb_joint_pin_2"
WASHER = "thumb_distal2_retaining_washer"
SCREW = "thumb_distal2_retaining_screw"
SIZE = 850


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_helpers(repo):
    path = repo / "cad/manufacturing_v1/render.py"
    expected = sha(path)
    spec = importlib.util.spec_from_file_location("_manufacturing_v1_render_helpers", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    inputs = module.Inputs(repo)
    inputs.add(path, expected)
    inputs.add(__file__)
    return module, inputs


def load_build(root, helpers, inputs):
    scene = helpers.read(inputs.add(root / "generated/scene.json"))
    build = helpers.read(inputs.add(root / "generated/build_report.json"))
    design = helpers.read(inputs.add(root / "generated/design.json", build["generated_design_sha256"]))
    inputs.add(root / "generated/hand.step", build["assembly_step_sha256"])
    inputs.add(root / "generated/mass_inertia.json")
    helpers.guarded_sources(scene, inputs)
    helpers.guarded_sources(build, inputs)
    guard = build.get("input_hash_guard", {})
    if (scene["source_sha256"] != build["source_sha256"] or
            build.get("completed") is not True or build.get("errors") or
            guard.get("passed") is not True or guard.get("start") != guard.get("end") or
            guard.get("start") != build["source_sha256"]):
        raise ValueError("Incomplete or stale guarded build: " + str(root))
    rows = scene["hand"]
    if len({row["name"] for row in rows}) != len(rows) or build["part_count"] != len(rows):
        raise ValueError("Duplicate names or incorrect build count")
    return scene, build, design


def load_candidate(repo, profile, renderer, helpers, inputs):
    root = repo / "cad/manufacturing_v2"
    old_root = repo / "cad/manufacturing_v1"
    inputs.add(root / "RETENTION_DECISION.md")
    scene, build, design = load_build(root, helpers, inputs)
    old_scene, old_build, old_design = load_build(old_root, helpers, inputs)
    if any(design[key] != old_design[key] or design[key] != profile["design"][key]
           for key in helpers.KINEMATIC_KEYS):
        raise ValueError("Axes, link dimensions or explicit IP range changed")
    old = {row["name"]: row for row in old_scene["hand"]}
    new = {row["name"]: row for row in scene["hand"]}
    modified = {f"{digit}_joint_pin_{joint}" for digit in DIGITS for joint in (1, 2)}
    removed = {f"{digit}_distal{joint}_retaining_clip" for digit in DIGITS for joint in (1, 2)}
    added = {f"{digit}_distal{joint}_retaining_{kind}"
             for digit in DIGITS for joint in (1, 2) for kind in ("washer", "screw")}
    unchanged = set(old) - modified - removed
    expected_count = len(old) - len(removed) + len(added)
    if (len(old) != 349 or len(unchanged) != 329 or len(new) != expected_count or
            expected_count != 359 or set(new) != (set(old) - removed) | added):
        raise ValueError("Expected 349 - 10 clips + 20 keeper parts = 359")
    expected_sets = dict(modified_names=modified, removed_names=removed,
                         added_names=added, unchanged_names=unchanged)
    for key, expected in expected_sets.items():
        values = build.get("part_changes", {}).get(key, [])
        if len(values) != len(expected) or set(values) != expected:
            raise ValueError("Build change contract differs: " + key)
    for key, expected in (("modified_part_count", 10), ("removed_part_count", 10),
                          ("added_part_count", 20), ("unchanged_part_count", 329)):
        if build.get(key) != expected:
            raise ValueError("Incorrect build change count: " + key)
    for row in old.values():
        inputs.add(old_root / row["step"], row["sha256_step"])
        inputs.add(old_root / row["mesh"], row["sha256_stl"])
    for name in unchanged:
        for key in ("body", "sha256_step", "sha256_stl", "volume_mm3", "bbox_mm"):
            if new[name][key] != old[name][key]:
                raise ValueError("Unchanged source part differs: " + name + "/" + key)
    for name in modified:
        if (new[name]["body"] != old[name]["body"] or
                new[name]["sha256_step"] == old[name]["sha256_step"] or
                new[name]["sha256_stl"] == old[name]["sha256_stl"]):
            raise ValueError("Pin change/body contract differs: " + name)
    for digit in DIGITS:
        for joint in (1, 2):
            parent = new[f"{digit}_joint_pin_{joint}"]["body"]
            for kind in ("washer", "screw"):
                if new[f"{digit}_distal{joint}_retaining_{kind}"]["body"] != parent:
                    raise ValueError("Keeper does not follow its passive pin parent")
    neutral = profile["poses"].forward_kinematics(design, np.zeros(20))["bodies"]
    parts = []
    for row in scene["hand"]:
        if row["mesh_units"] != "mm" or row["body"] not in neutral:
            raise ValueError("Unexpected coordinate units/body: " + row["name"])
        mesh = inputs.add(root / row["mesh"], row["sha256_stl"])
        inputs.add(root / row["step"], row["sha256_step"])
        parts.append(dict(name=row["name"], body=row["body"], mesh_path=mesh,
                          points=renderer.read_stl_points(mesh), color=row["color"][:3],
                          neutral_inverse=np.linalg.inv(neutral[row["body"]])))
    identity = dict(baseline_part_count=len(old), candidate_part_count=len(parts),
                    modified_part_names=sorted(modified), removed_part_names=sorted(removed),
                    added_part_names=sorted(added), unchanged_part_names=sorted(unchanged),
                    unchanged_STEP_STL_volume_bbox_body_count=len(unchanged),
                    kinematic_fields_equal=list(helpers.KINEMATIC_KEYS),
                    basis="Guarded saved STEP/STL hashes and declared body/volume/bounds, not manufacturing tolerance approval")
    return dict(parts=parts, design=design, stage=build.get("stage", "manufacturing_v2"),
                label="M2 나사·규격 와셔 고정 후보", coordinates="neutral_world_mm"), identity


def local_detail(source, poses, names, offsets):
    frame = poses.forward_kinematics(source["design"], np.zeros(20))["bodies"]["thumb_ip"]
    world_to_local = np.linalg.inv(frame)
    lookup = {part["name"]: part for part in source["parts"]}
    parts, placements = [], []
    for name in names:
        offset = np.eye(4)
        offset[0, 3] = float(offsets.get(name, 0.))
        part = lookup[name]
        transform = offset @ world_to_local
        # The meshes already contain world-mm geometry. For this display-only
        # assembly, palm is identity and neutral_inverse maps to the IP frame.
        parts.append(dict(part, body="palm", neutral_inverse=transform))
        placements.append(dict(part=name, source_body=part["body"], display_body="palm",
                               source_world_to_display_local=transform.tolist(),
                               display_translation_local_mm=offset[:3, 3].tolist()))
    return dict(parts=parts, design=source["design"], stage=source["stage"], label="엄지 IP 고정부",
                coordinates="neutral_thumb_IP_local_mm_with_declared_display_offsets"), dict(
                    displayed_parts=list(names), hidden_parts=len(source["parts"]) - len(names),
                    placements=placements, section_cut=False, synthetic_geometry=False,
                    exploded=any(offsets.get(name, 0.) != 0 for name in names),
                    assembly_motion_or_insertion_test=False)


def detail_sheet(renderer, helpers, assembled, exploded, top_bounds, head, keeper):
    ymin = min(bounds[1] for bounds in top_bounds)
    ymax = max(bounds[3] for bounds in top_bounds)
    crop_height = min(SIZE, max(310, ymax - ymin + 65))
    crop_top = max(0, min(SIZE - crop_height, int(round((ymin + ymax - crop_height) / 2))))
    if crop_top > ymin - 16 or crop_top + crop_height < ymax + 17:
        raise ValueError("Detail crop clips actual saved geometry")
    top_tiles = [tile.crop((0, crop_top, SIZE, crop_top + crop_height)) for tile in (assembled, exploded)]
    title_height, label_height, footer_height = 130, 68, 165
    height = title_height + label_height + crop_height + label_height + SIZE + footer_height
    result = Image.new("RGB", (2 * SIZE, height), renderer.BG)
    draw = ImageDraw.Draw(result)
    def text(x, y, value, width, size=23):
        helpers.text_fit(renderer, draw, (x, y), value, width, size)
    text(24, 14, "엄지 IP 고정 방식 · 핀 끝에 와셔와 나사를 체결", 2 * SIZE - 48, 30)
    text(24, 61, "실제 저장 CAD / 위 두 칸은 같은 축척 · 아래는 머리 홈을 보는 사선 확대", 2 * SIZE - 48, 23)
    text(24, 96, "왼쪽 머리 슬롯은 회전 억제용 · 오른쪽 M2 나사가 와셔를 핀 끝에 고정", 2 * SIZE - 48, 21)
    top_labels = [("조립 위치 · 3부품만 표시", "나머지356부품은 숨겼습니다"),
                  ("분해 표시 · 위치만 옮긴 같은 부품", "로컬 X 이동: 핀 -12 / 와셔 +8 / 나사 +20 mm")]
    for index, tile in enumerate(top_tiles):
        x = index * SIZE
        text(x + 16, title_height + 5, top_labels[index][0], SIZE - 32, 24)
        text(x + 16, title_height + 37, top_labels[index][1], SIZE - 32, 19)
        result.paste(tile, (x, title_height + label_height))
    y = title_height + label_height + crop_height
    lower_labels = [("핀 머리 슬롯 · 머리 쪽 사선", "핀 전체 표시 / 슬롯에 일자 공구로 카운터홀드"),
                    ("와셔와 육각 나사 머리 · 나사 쪽 사선", "와셔 Ø7 / 나사 머리 Ø3.8 · 육각 AF1.5")]
    for index, tile in enumerate((head, keeper)):
        x = index * SIZE
        text(x + 16, y + 5, lower_labels[index][0], SIZE - 32, 24)
        text(x + 16, y + 37, lower_labels[index][1], SIZE - 32, 20)
        result.paste(tile, (x, y + label_height))
    lines = [
        "구매 규격의 명목 형상: DIN9021 M2 와셔 Ø7 / 내경2.2 / 두께0.8 + M2×4 나사 머리 Ø3.8×2",
        "핀은 맞춤 가공품 · 나사산은 간이 원통 표현 · 공급사 CAD가 아닌 공개 명목치 기반 모델",
        "분해 표시의 이동은 조립 순서·실제 삽입 통과를 뜻하지 않습니다. 그림으로 공구 접근을 승인하지 않습니다.",
        "내외경·두께 공차, 나사 물림·풀림·유지력·마찰·강도는 별도 검증이 필요합니다."]
    for index, line in enumerate(lines):
        text(24, height - footer_height + 15 + 32 * index, line, 2 * SIZE - 48, 20)
    return result, dict(common_top_vertical_crop_px=[crop_top, crop_top + crop_height],
                        foreground_preserved=True, pixel_scale_changed_by_crop=False,
                        lower_panels_have_separate_camera_fits=True)


def end_guard(inputs):
    end = {name: sha(inputs.repo / name) if (inputs.repo / name).is_file() else None
           for name in inputs.hashes}
    return dict(start=dict(inputs.hashes), end=end, passed=end == inputs.hashes)


def closeup_camera(renderer, assembly, gestures, normal):
    """Fit all actual geometry, with sufficient depth distance independent of scale."""
    camera = renderer.view_fit([assembly], gestures, np.asarray(normal, dtype=float), 1.)
    focus = np.asarray(camera["focus"])
    direction = np.asarray(camera["position"]) - focus
    distance = max(float(np.linalg.norm(direction)), 150.)
    camera["position"] = (focus + direction / np.linalg.norm(direction) * distance).tolist()
    camera["parallel_scale_mm"] *= 1.10
    camera.update(framing_margin_fraction=.116, full_part_geometry_fitted=True,
                  minimum_camera_distance_mm=150., viewing_face="negative_X_pin_head" if normal[0] < 0 else "positive_X_socket_head")
    return camera


def prepare_closeup(renderer, assembly, cameras, size, mujoco):
    """Lower panels only: reveal actual recessed walls without shadow-map acne."""
    state = renderer.make_renderer(assembly, cameras, size, mujoco)
    model = state["model"]
    # MuJoCo stores clipping distances relative to model.stat.extent. Explicit
    # physical planes avoid defaults scaled from the original world-mm mesh.
    extent = float(model.stat.extent)
    model.vis.map.znear = .001 / extent
    model.vis.map.zfar = 1.0 / extent
    model.vis.headlight.ambient[:] = .22
    model.vis.headlight.diffuse[:] = .30
    model.vis.headlight.specular[:] = .08
    camera = next(iter(cameras.values()))
    focus = np.asarray(camera["focus"]) * .001
    light_position = (np.asarray(camera["position"]) * .001
                      + np.asarray(camera["right"]) * .06 + np.asarray(camera["up"]) * .08)
    light_direction = focus - light_position
    model.light_pos[0] = light_position
    model.light_dir[0] = light_direction / np.linalg.norm(light_direction)
    model.light_type[0] = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
    model.light_diffuse[0] = [.65, .65, .65]
    model.light_specular[0] = [.08, .08, .08]
    model.light_castshadow[:] = False
    assembly["_render_state"] = state
    return dict(near_plane_mm=1., far_plane_mm=1000., model_extent_m=extent,
                headlight_ambient=.22, headlight_diffuse=.30, headlight_specular=.08,
                directional_key_diffuse=.65, directional_key_specular=.08,
                directional_key_position_m=light_position.tolist(),
                directional_key_direction=(light_direction / np.linalg.norm(light_direction)).tolist(), shadows=False,
                reason="Oblique recessed-wall visibility; disable tiny-part shadow-map artifacts; no geometry change")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=HERE.parents[1])
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    repo, output = args.repo.resolve(), args.output.resolve()
    if os.path.lexists(args.output) or output.exists():
        parser.error("Use a new output directory; previous evidence is preserved")
    if Path(__file__).resolve() != repo / "cad/manufacturing_v2/render.py":
        parser.error("Renderer must belong to the selected checkout")
    if os.name == "nt" and output.drive.upper() != "D:":
        parser.error("Use a new D: output directory")
    if output.is_relative_to(repo / "cad"):
        parser.error("Render into a separate output directory outside CAD inputs")
    helpers, inputs = load_helpers(repo)
    profile, renderer = helpers.read_profile(repo, inputs)
    candidate, identity = load_candidate(repo, profile, renderer, helpers, inputs)
    gestures = [g for g in profile["document"]["gestures"] if g["id"] in ("open", "thumb_wrist")]
    if [g["id"] for g in gestures] != ["open", "thumb_wrist"]:
        raise ValueError("Missing intended open / latest thumb-wrist profile poses")
    whole_camera = {"oblique": renderer.view_fit([candidate], gestures, renderer.VIEWS["oblique"], 1.)}
    zero = [dict(angles_deg=np.zeros(20).tolist())]
    details = [local_detail(candidate, profile["poses"], [PIN, WASHER, SCREW], {}),
               local_detail(candidate, profile["poses"], [PIN, WASHER, SCREW], {PIN: -12., WASHER: 8., SCREW: 20.}),
               local_detail(candidate, profile["poses"], [PIN], {}),
               local_detail(candidate, profile["poses"], [WASHER, SCREW], {})]
    assembled, exploded, head, keeper = [row[0] for row in details]
    side_camera = {"detail": renderer.view_fit([assembled, exploded], zero, np.array([.55, -.7, 1.]), 1.)}
    head_camera = {"head": closeup_camera(renderer, head, zero, [-1., .08, .22])}
    keeper_camera = {"keeper": closeup_camera(renderer, keeper, zero, [1., -.22, .20])}
    assemblies = [candidate, assembled, exploded, head, keeper]
    import mujoco
    output.mkdir(parents=True, exist_ok=False)
    completed, records = [], []
    try:
        whole_tiles = []
        for gesture in gestures:
            image, bounds = renderer.render_one(candidate, gesture["angles_deg"], "oblique", whole_camera, SIZE, mujoco)
            whole_tiles.append((gesture["label_ko"], image))
            records.append(dict(kind="whole_hand", pose=gesture["id"], angles_deg=gesture["angles_deg"],
                                displayed_parts=len(candidate["parts"]), hidden_parts=0, foreground_bounds_px=bounds))
        helpers.basic_sheet(renderer, whole_tiles, "현재 손 외형을 유지한 핀 고정 후보",
                            "펼친 손 / 엄지 끝마디만 팔목 쪽으로15° · 같은 카메라·같은 축척 · 실제359부품",
                            "핀10수정 + 클립10제거 + 와셔·나사20추가 / 나머지329부품 STEP·STL 동일", SIZE).save(output / "whole_hand.png")
        completed.append("whole_hand.png")
        helpers.close_renderer(candidate)
        images, bounds_list = [], []
        for (assembly, description), view, camera in zip(details, ("detail", "detail", "head", "keeper"),
                                                         (side_camera, side_camera, head_camera, keeper_camera)):
            if view in ("head", "keeper"):
                description["closeup_lighting_and_depth"] = prepare_closeup(renderer, assembly, camera, SIZE, mujoco)
            image, bounds = renderer.render_one(assembly, zero[0]["angles_deg"], view, camera, SIZE, mujoco)
            images.append(image)
            bounds_list.append(bounds)
            records.append(dict(kind="retention_detail", view=view, foreground_bounds_px=bounds, **description))
            helpers.close_renderer(assembly)
        image, crop = detail_sheet(renderer, helpers, images[0], images[1], bounds_list[:2], images[2], images[3])
        image.save(output / "retention_assembly.png")
        completed.append("retention_assembly.png")
        guard = end_guard(inputs)
        if not guard["passed"]:
            raise ValueError("A source changed while rendering")
        report = dict(schema_version="manufacturing_v2.render.v1", status="completed", completed=True,
                      inputs_unchanged=True, input_hash_guard=guard, source_sha256=inputs.hashes,
                      output_sha256={name: sha(output / name) for name in completed},
                      candidate_part_count=len(candidate["parts"]), baseline_part_count=349,
                      identity_checks=identity, profile_poses=gestures,
                      original_thumb_ip_range_deg=profile["original_design"]["joint_ranges_deg"]["thumb_ip"],
                      exploratory_display_thumb_ip_range_deg=profile["design"]["joint_ranges_deg"]["thumb_ip"],
                      camera_groups=dict(whole_hand=whole_camera, assembly_and_exploded=side_camera,
                                         counterhold_head=head_camera, screw_and_washer=keeper_camera),
                      detail_crop=crop, renders=records,
                      renderer=dict(name="MuJoCo/Pillow", mujoco_version=mujoco.__version__, off_screen=True,
                                    simulation_steps=0, synthetic_geometry=False, input_meshes_modified=False,
                                    section_cut_performed=False, tile_size_px=SIZE),
                      limitations=["The complete candidate is shown in two poses with one camera and scale; no baseline whole-hand panels are duplicated.",
                                   "Close-ups use saved meshes and explicitly declared display translations. They hide the other hand components.",
                                   "The lower oblique close-ups fit all displayed parts with separate cameras and explicit depth/lighting; the upper assembled/exploded pair shares its unchanged fit.",
                                   "Screws and washers are nominal parametric purchased-part representations, not downloaded vendor CAD. Threads are simplified.",
                                   "Images do not prove insertion, tool access, physical thread engagement, retention strength, tolerance clearance or motor performance.",
                                   "A machining blank is intentionally not included: the shown pin is the saved simplified assembly pin, not a tap-drill manufacturing drawing."])
        renderer.write_json(output / "render_report.json", report)
        print(json.dumps(dict(status="completed", outputs=completed, inputs_unchanged=True)), flush=True)
    except Exception as error:
        guard = end_guard(inputs)
        renderer.write_json(output / "render_report.json", dict(schema_version="manufacturing_v2.render.v1",
            status="failed", completed=False, error=repr(error), inputs_unchanged=guard["passed"],
            input_hash_guard=guard, source_sha256=inputs.hashes,
            output_sha256={name: sha(output / name) for name in completed}))
        raise
    finally:
        for assembly in assemblies:
            helpers.close_renderer(assembly)


if __name__ == "__main__":
    main()
