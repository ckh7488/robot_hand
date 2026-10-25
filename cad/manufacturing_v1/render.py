"""Actual-mesh whole-hand and pin/bushing comparison; no CAD mutation.

The close-ups are explicitly exploded or isolated component views, not section
cuts. Run in a hidden process after build.py has finished and frozen its inputs.
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

for _key in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_key, "1")
sys.dont_write_bytecode = True

import numpy as np
from PIL import Image, ImageDraw

HERE = Path(__file__).resolve().parent
PIN = "thumb_joint_pin_2"
BUSH = "thumb_segment2_distal_left_bushing"
KINEMATIC_KEYS = ("units", "fingers", "thumb", "joint_ranges_deg", "palm_target_envelope_mm")


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class Inputs:
    def __init__(self, repo):
        self.repo = Path(repo).resolve()
        self.hashes = {}

    def add(self, path, expected=None):
        path = Path(path).resolve()
        if not path.is_relative_to(self.repo) or not path.is_file():
            raise ValueError("Missing input or repository escape: " + str(path))
        key = path.relative_to(self.repo).as_posix()
        value = sha(path)
        if expected is not None and expected != value:
            raise ValueError("Stale input: " + key)
        if key in self.hashes and self.hashes[key] != value:
            raise ValueError("Input changed while loading: " + key)
        self.hashes[key] = value
        return path

    def verify(self):
        return {key: (self.repo / key).is_file() and sha(self.repo / key) == value
                for key, value in self.hashes.items()}


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def read_profile(repo, inputs):
    profile_path = inputs.add(repo / "cad/thumb_wrist_v1/wrist_profile.py")
    inputs.add(profile_path.with_name("profile.json"))
    specification = read(profile_path.with_name("profile.json"))
    base = (repo / specification["base_candidate"]).resolve()
    for relative in ("poses.py", "poses.json", "design.json", "generated/scene.json", "render_gestures.py"):
        inputs.add(base / relative)
    spec = importlib.util.spec_from_file_location("_manufacturing_wrist_profile", profile_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    profile = module.load(repo)
    renderer = importlib.import_module("render_gestures")
    if (Path(renderer.__file__).resolve() != base / "render_gestures.py" or
            Path(profile["poses"].__file__).resolve() != base / "poses.py" or renderer.ROOT.resolve() != repo):
        raise ValueError("Profile/renderer imported from a different checkout")
    for relative, expected in profile["catalog"]["reference"]["source_sha256"].items():
        inputs.add(repo / relative, expected)
    return profile, renderer


def guarded_sources(document, inputs):
    if document.get("inputs_unchanged") is not True or not document.get("source_sha256"):
        raise ValueError("Generated input has no successful source guard")
    for relative, expected in document["source_sha256"].items():
        inputs.add(inputs.repo / relative, expected)


def load_candidate(candidate_root, profile, renderer, inputs):
    scene = read(inputs.add(candidate_root / "generated/scene.json"))
    build = read(inputs.add(candidate_root / "generated/build_report.json"))
    design = read(inputs.add(candidate_root / "generated/design.json", build["generated_design_sha256"]))
    inputs.add(candidate_root / "generated/hand.step", build["assembly_step_sha256"])
    inputs.add(candidate_root / "generated/mass_inertia.json")
    guarded_sources(scene, inputs)
    guarded_sources(build, inputs)
    if scene["source_sha256"] != build["source_sha256"]:
        raise ValueError("Scene/build source guards differ")
    guard = build.get("input_hash_guard", {})
    if (build.get("completed") is not True or build.get("errors") or guard.get("passed") is not True
            or guard.get("start") != guard.get("end") or guard.get("start") != build["source_sha256"]):
        raise ValueError("Incomplete or unstable build")
    if any(design[k] != profile["design"][k] for k in KINEMATIC_KEYS):
        raise ValueError("Candidate axes, dimensions or explicit IP range differ from the profile")
    old_scene = read(inputs.add(profile["base"] / "generated/scene.json"))
    old_rows = {r["name"]: r for r in old_scene["hand"]}
    rows = scene["hand"]
    if len(rows) != 349 or build.get("part_count") != 349 or len(old_rows) != 349:
        raise ValueError("Expected 349 original and 349 candidate hand parts")
    if [r["name"] for r in rows] != [r["name"] for r in old_scene["hand"]]:
        raise ValueError("Candidate names/order differ from the original 349-part hand")
    expected_changes = {f"{digit}_joint_pin_{joint}" for digit in ("index", "middle", "ring", "little", "thumb") for joint in (1, 2)}
    expected_changes |= {f"{digit}_segment{joint}_distal_left_bushing" for digit in ("index", "middle", "ring", "little", "thumb") for joint in (1, 2)}
    changed = set()
    neutral = profile["poses"].forward_kinematics(design, np.zeros(20))["bodies"]
    parts = []
    for row in rows:
        old = old_rows[row["name"]]
        if row["mesh_units"] != "mm" or row["body"] not in neutral or row["body"] != old["body"]:
            raise ValueError("Part body or coordinate-unit mapping changed: " + row["name"])
        # Resolve relative to the candidate directory, including the explicit
        # ../gesture_hand_v3 paths. Inputs.add still rejects repository escape.
        mesh = inputs.add(candidate_root / row["mesh"], row["sha256_stl"])
        inputs.add(candidate_root / row["step"], row["sha256_step"])
        if row["sha256_stl"] != old["sha256_stl"] or row["sha256_step"] != old["sha256_step"]:
            changed.add(row["name"])
        parts.append(dict(name=row["name"], body=row["body"], mesh_path=mesh,
                          points=renderer.read_stl_points(mesh), color=row["color"][:3],
                          neutral_inverse=np.linalg.inv(neutral[row["body"]])))
    if changed != expected_changes:
        raise ValueError("The changed source parts are not exactly ten pins and ten left parent bushings")
    return dict(parts=parts, design=design, stage=build.get("stage", "manufacturing_v1"),
                label="제작 수정 후보", coordinates="neutral_world_mm"), dict(
                    changed_part_names=sorted(changed), changed_parts=20, unchanged_part_hashes=329,
                    hand_part_count=349, comparison_basis="STEP and STL hashes; not a geometric tolerance or assembly-motion proof")


def text_fit(renderer, draw, xy, text, width, size):
    while size > 12 and draw.textbbox((0, 0), text, font=renderer.font(size))[2] > width:
        size -= 1
    font = renderer.font(size)
    if draw.textbbox((0, 0), text, font=font)[2] > width:
        raise ValueError("Caption does not fit")
    draw.text(xy, text, font=font, fill=renderer.INK)


def basic_sheet(renderer, tiles, title, subtitle, footer, size=850):
    rows = math.ceil(len(tiles) / 2)
    image = Image.new("RGB", (size * 2, 104 + rows * (size + 44) + 90), renderer.BG)
    draw = ImageDraw.Draw(image)
    text_fit(renderer, draw, (22, 14), title, image.width - 44, 30)
    text_fit(renderer, draw, (22, 59), subtitle, image.width - 44, 22)
    for index, (label, tile) in enumerate(tiles):
        x, y = index % 2 * size, 104 + index // 2 * (size + 44)
        text_fit(renderer, draw, (x + 14, y + 8), label, size - 28, 24)
        image.paste(tile, (x, y + 44))
    text_fit(renderer, draw, (22, image.height - 68), footer, image.width - 44, 21)
    text_fit(renderer, draw, (22, image.height - 36), "같은 카메라·같은 축척 · 실제 저장 메시 · 렌더는 공차/조립/강도 승인 검사가 아님", image.width - 44, 19)
    return image


def close_assembly(source, poses, exploded):
    frame = poses.forward_kinematics(source["design"], np.zeros(20))["bodies"]["thumb_ip"]
    inverse = np.linalg.inv(frame)
    lookup = {p["name"]: p for p in source["parts"]}
    names = [PIN, BUSH] if exploded else [BUSH]
    parts, placements = [], []
    for name in names:
        offset = np.eye(4)
        if exploded and name == PIN:
            offset[0, 3] = -35.
        original = lookup[name]
        parts.append(dict(original, body="palm", neutral_inverse=offset @ inverse))
        placements.append(dict(part=name, source_body=original["body"], display_body="palm",
                               source_world_to_display_local= (offset @ inverse).tolist(),
                               display_translation_in_thumb_ip_frame_mm=offset[:3, 3].tolist()))
    return dict(parts=parts, design=source["design"], stage=source["stage"], label=source["label"],
                coordinates="thumb_IP_neutral_local_mm_with_explicit_display_translation"), dict(
                    displayed_parts=names, hidden_hand_parts=349 - len(names), is_section_cut=False,
                    exploded=exploded, placements=placements,
                    meaning="Display separation only; no assembly insertion sweep is implied." if exploded else
                    "Only the actual saved parent bushing is shown along its axis; the pin and other hand parts are hidden.")


def close_renderer(assembly):
    state = assembly.pop("_render_state", None)
    if state is not None:
        state["renderer"].close()


def pin_sheet(renderer, exploded_tiles, exploded_bounds, bore_tiles, size):
    ymin = min(b[1] for b in exploded_bounds)
    ymax = max(b[3] for b in exploded_bounds)
    height = min(size, max(330, ymax - ymin + 65))
    top = max(0, min(size - height, int(round((ymin + ymax - height) / 2))))
    if top > ymin - 16 or top + height < ymax + 17:
        raise ValueError("Close-up display crop would clip real geometry")
    cropped = [tile.crop((0, top, size, top + height)) for tile in exploded_tiles]
    head, caption, footer = 110, 58, 118
    result = Image.new("RGB", (2 * size, head + caption + height + caption + size + footer), renderer.BG)
    draw = ImageDraw.Draw(result)
    text_fit(renderer, draw, (22, 14), "엄지 IP 핀·부싱 · D형 내경과 원형 내경 비교", result.width - 44, 30)
    text_fit(renderer, draw, (22, 59), "실제 부품 근접 보기 / 위: 분해 표시 · 아래: 부싱만 축 방향에서 보기", result.width - 44, 22)
    for index, tile in enumerate(cropped):
        x = index * size
        label = "원본 · D절삭 핀 + D형 내경" if index == 0 else "후보 · 원형 핀 + 원형 내경"
        text_fit(renderer, draw, (x + 15, head + 10), label, size - 30, 24)
        result.paste(tile, (x, head + caption))
        y = head + caption + height
        label = "원본 왼쪽 부모 부싱 · D형 통로" if index == 0 else "후보 왼쪽 부모 부싱 · 원형 Ø4.03mm 통로"
        text_fit(renderer, draw, (x + 15, y + 13), label, size - 30, 22)
        result.paste(bore_tiles[index], (x, y + caption))
    for i, value in enumerate([
            "위: 핀만 IP축 방향으로 -35mm 이동해 벌려 표시했습니다. 실제 장착 위치나 삽입 순서를 뜻하지 않습니다.",
            "아래: 다른348개 부품을 숨긴 실제 부싱입니다. 단면 절삭·가상 구멍·형상 보정은 하지 않았습니다.",
            "핀/부싱 조립 통로 수정 후보 · 축·손가락 길이 유지 · 끼워맞춤·보유링·강도·마찰은 별도 검증"]):
        text_fit(renderer, draw, (22, result.height - footer + 12 + 32 * i), value, result.width - 44, 19)
    return result, dict(exploded_common_vertical_crop_px=[top, top + height],
                        crop_preserves_all_recorded_foreground_bounds=True, scale_change_from_crop=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=HERE.parents[1])
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    repo, output = args.repo.resolve(), args.output.resolve()
    if os.path.lexists(args.output) or output.exists():
        parser.error("Use a new output directory; earlier evidence is preserved")
    if Path(__file__).resolve() != repo / "cad/manufacturing_v1/render.py":
        parser.error("Run the renderer from the selected repository checkout")
    if os.name == "nt" and output.drive.upper() != "D:":
        parser.error("This workspace requires a new output directory on D:")
    inputs = Inputs(repo)
    inputs.add(__file__)
    profile, renderer = read_profile(repo, inputs)
    original = renderer.load_candidate(profile["base"], profile["catalog"], inputs)
    candidate, identity = load_candidate(HERE, profile, renderer, inputs)
    # The original is rendered at the same explicit exploratory IP angle as the
    # candidate; its saved design/ranges and every source mesh remain unchanged.
    original["design"] = copy.deepcopy(profile["design"])
    original["label"] = "원본 GESTURE002"
    gestures = [g for g in profile["document"]["gestures"] if g["id"] in ("open", "thumb_wrist")]
    if [g["id"] for g in gestures] != ["open", "thumb_wrist"]:
        raise ValueError("Expected open and wrist-directed thumb profile gestures")
    whole_camera = {"oblique": renderer.view_fit([original, candidate], gestures, renderer.VIEWS["oblique"], 1.)}
    zero = [dict(angles_deg=np.zeros(20).tolist())]
    exploded = [close_assembly(a, profile["poses"], True) for a in (original, candidate)]
    bores = [close_assembly(a, profile["poses"], False) for a in (original, candidate)]
    exploded_camera = {"detail": renderer.view_fit([a for a, _ in exploded], zero, np.array([0., .8, 1.]), 1.)}
    bore_camera = {"axial": renderer.view_fit([a for a, _ in bores], zero, np.array([-1., 0., 0.]), 1.)}
    assemblies = [original, candidate] + [a for a, _ in exploded + bores]
    import mujoco
    output.mkdir(parents=True, exist_ok=False)
    completed, records = [], []
    try:
        tiles = []
        for gesture in gestures:
            for assembly in (original, candidate):
                image, bounds = renderer.render_one(assembly, gesture["angles_deg"], "oblique", whole_camera, 850, mujoco)
                tiles.append((assembly["label"] + " · " + gesture["label_ko"], image))
                records.append(dict(kind="whole_hand", source=assembly["label"], pose=gesture["id"],
                                    angles_deg=gesture["angles_deg"], parts_shown=349, foreground_bounds_px=bounds))
        basic_sheet(renderer, tiles, "손 외형은 유지하고 내부 핀·부싱을 수정",
                    "왼쪽 원본 / 오른쪽 제작 후보 · 같은 두 자세와 같은 349개 부품 구성",
                    "수정20부품: 원형 핀10개 + 왼쪽 부모 부싱10개 / 나머지329개 STEP·STL 해시 동일").save(output / "whole_hand.png")
        completed.append("whole_hand.png")
        close_renderer(original)
        close_renderer(candidate)
        exploded_tiles, bounds_list, bore_tiles = [], [], []
        for assembly, description in exploded:
            image, bounds = renderer.render_one(assembly, zero[0]["angles_deg"], "detail", exploded_camera, 850, mujoco)
            exploded_tiles.append(image)
            bounds_list.append(bounds)
            records.append(dict(kind="exploded_pin_and_bushing", source=assembly["label"],
                                foreground_bounds_px=bounds, **description))
            close_renderer(assembly)
        for assembly, description in bores:
            image, bounds = renderer.render_one(assembly, zero[0]["angles_deg"], "axial", bore_camera, 850, mujoco)
            bore_tiles.append(image)
            records.append(dict(kind="axial_bushing_only", source=assembly["label"],
                                foreground_bounds_px=bounds, **description))
            close_renderer(assembly)
        image, crop = pin_sheet(renderer, exploded_tiles, bounds_list, bore_tiles, 850)
        image.save(output / "pin_assembly.png")
        completed.append("pin_assembly.png")
        unchanged = inputs.verify()
        if not all(unchanged.values()):
            raise ValueError("Input changed during rendering: " + str([k for k, v in unchanged.items() if not v]))
        report = dict(schema_version="manufacturing_v1.render.v1", status="completed", completed=True,
                      inputs_unchanged=True, source_sha256=inputs.hashes,
                      output_sha256={name: sha(output / name) for name in completed},
                      identity_checks=identity, candidate_part_count=349, original_part_count=349,
                      original_range_deg=profile["original_design"]["joint_ranges_deg"]["thumb_ip"],
                      explicit_display_range_deg=profile["design"]["joint_ranges_deg"]["thumb_ip"],
                      profile_poses=gestures, camera_groups=dict(whole_hand=whole_camera,
                      exploded=exploded_camera, axial_bushing=bore_camera), detail_crop=crop,
                      renders=records, renderer=dict(name="MuJoCo/Pillow", mujoco_version=mujoco.__version__,
                      off_screen=True, simulation_steps=0, synthetic_geometry=False,
                      input_meshes_modified=False, section_cut_performed=False, tile_size_px=850),
                      limitations=["Whole-hand panels show every actual saved mesh with common pose/camera/scale.",
                      "Close-ups intentionally hide other parts and translate the pin -35 mm along the neutral IP axis for display.",
                      "The axial bore pictures are actual bushing meshes, not generated holes or analytical section drawings.",
                      "Visual comparison does not test pin insertion, clip installation, tolerances, friction, strength, or physical joint range.",
                      "Only the displayed IP range is expanded on the original; no saved baseline source is changed."])
        renderer.write_json(output / "render_report.json", report)
        print(json.dumps(dict(status="completed", outputs=completed, inputs_unchanged=True)), flush=True)
    except Exception as error:
        renderer.write_json(output / "render_report.json", dict(schema_version="manufacturing_v1.render.v1",
            status="failed", completed=False, error=repr(error), inputs_unchanged=all(inputs.verify().values()),
            source_sha256=inputs.hashes, output_sha256={name: sha(output / name) for name in completed}))
        raise
    finally:
        for assembly in assemblies:
            close_renderer(assembly)


if __name__ == "__main__":
    main()
