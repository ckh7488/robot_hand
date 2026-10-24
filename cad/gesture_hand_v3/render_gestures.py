"""Render saved CAD triangles in nine authored poses without changing geometry.

Run with a hidden Python process and MuJoCo offscreen support. This module
starts no child processes and never launches a GUI. A new output directory is
required so an earlier render or its evidence cannot be silently overwritten.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ET

for _key in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_key, "1")
sys.dont_write_bytecode = True

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial.transform import Rotation

import poses

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
BG = (245, 248, 251)
INK = (31, 45, 62)
VIEWS = {"front": np.array([0., 0., 1.]),
         "oblique": np.array([.72, -.45, 1.])}
VIEW_LABELS = {"front": "정면", "oblique": "사선"}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def write_json(path, data):
    Path(path).write_text(json.dumps(data, indent=2, ensure_ascii=False,
                                    allow_nan=False) + "\n", encoding="utf-8")


def local_path(base, relative):
    path = (base / relative).resolve()
    if not path.is_relative_to(base.resolve()):
        raise ValueError("Input path escapes its declared root: " + str(relative))
    return path


class Inputs:
    def __init__(self):
        self.hashes = {}

    def add(self, path, expected=None):
        path = Path(path).resolve()
        actual = sha(path)
        if expected is not None and actual != expected:
            raise ValueError("Stale generated input: " + str(path))
        key = path.relative_to(ROOT).as_posix() if path.is_relative_to(ROOT) else str(path)
        if key in self.hashes and self.hashes[key] != actual:
            raise ValueError("Input changed while loading: " + key)
        self.hashes[key] = actual
        return path

    def verify(self):
        return {name: sha(ROOT / name) == digest for name, digest in self.hashes.items()}


def check_sources(document, inputs):
    if document.get("inputs_unchanged") is not True:
        raise ValueError("Generated CAD does not have a successful input guard")
    for relative, expected in document["source_sha256"].items():
        inputs.add(local_path(ROOT, relative), expected)


def read_stl_points(path):
    """Read actual STL vertices for framing; rendering reads the same STL file."""
    raw = Path(path).read_bytes()
    if len(raw) >= 84:
        count = struct.unpack_from("<I", raw, 80)[0]
        if len(raw) == 84 + 50 * count:
            records = np.frombuffer(raw, dtype=np.dtype([
                ("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")]),
                offset=84, count=count)
            points = records["vertices"].reshape(-1, 3).astype(float)
        else:
            points = np.array([[float(x) for x in line.split()[1:]]
                               for line in raw.decode("ascii").splitlines()
                               if line.strip().startswith("vertex ")], dtype=float)
    else:
        points = np.empty((0, 3))
    if points.ndim != 2 or points.shape[1] != 3 or not len(points) or not np.isfinite(points).all():
        raise ValueError("Empty or nonfinite STL vertices: " + str(path))
    return points


def load_candidate(cad_root, catalog, inputs):
    scene_path = inputs.add(cad_root / "generated/scene.json")
    build_path = inputs.add(cad_root / "generated/build_report.json")
    design_path = inputs.add(cad_root / "design.json")
    scene, build, design = map(read_json, (scene_path, build_path, design_path))
    check_sources(scene, inputs)
    check_sources(build, inputs)
    if scene["source_sha256"] != build["source_sha256"]:
        raise ValueError("Scene/build source guards disagree")
    # A repaired shell can differ; its joint frames and authored pose meaning
    # must not silently change under this catalog.
    for g in catalog["gestures"]:
        a = poses.forward_kinematics(design, g["angles_deg"])
        b = poses.forward_kinematics(catalog["kinematic_design"], g["angles_deg"])
        if any(not np.allclose(a["bodies"][n], b["bodies"][n], rtol=0, atol=1e-9) for n in a["bodies"]):
            raise ValueError("CAD/catalog kinematics differ")
        if any(not np.allclose(a["tips"][n], b["tips"][n], rtol=0, atol=1e-9) for n in a["tips"]):
            raise ValueError("CAD/catalog link endpoints differ")
    neutral = poses.forward_kinematics(design, np.zeros(20))["bodies"]
    parts = []
    for row in scene["hand"]:
        if row.get("mesh_units") != "mm" or row["body"] not in neutral:
            raise ValueError("Missing explicit world-mm mesh/body mapping")
        mesh_path = inputs.add(local_path(cad_root, row["mesh"]), row["sha256_stl"])
        parts.append(dict(name=row["name"], body=row["body"], mesh_path=mesh_path,
                          points=read_stl_points(mesh_path),
                          color=row["color"][:3], neutral_inverse=np.linalg.inv(neutral[row["body"]])))
    if len(parts) != build["part_count"]:
        raise ValueError("Scene/build part counts disagree")
    # STEP is not rasterized, but bind the visual evidence to its saved assembly.
    inputs.add(cad_root / "generated/hand.step")
    return dict(parts=parts, design=design, stage=build["stage"],
                label="재구성 CAD · " + build["stage"], coordinates="neutral_world_mm")


def load_reference(reference_root, inputs):
    design = read_json(inputs.add(reference_root / "design.json"))
    mapping = read_json(inputs.add(reference_root / "parts.json"))
    xml = ET.parse(inputs.add(reference_root / "preview.xml"))
    assets = {n.attrib["name"]: n for n in xml.findall("./asset/mesh")}
    geoms = {}
    for geom in xml.findall(".//geom"):
        if "mesh" not in geom.attrib:
            continue
        asset = assets[geom.attrib["mesh"]]
        if not np.allclose(np.fromstring(asset.get("scale", "1 1 1"), sep=" "), .001):
            raise ValueError("Reference mesh is not stored in millimetres")
        if not np.allclose(np.fromstring(geom.get("pos", "0 0 0"), sep=" "), 0):
            raise ValueError("Reference geom has an unexpected local translation")
        if not np.allclose(np.fromstring(geom.get("quat", "1 0 0 0"), sep=" "), [1, 0, 0, 0]):
            raise ValueError("Reference geom has an unexpected local rotation")
        filename = asset.attrib["file"]
        if filename in geoms:
            raise ValueError("Reference mesh is instanced more than once")
        geoms[filename] = geom
    if set(geoms) != set(mapping):
        raise ValueError("Reference XML/part manifest mismatch")
    parts = []
    for relative, row in mapping.items():
        body = "palm" if row["frame"] == "world" else row["frame"]
        if body not in {"palm", *poses.NAMES}:
            raise ValueError("Unexpected reference body: " + body)
        mesh_path = inputs.add(local_path(reference_root, relative))
        color = np.fromstring(geoms[relative].get("rgba", ".22 .38 .56 1"), sep=" ")[:3]
        # Accepted meshes are body-local mm. Do not subtract the neutral frame.
        parts.append(dict(name=row["part"], body=body, mesh_path=mesh_path,
                          points=read_stl_points(mesh_path),
                          color=color, neutral_inverse=np.eye(4)))
    return dict(parts=parts, design=design, stage="FORM002_visual_reference",
                label="수용 외형 FORM002", coordinates="body_local_mm")


def transformations(assembly, angles):
    frames = poses.forward_kinematics(assembly["design"], angles)["bodies"]
    return [(p, frames[p["body"]] @ p["neutral_inverse"]) for p in assembly["parts"]]


def view_fit(assemblies, gestures, normal, aspect):
    """One parallel camera, fitted to every source and every pose without crop."""
    normal = normal / np.linalg.norm(normal)
    right = np.cross([0., 1., 0.], normal)
    right /= np.linalg.norm(right)
    up = np.cross(normal, right)
    basis = np.column_stack((right, up, normal))
    low = np.full(3, np.inf); high = -low
    for assembly in assemblies:
        for g in gestures:
            for part, T in transformations(assembly, g["angles_deg"]):
                points = part["points"] @ T[:3, :3].T + T[:3, 3]
                projected = points @ basis
                low = np.minimum(low, projected.min(axis=0))
                high = np.maximum(high, projected.max(axis=0))
    focus = basis @ ((low + high) / 2)
    extent = high - low
    scale = max(extent[1], extent[0] / aspect) * .56
    distance = max(float(np.linalg.norm(extent)), 1.) * 3
    return dict(position=(focus + normal * distance).tolist(), focus=focus.tolist(),
                right=right.tolist(),
                up=up.tolist(), parallel_scale_mm=float(scale),
                projected_union_bounds_mm=[low.tolist(), high.tolist()],
                framing_margin_fraction=.06, projection="parallel")


def make_renderer(assembly, cameras, size, mujoco):
    fmt = lambda values: " ".join(format(float(x), ".17g") for x in values)
    xml = ET.Element("mujoco")
    ET.SubElement(xml, "compiler", angle="radian", fusestatic="false")
    visual = ET.SubElement(xml, "visual")
    ET.SubElement(visual, "global", offwidth=str(size), offheight=str(size))
    ET.SubElement(visual, "headlight", ambient=".65 .65 .65", diffuse=".5 .5 .5", specular=".1 .1 .1")
    asset = ET.SubElement(xml, "asset")
    rgb = fmt(np.array(BG) / 255)
    ET.SubElement(asset, "texture", type="skybox", builtin="flat", rgb1=rgb, rgb2=rgb,
                  width="256", height="1536")
    world = ET.SubElement(xml, "worldbody")
    ET.SubElement(world, "light", pos=".4 .3 .9", diffuse=".5 .5 .5")
    for name, camera in cameras.items():
        ET.SubElement(world, "camera", name=name, pos=fmt(np.array(camera["position"]) * .001),
                      xyaxes=fmt(camera["right"] + camera["up"]), orthographic="true", ipd="0",
                      fovy=format(2 * camera["parallel_scale_mm"] * .001, ".17g"))
    for i, part in enumerate(assembly["parts"]):
        ET.SubElement(asset, "mesh", name="mesh" + str(i), file=part["mesh_path"].as_posix(),
                      scale=".001 .001 .001", maxhullvert="64")
        body = ET.SubElement(world, "body", name="part" + str(i))
        ET.SubElement(body, "geom", type="mesh", mesh="mesh" + str(i),
                      rgba=fmt(list(part["color"]) + [1.]), contype="0", conaffinity="0")
    model = mujoco.MjModel.from_xml_string(ET.tostring(xml, encoding="unicode"))
    data = mujoco.MjData(model)
    ids = [model.body("part" + str(i)).id for i in range(len(assembly["parts"]))]
    return dict(model=model, data=data, ids=ids, renderer=mujoco.Renderer(model, height=size, width=size))


def render_one(assembly, angles, view, cameras, size, mujoco):
    if "_render_state" not in assembly:
        assembly["_render_state"] = make_renderer(assembly, cameras, size, mujoco)
    state = assembly["_render_state"]
    model, data, renderer = state["model"], state["data"], state["renderer"]
    for body_id, (_, T) in zip(state["ids"], transformations(assembly, angles)):
        model.body_pos[body_id] = T[:3, 3] * .001
        model.body_quat[body_id] = Rotation.from_matrix(T[:3, :3]).as_quat()[[3, 0, 1, 2]]
    mujoco.mj_forward(model, data)
    renderer.update_scene(data, camera=view)
    pixels = np.asarray(renderer.render())[:, :, :3].copy()
    # Compare with the actual skybox pixels; color-space conversion may shift BG.
    border = np.concatenate((pixels[0], pixels[-1], pixels[:, 0], pixels[:, -1]))
    foreground = np.max(np.abs(pixels.astype(float) - np.median(border, axis=0)), axis=2) > 20
    ys, xs = np.where(foreground)
    if len(xs) == 0:
        raise ValueError("Empty offscreen image")
    bounds = [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())]
    if min(bounds[0], bounds[1], size - 1 - bounds[2], size - 1 - bounds[3]) < 3:
        raise ValueError("CAD reaches the image border; refusing a clipped panel")
    return Image.fromarray(pixels), bounds


def font(size):
    for filename in ("C:/Windows/Fonts/malgun.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        if Path(filename).exists():
            return ImageFont.truetype(filename, size)
    return ImageFont.load_default()


def sheet(tiles, columns, size, title, footer):
    rows = (len(tiles) + columns - 1) // columns
    head, label, foot = 75, 48, 75
    image = Image.new("RGB", (columns * size, head + rows * (size + label) + foot), BG)
    draw = ImageDraw.Draw(image)
    draw.text((24, 15), title, font=font(30), fill=INK)
    for i, (name, tile) in enumerate(tiles):
        x, y = (i % columns) * size, head + (i // columns) * (size + label)
        draw.text((x + 16, y + 6), name, font=font(25), fill=INK)
        image.paste(tile, (x, y + label))
    draw.text((24, image.height - foot + 10), footer, font=font(20), fill=INK)
    return image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cad-root", type=Path, default=HERE)
    parser.add_argument("--poses", type=Path, default=HERE / "poses.json")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--tile-size", type=int, default=700)
    parser.add_argument("--compare-reference", action="store_true")
    parser.add_argument("--reference-root", type=Path, default=ROOT / "cad/accepted_form_v1")
    args = parser.parse_args()
    if args.tile_size < 300 or args.tile_size > 1600:
        parser.error("tile-size must be between 300 and 1600")
    if args.output.exists():
        parser.error("Use a new output directory; earlier evidence is never overwritten")
    output = args.output.resolve()
    if output.is_relative_to(args.cad_root.resolve()) or output.is_relative_to(args.reference_root.resolve()):
        parser.error("Render output must be outside the CAD input directories")
    # No file is written until dependencies and every declared input validate.
    import mujoco
    inputs = Inputs()
    inputs.add(__file__); inputs.add(poses.__file__)
    catalog = poses.load_catalog(inputs.add(args.poses))
    candidate = load_candidate(args.cad_root.resolve(), catalog, inputs)
    assemblies = [candidate]
    if args.compare_reference:
        assemblies.append(load_reference(args.reference_root.resolve(), inputs))
    fit_gestures=list(catalog["gestures"])
    if args.compare_reference:
        fit_gestures.append(dict(angles_deg=assemblies[1]["design"]["fist_deg"]))
    cameras = {name: view_fit(assemblies, fit_gestures, direction, 1.)
               for name, direction in VIEWS.items()}
    output.mkdir(parents=True, exist_ok=False)
    records = []; outputs = []
    try:
        tile_cache = {}
        for view, camera in cameras.items():
            tiles = []
            for g in catalog["gestures"]:
                image, bounds = render_one(candidate, g["angles_deg"], view, cameras, args.tile_size, mujoco)
                tile_cache[view, g["id"]] = image
                tiles.append((g["label_ko"], image))
                records.append(dict(source="candidate", gesture=g["id"], view=view,
                                    angles_deg=g["angles_deg"], foreground_bounds_px=bounds,
                                    pose_origin=g.get("pose_origin","unchanged_gesture_candidate")))
            filename = "gestures_" + view + ".png"
            sheet(tiles, 3, args.tile_size,
                  "실제 CAD · 아홉 손 모양 · " + VIEW_LABELS[view] + " / " + candidate["stage"],
                  "같은 카메라·같은 축척 / 저장된 부품만 회전·이동 / 충돌·자연스러움·구동 성능 승인 아님").save(output / filename)
            outputs.append(filename)
        if args.compare_reference:
            reference = assemblies[1]
            for view, camera in cameras.items():
                tiles = []
                for ident in ("open", "fist"):
                    g = next(g for g in catalog["gestures"] if g["id"] == ident)
                    reference_angles=reference["design"]["fist_deg"] if ident=="fist" else np.zeros(20).tolist()
                    image, bounds = render_one(reference, reference_angles, view, cameras, args.tile_size, mujoco)
                    records.append(dict(source="accepted_FORM002", gesture=ident, view=view,
                                        angles_deg=reference_angles, foreground_bounds_px=bounds,
                                        pose_origin="authored_FORM002_fist" if ident=="fist" else "common_open",
                                        candidate_minus_reference_deg=(np.asarray(g["angles_deg"])-reference_angles).tolist()))
                    reference_label=g["label_ko"]+" · "+reference["label"]+(" · 원본 주먹각" if ident=="fist" else " · 같은 펼친 각")
                    candidate_label=g["label_ko"]+" · "+candidate["label"]+(" · 엄지 보정 주먹각" if ident=="fist" else " · 같은 펼친 각")
                    tiles.extend([(reference_label,image),(candidate_label,tile_cache[view, ident])])
                filename = "reference_comparison_" + view + ".png"
                sheet(tiles, 2, args.tile_size,
                      "수용 외형·원본 주먹과 보정 후보 · " + VIEW_LABELS[view],
                      "왼쪽 FORM002 원본 / 오른쪽 저장 CAD · 펼친 손은 같은 각 · 주먹은 엄지만 보정 · 같은 카메라·축척").save(output / filename)
                outputs.append(filename)
        unchanged = inputs.verify()
        if not all(unchanged.values()):
            raise ValueError("Input mutation during rendering: " + str([k for k, v in unchanged.items() if not v]))
        report = dict(schema_version="gesture_hand_v3.render.v1", status="completed",
                      source_sha256=inputs.hashes, inputs_unchanged=True,
                      cad_stage=candidate["stage"], candidate_part_count=len(candidate["parts"]),
                      renderer=dict(name="MuJoCo/Pillow", mujoco_version=mujoco.__version__,
                                    off_screen=True, simulation_steps=0,
                                    tile_size_px=args.tile_size,
                                    opaque_parts=True, no_parts_hidden=True, synthetic_geometry=False),
                      cameras=cameras, renders=records,
                      reference_comparison_pose_policy="Open: identical zero angles. Fist: authored FORM002 angles on the left, explicit corrected candidate thumb angles on the right. Shared camera and scale.",
                      source_coordinate_conventions={a["label"]: a["coordinates"] for a in assemblies},
                      output_sha256={name: sha(output / name) for name in outputs},
                      limitations=["Actual saved CAD triangles are rigidly posed; no generative imagery or exterior replacement.",
                                   "These images do not measure contact, continuous clearance, cable routing or actuator performance.",
                                   "Gesture naturalness remains a visual review; range validity does not establish collision safety.",
                                   "FORM002 comparison uses its accepted local STL shapes, not manufacturing-approved solids."])
        write_json(output / "render_report.json", report)
        print(json.dumps({"output": str(output), "images": outputs, "inputs_unchanged": True}), flush=True)
    except Exception as error:
        write_json(output / "render_failure.json", dict(status="failed", error=repr(error),
                   source_sha256=inputs.hashes, completed_images=outputs))
        raise
    finally:
        for assembly in assemblies:
            if "_render_state" in assembly:
                assembly["_render_state"]["renderer"].close()


if __name__ == "__main__":
    main()
