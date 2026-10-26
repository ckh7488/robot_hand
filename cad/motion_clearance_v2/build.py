"""Candidate-only palm rim relief. Existing hand parts and joint axes are frozen.

The sloped cutter bounds the original proximal shell's lower box face at -26
degrees. A 0.7 mm offset and widened lane leave room for the -25 degree probe.
This is a geometric construction, not a strength or manufacturing tolerance.
"""
import argparse, copy, hashlib, json, math
from pathlib import Path
import cadquery as cq

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SOURCE = ROOT / 'cad/thumb_ip_drive_v1/generated'


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def resolve(row, key):
    return ROOT / row[key] if row['location'] == 'repo' else SOURCE / row[key]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    out = args.output
    (out / 'parts').mkdir(parents=True)
    (out / 'meshes').mkdir()
    source_scene = SOURCE / 'scene.json'
    design_path = ROOT / 'cad/manufacturing_v2/generated/design.json'
    design = read(design_path)
    scene = copy.deepcopy(read(source_scene))
    hashes = {p.relative_to(ROOT).as_posix(): sha(p) for p in [Path(__file__), source_scene, design_path]}
    parts = {}
    for row in scene['hand']:
        for key, hk in [('step', 'sha256_step'), ('mesh', 'sha256_stl')]:
            path = resolve(row, key).resolve()
            if sha(path) != row[hk]:
                raise ValueError('Changed source ' + row['name'])
            hashes[path.relative_to(ROOT).as_posix()] = row[hk]
            row[key] = path.relative_to(ROOT).as_posix()
        row['location'] = 'repo'
        if row['name'] == 'Palm_tapered_chassis' or row['name'].endswith('_segment1_segment_shell'):
            shape = cq.importers.importStep(str(ROOT / row['step'])).val()
            if not shape.isValid() or len(shape.Solids()) != 1:
                raise ValueError('Invalid source ' + row['name'])
            parts[row['name']] = shape
    original = parts['Palm_tapered_chassis']
    palm = original
    cuts = []
    margin, shaping_angle, floor_guard = .7, 26., -39.5
    for finger in design['fingers']:
        if finger.get('base_yaw_deg', 0) or finger.get('flexion_offset', [0, 0, 0]) != [0, 0, 0]:
            raise ValueError('Re-derive relief for a changed root frame')
        name = finger['name']
        root = finger['root']
        box = parts[name + '_segment1_segment_shell'].BoundingBox()
        half_width = max(root[0]-box.xmin, box.xmax-root[0]) + margin
        local_bottom = box.zmin - root[2]
        slope = math.tan(math.radians(shaping_angle))
        intercept = local_bottom/math.cos(math.radians(shaping_angle)) - margin
        reach = box.ymax-root[1]+3
        points = [(0, intercept), (reach, intercept-slope*reach), (reach, 70), (0, 70)]
        wire = cq.Wire.makePolygon([cq.Vector(-half_width, y, z) for y, z in points], close=True)
        cutter = cq.Solid.extrudeLinear(wire, [], cq.Vector(2*half_width, 0, 0)).translate(root)
        above_floor = cq.Solid.makeBox(400, 500, 150, cq.Vector(-200, -100, floor_guard))
        cutter = cutter.intersect(above_floor)
        before = palm.Volume()
        palm = palm.cut(cutter).clean()
        if not palm.isValid() or len(palm.Solids()) != 1:
            raise ValueError('Relief disconnects palm at ' + name)
        cuts.append(dict(finger=name, root_mm=root, half_width_mm=half_width,
                         local_y_span_mm=[0, reach], local_z_lower_intercept_mm=intercept,
                         local_z_lower_slope=-slope, removed_mm3=before-palm.Volume()))
    if palm.cut(original).Volume() > 1e-6:
        raise ValueError('Unexpected added material')
    # Explicitly preserve the entire existing floor and each complete rear root
    # mounting footprint. A valid connected solid alone cannot prove these seats.
    floor_mask = cq.Solid.makeBox(400, 500, 20, cq.Vector(-200, -100, -60.5))
    preserved = {'existing_floor_below_z_minus40_5_removed_mm3': original.intersect(floor_mask).cut(palm).Volume()}
    for finger in design['fingers']:
        x, y, z = finger['root']
        mask = cq.Solid.makeBox(32, 14, 40, cq.Vector(x-16, y-15, -50))
        preserved[finger['name']+'_rear_root_mount_removed_mm3'] = original.intersect(mask).cut(palm).Volume()
    if max(preserved.values()) > 1e-6:
        raise ValueError('A protected floor/root mount lost material')
    target = next(row for row in scene['hand'] if row['name'] == 'Palm_tapered_chassis')
    step, mesh = out/'parts/Palm_tapered_chassis.step', out/'meshes/Palm_tapered_chassis.stl'
    cq.exporters.export(palm, str(step))
    cq.exporters.export(palm, str(mesh), tolerance=.03, angularTolerance=.08)
    box = palm.BoundingBox()
    target.update(step=step.relative_to(ROOT).as_posix(), mesh=mesh.relative_to(ROOT).as_posix(),
                  sha256_step=sha(step), sha256_stl=sha(mesh), volume_mm3=palm.Volume(),
                  bbox_mm=[[box.xmin, box.ymin, box.zmin], [box.xmax, box.ymax, box.zmax]],
                  source='MOTION002_palm_rim_relief')
    # Preserve the source metadata, including the explicit fact that the separate
    # motor bench and incomplete tendon paths are not installed whole-hand parts.
    (out/'scene.json').write_text(json.dumps(scene, indent=2), encoding='utf-8')
    (out/'design.json').write_bytes(design_path.read_bytes())
    report = dict(completed=True, errors=[], source_sha256=hashes, hand_parts=len(scene['hand']),
                  modified_names=['Palm_tapered_chassis'], unchanged_parts=len(scene['hand'])-1,
                  shaping_angle_deg=-shaping_angle, target_mcp_extension_deg=-25,
                  cutter_clearance_mm=margin, cutter_floor_limit_z_mm=floor_guard,
                  original_volume_mm3=original.Volume(), candidate_volume_mm3=palm.Volume(),
                  removed_volume_mm3=original.cut(palm).Volume(), added_volume_mm3=palm.cut(original).Volume(),
                  preserved_regions=preserved, lane_cuts=cuts, valid_single_solid=True,
                  axes_lengths_and_poses_changed=False, manufacturing_release=False,
                  remaining=['Root carrier and housing still interfere with proximal shells',
                             'No structural strength, print tolerances, full assembly or actuation approval'])
    report['input_hash_guard'] = {'passed': all(sha(ROOT/k) == h for k, h in hashes.items())}
    (out/'build_report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ['source_sha256','lane_cuts']}, indent=2), flush=True)


if __name__ == '__main__':
    main()
