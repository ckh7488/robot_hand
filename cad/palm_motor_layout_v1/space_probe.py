"""Show current native palm material before changing any motor placement."""
import importlib.util
import json
from pathlib import Path
import sys

import cadquery as cq
import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.dont_write_bytecode = True
spec = importlib.util.spec_from_file_location('_palm_space_source', ROOT/'cad/wiring_layout_v1/build_screen.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
p = m.p


def font(size):
    return ImageFont.truetype('C:/Windows/Fonts/malgun.ttf', size)


def section_edges(shape, z):
    rows = []
    for value in cq.Workplane('XY').add(shape).section(z).vals():
        for edge in value.Edges():
            n = 2 if edge.geomType() == 'LINE' else 41
            rows.append([list(edge.positionAt(float(t)).toTuple()) for t in np.linspace(0, 1, n)])
    return rows


def main():
    out = HERE/'reviews/space.json'
    if out.exists():
        raise FileExistsError('Preserve the initial space inspection')
    scene = p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')
    palmrow = next(r for r in scene['hand'] if r['name'] == 'Palm_tapered_chassis')
    palm = cq.importers.importStep(str(ROOT/palmrow['step'])).val()
    saved = p.read(ROOT/'cad/wiring_layout_v1/generated/packaging.json')['parts']
    palm_parts = [r for r in saved if r['body'] == 'palm']
    src = {r: m.sha(ROOT/r) for r in [
        'cad/palm_motor_layout_v1/space_probe.py',
        'cad/wiring_layout_v1/generated/packaging.json',
        'cad/wiring_layout_v1/reviews/overlap_locations.json',
        'cad/motion_clearance_v2/generated/assembly_catalog.json', palmrow['step']]}
    levels = [-39.5, -21., -5.]
    sections = [dict(z_mm=z, edges=section_edges(palm, z)) for z in levels]
    faces = []
    for f in palm.Faces():
        if f.geomType() == 'PLANE':
            faces.append(dict(center_mm=list(f.Center().toTuple()), normal=list(f.normalAt().toTuple()),
                              area_mm2=f.Area(), bbox_mm=m.pack._bbox(f)))
    evidence = p.read(ROOT/'cad/wiring_layout_v1/reviews/overlap_locations.json')
    print(json.dumps(dict(evidence_keys=list(evidence), sections=[len(r['edges']) for r in sections])), flush=True)
    image = Image.new('RGB', (1800, 840), 'white')
    draw = ImageDraw.Draw(image)
    draw.text((30, 14), '현재 손바닥 실제 CAD 단면 · 모터 위치를 바꾸기 전', font=font(28), fill='#172b40')
    colors = ['#bb6b12', '#7139a3', '#14856b', '#245da1']
    modules = list(dict.fromkeys(r['module'] for r in palm_parts))
    for panel, section in enumerate(sections):
        ox = 300+panel*600
        def xy(point):
            return (ox+float(point[0])*2.7, 680-float(point[1])*2.7)
        draw.text((panel*600+22, 68), f"Z = {section['z_mm']:g} mm", font=font(24), fill='#172b40')
        for edge in section['edges']:
            draw.line([xy(v) for v in edge], fill='#263747', width=3)
        for k, module in enumerate(modules):
            motor = next(r for r in palm_parts if r['module'] == module and r['name'].endswith('_motor'))
            box = motor['bbox_mm']
            x0, y0 = xy([box[0][0], box[1][1]])
            x1, y1 = xy([box[1][0], box[0][1]])
            draw.rectangle((x0, y0, x1, y1), outline=colors[k], width=3)
            draw.text((x0+3, y0+3), str(k+1), font=font(20), fill=colors[k])
        for y in [0, 50, 100, 150]:
            pos = xy([-97, y])
            draw.text(pos, str(y), font=font(15), fill='#536473')
    draw.text((30, 728), '검정: 해당 높이의 실제 손바닥 재료 경계 / 색 사각형: 모터 전체 XY 외곽 참고(단면 아님)', font=font(22), fill='#172b40')
    draw.text((30, 768), '1 엄지 뿌리회전   2 엄지 대립   3 약지 MCP   4 중지 MCP · 손 형상/관절/모터 배치는 아직 변경하지 않음', font=font(22), fill='#172b40')
    (HERE/'images').mkdir(parents=True, exist_ok=True)
    image.save(HERE/'images/current_sections.png')
    report = dict(completed=True, source_sha256=src, palm_faces=faces, sections=sections,
                  palm_parts=palm_parts, hand_geometry_changed=False,
                  interpretation='Actual native palm sections with motor bounding boxes for localization only; not a packing or support verdict.')
    p.write(out, report)
    print(json.dumps(dict(completed=True, plane_faces=len(faces))), flush=True)


if __name__ == '__main__':
    main()
