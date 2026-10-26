"""Full-obstacle insertion of the retained motor base fasteners.

The prior guide study compared these fasteners to NEW guides only. Here the
original cases, cradles, adapters, rack and other hardware are also present.
Parts scheduled later are enumerated rather than silently exempted.
"""
import argparse
import importlib.util
from pathlib import Path
import sys
import time
import types

import cadquery as cq

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.dont_write_bytecode = True
spec = importlib.util.spec_from_file_location('_assembly_service', ROOT/'cad/wire_guides_v1/service_check.py')
s = importlib.util.module_from_spec(spec)
spec.loader.exec_module(s)
g, b, m, p = s.g, s.b, s.m, s.p


def load_scene():
    saved = p.read(ROOT/'cad/wire_guides_v1/generated/scene.json')
    new = s.load(saved['new_parts'])
    retained = s.load(saved['retained_packaging'])
    arm = [dict(name=r['name'], shape=cq.importers.importStep(str(ROOT/'cad/modular_roots_v9/parts'/(r['asset']+'.step'))).val())
           for r in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference']]
    return saved, new, retained, arm


def configurations(new, retained, arm):
    remote = {r['module'] for r in new}
    screws = [r for r in retained if r.get('module') in remote and r['name'].endswith('_base_screws')]
    nuts = [r for r in retained if r.get('module') in remote and r['name'].endswith('_base_nuts')]
    screw_names = {r['name'] for r in screws}
    nut_names = {r['name'] for r in nuts}
    assert len(screws) == len(nuts) == 16
    common = new + arm
    return [dict(id='nuts_before_base_screws', moving=nuts,
                 fixed=[r for r in retained if r['name'] not in screw_names | nut_names] + common,
                 translations=[[-x, 0., 0.] for x in [8., 6., 4., 2., 0.]],
                 absent=sorted(screw_names), later='Electrical leads, plugs and lid closure are fitted later; lids are conservatively included as obstacles.'),
            dict(id='base_screws_after_nuts', moving=screws,
                 fixed=[r for r in retained if r['name'] not in screw_names] + common,
                 translations=[[x, 0., 0.] for x in [8., 6., 4., 2., 0.]],
                 absent=[], later='Electrical leads and plugs are fitted later; lids are conservatively included as obstacles.')]


def material(moving, fixed):
    return b.test_against(moving, fixed, types.SimpleNamespace(bounds=m.pack._bbox))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--output', type=Path, default=HERE/'reviews/base_hardware.json')
    args = ap.parse_args()
    if args.output.exists():
        raise FileExistsError('Preserve earlier insertion evidence')
    started = time.monotonic()
    saved, new, retained, arm = load_scene()
    paths = configurations(new, retained, arm)
    source_paths = [HERE/'check.py', ROOT/'cad/wire_guides_v1/generated/scene.json',
                    ROOT/'cad/wire_guides_v1/service_check.py', ROOT/'cad/modular_roots_v9/build_candidate.py',
                    ROOT/'cad/form_candidate_v1/generated/scene.json']
    source = {path.relative_to(ROOT).as_posix(): b.sha(path) for path in source_paths}
    for row in new + retained:
        source[row['step']] = b.sha(ROOT/row['step'])
    report = dict(completed=False, source_sha256=source, paths={},
                  reporting_floor_mm3=.001, boolean_tolerance_mm=1e-6,
                  screw_compounds=16, nut_compounds=16, fasteners_per_compound=4,
                  geometry_changed=False, upstream_palm_overlaps=8,
                  intended_positive_overlap_exemptions=[],
                  scope='Finite rigid insertion samples against complete retained packaging, new guides and neutral arm. Not continuous collision, tool/fixture, thread/torque, module installation or physical assembly approval.')
    for path in paths:
        record = dict(moving=[r['name'] for r in path['moving']], obstacles=[r['name'] for r in path['fixed']],
                      explicitly_not_yet_fitted=path['absent'], later=path['later'], samples=[])
        report['paths'][path['id']] = record
        for vector in path['translations']:
            moved = [dict(name=r['name'], shape=r['shape'].translate(vector)) for r in path['moving']]
            result = material(moved, path['fixed'])
            record['samples'].append(dict(translation_mm=vector, native=result))
            p.write(args.output, report)
            print(path['id'], vector, len(result['overlaps']), len(result['operation_failures']), flush=True)
    report.update(completed=True, elapsed_s=time.monotonic()-started,
                  input_hash_guard=all(b.sha(ROOT/r) == h for r, h in source.items()),
                  sampled_hardware_paths_clear=all(not row['native']['overlaps'] and not row['native']['operation_failures']
                                                   for path in report['paths'].values() for row in path['samples']))
    p.write(args.output, report)
    if not report['input_hash_guard']:
        raise ValueError('Changed inputs')


if __name__ == '__main__':
    main()
