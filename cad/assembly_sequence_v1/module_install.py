"""Install preassembled remote modules upwards with captive rack bolts.

Rack nuts are fitted after the lift. Electrical leads are routed later. The
entire rack is assembled off the arm, giving access below it. Temporary
holding and the later rack-to-arm installation are separate open tasks.
"""
import importlib.util
from pathlib import Path
import sys
import time

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.dont_write_bytecode = True
spec = importlib.util.spec_from_file_location('_full_hardware', HERE/'check.py')
c = importlib.util.module_from_spec(spec)
spec.loader.exec_module(c)
s, g, b, m, p = c.s, c.g, c.b, c.m, c.p


def configurations(saved, new, retained):
    # Conservative neighbours: every other module is already present. Own
    # rack nuts are left out until the bolts have emerged through the rail.
    paths = []
    for group in saved['groups']:
        name = group['module']
        own = [r for r in retained+new if r.get('module') == name]
        rack_nuts = [r for r in own if '_rack_nut_' in r['name']]
        assert len(rack_nuts) == 2
        moving = [r for r in own if r not in rack_nuts]
        fixed = [r for r in retained+new if r.get('module') != name]
        paths.append(dict(id=name, moving=moving, fixed=fixed,
                          absent=[r['name'] for r in rack_nuts],
                          translations=[[0., 0., z] for z in [-35., -28., -21., -14., -7., 0.]]))
    nuts = [r for r in retained if r.get('module') in {g['module'] for g in saved['groups']} and '_rack_nut_' in r['name']]
    names = {r['name'] for r in nuts}
    paths.append(dict(id='rack_nuts_after_modules', moving=nuts,
                      fixed=[r for r in retained+new if r['name'] not in names], absent=[],
                      translations=[[0., 0., z] for z in [8., 6., 4., 2., 0.]]))
    return paths


def main():
    out = HERE/'reviews/module_install.json'
    if out.exists():
        raise FileExistsError('Preserve prior module installation evidence')
    started = time.monotonic()
    saved, new, retained, arm = c.load_scene()
    source = {str(path.relative_to(ROOT).as_posix()): b.sha(path)
              for path in [HERE/'module_install.py', HERE/'check.py', ROOT/'cad/wire_guides_v1/generated/scene.json']}
    for row in new+retained:
        source[row['step']] = b.sha(ROOT/row['step'])
    report = dict(completed=False, source_sha256=source, paths={}, geometry_changed=False,
                  reporting_floor_mm3=.001, boolean_tolerance_mm=1e-6,
                  operation='Offline rack assembly, lift a preassembled module vertically from below with its M3 bolts/spacers/guide already on the adapter; fit its two rack nuts later.',
                  scope='Actual separate solids against full rack and all other modules/guides. Finite samples only, no continuous collision/tool/fixture/thread/strength/rack-to-arm or physical assembly approval.',
                  explicitly_later=['Own rack nuts', 'Electrical leads and plug boxes', 'Rack-to-arm installation'],
                  electrical_paths_preserved=True, fixtures_qualified=False, arm_present=False)
    for path in configurations(saved, new, retained):
        record = dict(moving=[r['name'] for r in path['moving']], obstacles=[r['name'] for r in path['fixed']],
                      explicitly_not_yet_fitted=path['absent'], samples=[])
        report['paths'][path['id']] = record
        for vector in path['translations']:
            result = c.material([dict(name=r['name'], shape=r['shape'].translate(vector)) for r in path['moving']], path['fixed'])
            record['samples'].append(dict(translation_mm=vector, native=result))
            p.write(out, report)
        print(path['id'], sum(len(r['native']['overlaps']) for r in record['samples']), flush=True)
    report.update(completed=True, elapsed_s=time.monotonic()-started,
                  input_hash_guard=all(b.sha(ROOT/r) == h for r, h in source.items()),
                  sampled_module_paths_clear=all(not row['native']['overlaps'] and not row['native']['operation_failures']
                                                for path in report['paths'].values() for row in path['samples']))
    p.write(out, report)
    if not report['input_hash_guard']:
        raise ValueError('Changed module inputs')


if __name__ == '__main__':
    main()
