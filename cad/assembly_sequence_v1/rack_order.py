"""Right column first, then left; retain the obstructed all-neighbours test."""
import importlib.util
from pathlib import Path
import sys
import time

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.dont_write_bytecode = True
spec = importlib.util.spec_from_file_location('_rack_lift', HERE/'module_install.py')
a = importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)
c, b, m, p = a.c, a.b, a.m, a.p


def configurations(saved, new, retained):
    left = {g['module'] for g in saved['groups'] if g['motor_centre_mm'][0] < 0}
    representatives = {'index_mcp_actuator', 'little_mcp_actuator', 'index_spread_actuator',
                       'middle_spread_actuator', 'middle_pip_actuator', 'middle_dip_actuator', 'rack_nuts_after_modules'}
    configs = [r for r in a.configurations(saved, new, retained) if r['id'] in representatives]
    for row in configs:
        if row['id'] not in left and row['id'] != 'rack_nuts_after_modules':
            omitted = [r for r in row['fixed'] if r.get('module') in left]
            row['fixed'] = [r for r in row['fixed'] if r.get('module') not in left]
            row['absent'] += [r['name'] for r in omitted]
    return configs


def main():
    out = HERE/'reviews/rack_order.json'
    if out.exists():
        raise FileExistsError('Preserve prior staged lift evidence')
    previous = p.read(HERE/'reviews/module_install.json')
    # Original all-populated trial was stopped after the repeated ordering
    # block was reproduced. Preserve that partial report, reuse only complete
    # six-sample paths, and run this finite representative study to completion.
    started = time.monotonic()
    saved, new, retained, arm = c.load_scene()
    source = dict(previous['source_sha256'])
    for path in [HERE/'rack_order.py', HERE/'reviews/module_install.json']:
        source[path.relative_to(ROOT).as_posix()] = b.sha(path)
    if not all(b.sha(ROOT/r) == h for r, h in previous['source_sha256'].items()):
        raise ValueError('Original lift inputs changed')
    left = {g['module'] for g in saved['groups'] if g['motor_centre_mm'][0] < 0}
    report = dict(completed=False, source_sha256=source, paths={}, reused_unchanged_paths=[],
                  geometry_changed=False, reporting_floor_mm3=.001, boolean_tolerance_mm=1e-6,
                  sequence=['Preassemble motor/adapter/guides with M3 bolts and spacers off the rack.',
                            'Lift right-column modules before left-column modules exist on the rack.',
                            'Lift left-column modules; finally fit rack nuts from the opposite side.',
                            'Route electrical leads and close lids; attach rack to arm later.'],
                  scope='Offline rack, conditional order and finite rigid samples; not continuous collision, fixture/tool/strength, rack-to-arm or physical assembly qualification.',
                  original_all_neighbours_trial_completed=previous['completed'],
                  original_all_neighbours_clear=False, representative_modules=6,
                  untested_other_module_lift_paths=10, fixtures_qualified=False)
    for path in configurations(saved, new, retained):
        key = path['id']
        earlier = previous['paths'].get(key)
        if (key in left or key == 'rack_nuts_after_modules') and earlier and len(earlier['samples']) == len(path['translations']):
            report['paths'][key] = previous['paths'][key]
            report['reused_unchanged_paths'].append(key)
            continue
        record = dict(moving=[r['name'] for r in path['moving']], obstacles=[r['name'] for r in path['fixed']],
                      explicitly_not_yet_fitted=path['absent'], samples=[])
        report['paths'][key] = record
        for vector in path['translations']:
            result = c.material([dict(name=r['name'], shape=r['shape'].translate(vector)) for r in path['moving']], path['fixed'])
            record['samples'].append(dict(translation_mm=vector, native=result))
            p.write(out, report)
        print(key, sum(len(r['native']['overlaps']) for r in record['samples']), flush=True)
    report.update(completed=True, elapsed_s=time.monotonic()-started,
                  input_hash_guard=all(b.sha(ROOT/r) == h for r, h in source.items()),
                  conditional_sampled_rack_order_clear=all(not row['native']['overlaps'] and not row['native']['operation_failures']
                                                         for path in report['paths'].values() for row in path['samples']))
    p.write(out, report)
    if not report['input_hash_guard']:
        raise ValueError('Changed ordered lift inputs')


if __name__ == '__main__':
    main()
