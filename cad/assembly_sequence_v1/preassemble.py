"""Seat the motor after inserting the cradle's base fasteners first."""
import importlib.util
from pathlib import Path
import sys
import time

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.dont_write_bytecode = True
spec = importlib.util.spec_from_file_location('_hardware_preassemble', HERE/'check.py')
c = importlib.util.module_from_spec(spec)
spec.loader.exec_module(c)
s, g, b, m, p = c.s, c.g, c.b, c.m, c.p


def configurations(saved, new, retained, arm):
    remote = {r['module'] for r in saved['groups']}
    is_motor = lambda r: r.get('module') in remote and r['name'].endswith(('_motor', '_rotor', '_body_screws'))
    motors = [r for r in retained if is_motor(r)]
    screws = [r for r in retained if r.get('module') in remote and r['name'].endswith('_base_screws')]
    screw_names = {r['name'] for r in screws}
    configs = [dict(id='base_screws_before_motor', moving=screws,
                    fixed=[r for r in retained if r['name'] not in screw_names and not is_motor(r)]+new+arm,
                    absent=[r['name'] for r in motors],
                    translations=[[x, 0., 0.] for x in [8., 6., 4., 2., 0.]])]
    # Build one module away from the rack, then lift that complete unit into
    # the populated rack in module_install.py. Own case, cradle, adapter,
    # fasteners and guides remain in this test; neighbours physically belong
    # to the later rack-installation stage, not this bench subassembly.
    for group in saved['groups']:
        name = group['module']
        if name not in ['index_spread_actuator', 'middle_spread_actuator', 'middle_pip_actuator', 'middle_dip_actuator']:
            continue
        moving = [r for r in motors if r['module'] == name]
        names = {r['name'] for r in moving}
        assert len(moving) == 3
        own = [r for r in retained+new if r.get('module') == name]
        fixed = [r for r in own if r['name'] not in names and '_rack_nut_' not in r['name']]
        configs.append(dict(id=name, moving=moving,
                            fixed=fixed,
                            absent=[r['name'] for r in retained+new if r['name'] not in names and r not in fixed],
                            translations=[[x, 0., 0.] for x in [20., 15., 10., 5., 0.]]))
    return configs


def main():
    out = HERE/'reviews/preassemble.json'
    if out.exists():
        raise FileExistsError('Preserve prior preassembly evidence')
    started = time.monotonic()
    saved, new, retained, arm = c.load_scene()
    source = {path.relative_to(ROOT).as_posix(): b.sha(path)
              for path in [HERE/'preassemble.py', HERE/'check.py', ROOT/'cad/wire_guides_v1/generated/scene.json']}
    for row in new+retained:
        source[row['step']] = b.sha(ROOT/row['step'])
    report = dict(completed=False, source_sha256=source, paths={}, geometry_changed=False,
                  reporting_floor_mm3=.001, boolean_tolerance_mm=1e-6,
                  representative_motor_modules=4,
                  scope='Screw-before-motor alternative: all16base screw compounds against retained/new/neutral arm with motor units fitted later; four representative motor units against their own complete cradle/adapter/guides/fasteners away from rack, then separate whole-module rack insertion. Finite samples only; other12motor seating paths, tools, factory motor internal assembly, temporary holding, load/torque and complete integration unverified.',
                  explicitly_later=['Electrical leads and plug boxes'], intended_positive_overlap_exemptions=[])
    for path in configurations(saved, new, retained, arm):
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
                  sampled_preassembly_clear=all(not row['native']['overlaps'] and not row['native']['operation_failures']
                                               for path in report['paths'].values() for row in path['samples']))
    p.write(out, report)
    if not report['input_hash_guard']:
        raise ValueError('Changed preassembly inputs')


if __name__ == '__main__':
    main()
