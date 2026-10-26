"""Exact-commit replay of original blocks and selected staged alternatives."""
import argparse
import importlib.util
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.dont_write_bytecode = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--commit', required=True)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    if args.output.exists() or args.output.resolve().is_relative_to(ROOT):
        raise ValueError('New external output required')
    def git(*xs):
        return subprocess.check_output(['git', *xs], cwd=ROOT, text=True, encoding='utf-8', creationflags=subprocess.CREATE_NO_WINDOW).strip()
    if git('rev-parse', 'HEAD') != args.commit or git('status', '--porcelain=v1'):
        raise ValueError('Wrong or dirty target')
    spec = importlib.util.spec_from_file_location('_assembly_core', HERE/'check.py')
    c = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(c)
    b, m, p = c.b, c.m, c.p
    def module(name, file):
        spec = importlib.util.spec_from_file_location(name, HERE/file)
        out = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(out)
        return out
    lift = module('_assembly_lift_core', 'module_install.py')
    pre = module('_assembly_pre_core', 'preassemble.py')
    ordered = module('_assembly_order_core', 'rack_order.py')
    manifest = p.read(ROOT/'checkpoints/ASSEMBLY012.manifest.json')
    frozen = {}
    for key, value in p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json').items():
        if key in ['geometry_sha256', 'code_sha256', 'report_sha256']:
            frozen.update(value)
    def guard():
        return all(b.sha(ROOT/r) == row['sha256'] for r, row in manifest['files'].items()) and all(b.sha(ROOT/r) == h for r, h in frozen.items())
    if not guard():
        raise ValueError('Changed source or frozen evidence')
    saved, new, retained, arm = c.load_scene()
    assert len(new) == 160 and len(retained) == 193 and len(arm) == 10
    families = {
        'base_hardware': (c.configurations(new, retained, arm), p.read(HERE/'reviews/base_hardware.json')),
        'module_install': (lift.configurations(saved, new, retained), p.read(HERE/'reviews/module_install.json')),
        'preassemble': (pre.configurations(saved, new, retained, arm), p.read(HERE/'reviews/preassemble.json')),
        'rack_order': (ordered.configurations(saved, new, retained), p.read(HERE/'reviews/rack_order.json'))}
    if not all(report['completed'] for family, (configs, report) in families.items() if family != 'module_install'):
        raise ValueError('Incomplete original study')
    selection = [
        ('base_hardware', 'nuts_before_base_screws', 2),
        ('base_hardware', 'base_screws_after_nuts', 2),
        ('base_hardware', 'base_screws_after_nuts', 4),
        ('preassemble', 'base_screws_before_motor', 2),
        ('preassemble', 'index_spread_actuator', 3),
        ('preassemble', 'middle_dip_actuator', 3),
        ('module_install', 'little_mcp_actuator', 1),
        ('module_install', 'index_mcp_actuator', 2),
        ('rack_order', 'middle_spread_actuator', 3),
        ('rack_order', 'middle_dip_actuator', 4),
        ('rack_order', 'middle_pip_actuator', 4),
        ('rack_order', 'rack_nuts_after_modules', 2)]
    differences = []
    results = []
    for family, key, index in selection:
        configs, expected = families[family]
        config = next(x for x in configs if x['id'] == key)
        want = expected['paths'][key]
        if [r['name'] for r in config['moving']] != want['moving'] or [r['name'] for r in config['fixed']] != want['obstacles']:
            raise ValueError('Obstacle/stage definition differs')
        vector = config['translations'][index]
        if vector != want['samples'][index]['translation_mm']:
            raise ValueError('Different sample')
        got = c.material([dict(name=r['name'], shape=r['shape'].translate(vector)) for r in config['moving']], config['fixed'])
        expected_native = want['samples'][index]['native']
        signature = lambda r: {(x['lead'], x['part']): x['material_intersection_mm3'] for x in r['overlaps']}
        aa, bb = signature(got), signature(expected_native)
        matched = aa.keys() == bb.keys() and all(abs(aa[k]-bb[k]) <= 1e-5 for k in aa.keys() & bb.keys())
        matched = matched and got['operation_failures'] == expected_native['operation_failures'] and got['pairs_in_scope'] == expected_native['pairs_in_scope']
        if not matched:
            differences.append(dict(family=family, path=key, index=index))
        results.append(dict(family=family, path=key, index=index, translation_mm=vector, native=got, matched=matched))
        print(family, key, index, 'matched', matched, flush=True)
    still_clean = not git('status', '--porcelain=v1')
    report = dict(completed=True, target_commit=args.commit, manifest_files=len(manifest['files']), frozen_files=len(frozen),
                  native_parts_reimported=353, neutral_arm_parts=10, key_samples=results, differences=differences,
                  input_hash_guard=guard(), checkout_clean=still_clean, independent_audit=False,
                  scope='12key rigid samples, including original failures and staged alternatives. Original complete studies, images, hand motion, fixtures/tools, factory motor internals, continuous collision, electrical/physical performance not rerun or approved.',
                  passed=not differences and guard() and still_clean)
    p.write(args.output, report)
    if not report['passed']:
        raise ValueError('Exact-SHA assembly replay differs')


if __name__ == '__main__':
    main()
