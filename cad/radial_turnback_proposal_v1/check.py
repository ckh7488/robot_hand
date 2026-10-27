"""Check only the two changed guides against every retained actual solid."""
import importlib.util
import sys
import time
from pathlib import Path

sys.dont_write_bytecode = True
spec = importlib.util.spec_from_file_location('_arm055_model', Path(__file__).with_name('model.py'))
a = importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)
n, np, d, ROOT, HERE = a.n, a.np, a.d, a.ROOT, a.HERE
from OCP.OSD import OSD_ThreadPool


def main():
    out = HERE/'reviews/check.json'
    assert not out.exists()
    started = time.monotonic()
    _, baseline, retained, T, prior, source = a.context()
    new, design = a.build(T, source)
    for f in [HERE/'model.py', Path(__file__)]:
        source[f.relative_to(ROOT).as_posix()] = n.m.drive.sha(f)
    assert OSD_ThreadPool.DefaultPool_s().NbThreads() == 1
    scene = n.a.a.prior.export(new, HERE/'generated/hardware')
    inherited = []
    replaced_records = []
    for label, test in prior['native'].items():
        for z in test['overlaps']:
            (replaced_records if z['lead'] in a.OLD_NAMES or z['part'] in a.OLD_NAMES else inherited).append(dict(group=label, record=z))
    for test in prior['inter_module_groups']:
        for z in test['native']['overlaps']:
            (replaced_records if z['lead'] in a.OLD_NAMES or z['part'] in a.OLD_NAMES else inherited).append(dict(
                group='modules__'+test['left']+'__'+test['right'], record=z))
    assert len(inherited)+len(replaced_records) == prior['overlap_records']
    raw = dict(completed=False, source_sha256=source, scene=scene, design=design,
        current862_installation_replaced=False, separate_proposal_rows=872,
        ARM054_installed_or_adopted=False, proposal_adopted=False,
        replaced_two_guides=a.OLD_NAMES, new_hardware_rows=2, retained_rows=870,
        motor_installation_or_command_changed_from_ARM054=False,
        guide_radius_OD_and_length_changed=False, long_materials_present=False,
        inherited_unchanged_overlap_records=inherited,
        old_replaced_guide_records_not_claimed_applicable=replaced_records,
        inherited_records_retested=False, prior156_groups_reexecuted=False,
        native={}, motion=[], service_paths_executed=0,
        reporting_floor_mm3=.001, boolean_tolerance_mm=1e-6, OCC_pool_threads=1,
        strength_preload_friction_tools_assembly_whole_drive_exterior_approved=False)
    def save():
        n.a.save(out, raw)
    def run(label, left, right):
        test = n.c.material(left, right)
        raw['native'][label] = test
        save()
        print(label, test['exact_pairs'], len(test['overlaps']), flush=True)
    save()
    all_retained = [z for rows in retained.values() for z in rows]
    run('new2__retained870_neutral', new, all_retained)
    run('new2_internal', new[:1], new[1:])
    arm = n.p.read(ROOT/'cad/modular_roots_v9/design.json')['arm']
    # The same diagnostic angles define the new guides' tests, not prior-result replay.
    for state in prior['motion']:
        F = d.transforms(state['angles_deg'], arm)
        moving = [dict(z, shape=n.g.original.transform(z['shape'], F['forearm'])) for z in new]
        for target in state['compared_frames']:
            obstacle = [dict(z, shape=n.g.original.transform(z['shape'], F[target])) for z in retained[target]]
            run(state['id']+'__new2__'+target, moving, obstacle)
        raw['motion'].append(dict(id=state['id'], angles_deg=state['angles_deg'],
            compared_frames=state['compared_frames'], transforms=F))
        save()
    tests = list(raw['native'].values())
    total = d.k.flat(tests)
    raw.update(completed=True, native_groups=len(tests), pairs_in_scope=total['pairs_in_scope'],
        AABB_native_pairs=total['exact_pairs'], overlap_records=len(total['overlaps']),
        operation_failures=len(total['operation_failures']),
        new_geometry_raw_clear=not total['overlaps'] and not total['operation_failures'],
        full_proposal_raw_clear=False, inherited_unchanged_records_count=len(inherited),
        original_replaced_records_count=len(replaced_records),
        old_and_new_records_are_not_full_assembly_independent_defect_count=True,
        input_hash_guard=all(n.m.drive.sha(ROOT/f) == h for f, h in source.items()),
        elapsed_s=time.monotonic()-started)
    save()
    print('completed', raw['native_groups'], raw['overlap_records'], len(inherited), flush=True)


if __name__ == '__main__':
    main()
