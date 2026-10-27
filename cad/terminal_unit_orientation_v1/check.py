"""Changed terminal packet, its real local cords and the original diagnostic arm poses."""
import importlib.util
import sys
import time
from pathlib import Path

sys.dont_write_bytecode = True
spec = importlib.util.spec_from_file_location('_arm056_model', Path(__file__).with_name('model.py'))
a = importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)
n, np, d, ROOT, HERE = a.n, a.np, a.d, a.ROOT, a.HERE
from OCP.OSD import OSD_ThreadPool


def main():
    out = HERE/'reviews/check.json'
    assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads() == 1
    start = time.monotonic()
    _, baseline, fixed, packet, T, S0, S, prior, older, source = a.context()
    changed, design = a.build(packet, T, S0, S)
    cords, routes = a.cords(T, S0)
    for f in [Path(__file__), HERE/'model.py']:
        source[f.relative_to(ROOT).as_posix()] = n.m.drive.sha(f)
    scene = n.a.a.prior.export(changed[:1], HERE/'generated/hardware')
    cord_scene = n.a.a.prior.export(cords, HERE/'generated/local_cords')
    old_records = list(prior['inherited_unchanged_overlap_records'])
    old_records += [dict(group=k, record=z) for k,v in prior['native'].items() for z in v['overlaps']]
    inherited = [z for z in old_records if not ({z['record']['lead'],z['record']['part']} & a.members())]
    changed_old = [z for z in old_records if ({z['record']['lead'],z['record']['part']} & a.members())]
    raw = dict(completed=False, source_sha256=source, scene=scene, cord_scene=cord_scene,
        design=design, local_routes=routes, proposal_rows=872, unchanged_rows=853,
        rotated_existing_hardware_rows=18, new_frame_rows=1, current862_installation_replaced=False,
        current_hand397_base364_16remote_sources_preserved=True, candidate_adopted=False,
        motor_installation_and_command_changed_from_ARM054=False, local_cord_reference_rows=12,
        long_housings_or_cords_installed=False, service_paths_executed=0,
        inherited_unchanged_overlap_records=inherited, old_changed_terminal_records_not_carried=changed_old,
        inherited_records_native_reexecuted=False, native={}, motion=[],
        reporting_floor_mm3=.001, boolean_tolerance_mm=1e-6, OCC_pool_threads=1)
    def save():
        n.a.save(out, raw)
    def run(label, left, right):
        raw['native'][label] = n.c.material(left, right)
        save()
        print(label,raw['native'][label]['exact_pairs'],len(raw['native'][label]['overlaps']),flush=True)
    save()
    others = [z for v in fixed.values() for z in v]
    hardware = changed+others
    run('changed19__unchanged853_neutral', changed, others)
    run('new_frame__rotated18', changed[:1], changed[1:])
    run('local_cords12__hardware872', cords, hardware)
    internal = [n.c.material([z], cords[i+1:]) for i,z in enumerate(cords)]
    raw['native']['local_cords_internal'] = d.k.flat(internal)
    save()
    arm = n.p.read(ROOT/'cad/modular_roots_v9/design.json')['arm']
    for state in older['motion']:
        F = d.transforms(state['angles_deg'], arm)
        moving = [dict(z, shape=n.g.original.transform(z['shape'], F['forearm'])) for z in changed]
        for target in state['compared_frames']:
            obstacle = [dict(z, shape=n.g.original.transform(z['shape'], F[target])) for z in fixed[target]]
            run(state['id']+'__changed19__'+target, moving, obstacle)
        raw['motion'].append(dict(id=state['id'],angles_deg=state['angles_deg'],compared_frames=state['compared_frames'],transforms=F))
        save()
    adapter = next(z for z in others if z['name'] == 'thumb_ip_actuator_adapter')['shape']
    raw['bridge_adapter_distance_mm'] = float(changed[0]['shape'].distance(adapter))
    total = d.k.flat(list(raw['native'].values()))
    raw.update(completed=True,native_groups=len(raw['native']),pairs_in_scope=total['pairs_in_scope'],
        AABB_native_pairs=total['exact_pairs'],overlap_records=len(total['overlaps']),
        operation_failures=len(total['operation_failures']),changed_geometry_raw_clear=not total['overlaps'] and not total['operation_failures'],
        full_proposal_raw_clear=False,inherited_records_count=len(inherited),old_terminal_records_not_carried_count=len(changed_old),
        input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()),elapsed_s=time.monotonic()-start)
    save()


if __name__ == '__main__':
    main()
