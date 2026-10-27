"""Exact-SHA bounded replay of a separate installation-axis proposal."""
import argparse
import importlib.util
import subprocess
import sys
from pathlib import Path

sys.dont_write_bytecode = True
spec = importlib.util.spec_from_file_location('_arm053_review_model', Path(__file__).with_name('model.py'))
p = importlib.util.module_from_spec(spec)
spec.loader.exec_module(p)
n, np, ROOT, HERE, d = p.n, p.np, p.ROOT, p.HERE, p.d
measure = n.m.drive.module('_arm053_metrics', ROOT/'cad/long_housing_route_v1/review_core.py')
from OCP.OSD import OSD_ThreadPool
from OCP.BRepExtrema import BRepExtrema_DistShapeShape


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--commit', required=True)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    def git(*xs):
        return subprocess.check_output(['git', *xs], cwd=ROOT, text=True,
            encoding='utf-8', creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse', 'HEAD') == args.commit and not git('status', '--porcelain=v1')
    assert not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
    manifest = n.p.read(ROOT/'checkpoints/ARM053.manifest.json')
    frozen = {}
    for key in ['geometry_sha256', 'code_sha256', 'report_sha256']:
        frozen.update(n.p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[key])
    def guard():
        return all(n.m.drive.sha(ROOT/f) == z['sha256'] for f, z in manifest['files'].items()) and all(
            n.m.drive.sha(ROOT/f) == h for f, h in frozen.items())
    assert guard() and OSD_ThreadPool.DefaultPool_s().NbThreads() == 1
    raw = n.p.read(HERE/'reviews/check.json')
    evidence = n.p.read(HERE/'reviews/evidence.json')
    frames, _ = d.inputs()
    _, modules, bundles, rack = p.prior.s.bundles(frames)
    moved, contract = p.placement(modules, bundles)
    fresh = p.cage()
    saved = n.c.load(n.p.read(ROOT/raw['scene'])['parts'])
    names = {z['name'] for z in moved} | {rack['name']}
    unchanged = {key: [z for z in rows if z['name'] not in names] for key, rows in frames.items()}
    result = dict(completed=False, target_commit=args.commit, differences=[], native_samples=[],
        geometry_comparisons=[], placements=[], contacts=[], ports=[],
        manifest_files=len(manifest['files']), frozen_files=len(frozen), OCC_pool_threads=1,
        current_installation_replaced=False, same_agent_not_independent_audit=True,
        whole_drive_or_exterior_approved=False)
    def save():
        n.a.save(args.output, result)
    save()
    for left, right in zip(fresh, saved):
        assert left['name'] == right['name']
        item = dict(name=left['name'], **measure.match(measure.metrics(left['shape']), measure.metrics(right['shape'])))
        result['geometry_comparisons'].append(item)
        if not item['matched']:
            result['differences'].append('geometry_'+left['name'])
        save()
    for got, want in zip(contract, raw['placements']):
        err = max(float(np.max(abs(np.array(got[k])-want[k]))) for k in
                  ['proposed_installation_transform', 'transform_from_current', 'proposed_centre_mm', 'output_axis_world'])
        ok = got['module'] == want['module'] and got['rows'] == want['rows'] and err < 1e-10
        result['placements'].append(dict(module=got['module'], rows=len(got['rows']), maximum_difference=err, matched=bool(ok)))
        if not ok:
            result['differences'].append('placement_'+got['module'])
    def compare(label, got, want):
        a, b = d.overlap_map(got), d.overlap_map(want)
        err = max([abs(a[k]['material_intersection_mm3']-b[k]['material_intersection_mm3'])
                   for k in a.keys() & b.keys()]+[0.])
        ok = a.keys() == b.keys() and err < 1e-5 and got['operation_failures'] == want['operation_failures'] and got['pairs_in_scope'] == want['pairs_in_scope']
        result['native_samples'].append(dict(id=label, native=got, maximum_volume_difference_mm3=err, matched=bool(ok)))
        if not ok:
            result['differences'].append(label)
        save()
        print(label, bool(ok), flush=True)
    all_unchanged = [z for rows in unchanged.values() for z in rows]
    compare('moved409__unchanged452_neutral', n.c.material(moved, all_unchanged), raw['native']['moved409__unchanged452_neutral'])
    compare('new11__retained861_neutral', n.c.material(saved, moved+all_unchanged), raw['native']['new11__retained861_neutral'])
    for i, row in enumerate(saved[:-1]):
        label = 'new_internal_'+str(i)
        want = raw['native'][label]
        if i == 0 or want['overlaps'] or want['operation_failures']:
            compare(label, n.c.material([row], saved[i+1:]), want)
    arm = n.p.read(ROOT/'cad/modular_roots_v9/design.json')['arm']
    for state in raw['motion']:
        T = d.transforms(state['angles_deg'], arm)
        assert max(np.max(abs(T[key]-state['transforms'][key])) for key in T) < 1e-10
        left = [dict(z, shape=n.g.original.transform(z['shape'], T['forearm'])) for z in moved+saved]
        for target in state['compared_frames']:
            label = state['id']+'__changed420__'+target
            right = [dict(z, shape=n.g.original.transform(z['shape'], T[target])) for z in unchanged[target]]
            compare(label, n.c.material(left, right), raw['native'][label])
    grouped = {z['module']: [] for z in contract}
    for z in moved:
        grouped[z['proposal_module']].append(z)
    neighbors = {frozenset([row[0]+'_actuator', row[1]+'_actuator']) for row in p.prior.ORDER}
    selected = [z for z in raw['inter_module_groups'] if z['native']['overlaps'] or z['native']['operation_failures']
                or frozenset([z['left'], z['right']]) in neighbors]
    for z in selected:
        compare('modules__'+z['left']+'__'+z['right'], n.c.material(grouped[z['left']], grouped[z['right']]), z['native'])
    beam = next(z['shape'] for z in unchanged['forearm'] if z['name'] == 'Forearm_box_beam')
    lookup = {z['name']: z for z in saved}
    for want in raw['contact_distances']:
        query = BRepExtrema_DistShapeShape(lookup[want['name']]['shape'].wrapped, beam.wrapped)
        query.SetMultiThread(False)
        query.Perform()
        assert query.IsDone()
        err = abs(query.Value()-want['actual_distance_to_beam_mm'])
        ok = err < 1e-5
        result['contacts'].append(dict(name=want['name'], difference_mm=err, matched=ok))
        if not ok:
            result['differences'].append('contact_'+want['name'])
    T = np.array(next(z for z in contract if z['module'] == 'thumb_ip_actuator')['transform_from_current'])
    old_ports = n.p.read(ROOT/'cad/compact_turnback_pack_v1/reviews/check.json')['design']
    for want in evidence['transformed_real_ports']:
        original = next(z for z in old_ports if z['id'] == 'motor_'+str(want['line']))
        point = (T@np.r_[original['free_port_world_mm'], 1.])[:3]
        tangent = T[:3, :3]@np.array(original['free_tangent_toward_hand'])
        err = max(np.max(abs(point-want['actual_transformed_motor_free_port_mm'])),
                  np.max(abs(tangent-want['actual_transformed_free_tangent'])))
        ok = bool(err < 1e-10)
        result['ports'].append(dict(line=want['line'], maximum_difference=float(err), matched=ok))
        if not ok:
            result['differences'].append('port_'+str(want['line']))
    result.update(completed=True, initial_core_passed=not result['differences'],
        native_groups=len(result['native_samples']), native_matches=sum(z['matched'] for z in result['native_samples']),
        geometry_matches=sum(z['matched'] for z in result['geometry_comparisons']),
        placement_matches=sum(z['matched'] for z in result['placements']),
        contact_matches=sum(z['matched'] for z in result['contacts']),
        port_matches=sum(z['matched'] for z in result['ports']),
        input_hash_guard=guard(), checkout_clean=not git('status', '--porcelain=v1'),
        replay_scope='Two neutral groups,carrier internal and any other positive/failure new-internal groups,all6changed-body pose groups,allpositive/failure intermodule groups and first pair each row.11sourceSTEPmetrics/16placements409names/3contacts/2realports. Otheremptygroups,renderpixels,alltrimBoolean,wholematerials/physical/drive/exterior not replayed or approved.',
        ARM051_ARM052_LINK050_old_checks_reexecuted=False)
    save()


if __name__ == '__main__':
    main()
