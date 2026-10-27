"""Exact-SHA replay of a bounded separate layout proposal, including failures."""
import argparse
import subprocess
import sys
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))
import proposal as p
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.OSD import OSD_ThreadPool

n, np, ROOT, HERE, d = p.n, p.np, p.ROOT, p.HERE, p.d
measure = n.m.drive.module('_arm052_geometry_metrics', ROOT/'cad/long_housing_route_v1/review_core.py')


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
    manifest = n.p.read(ROOT/'checkpoints/ARM052.manifest.json')
    frozen = {}
    for key in ['geometry_sha256', 'code_sha256', 'report_sha256']:
        frozen.update(n.p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[key])
    def guard():
        return all(n.m.drive.sha(ROOT/f) == z['sha256'] for f, z in manifest['files'].items()) and all(
            n.m.drive.sha(ROOT/f) == h for f, h in frozen.items())
    assert guard() and OSD_ThreadPool.DefaultPool_s().NbThreads() == 1
    raw = n.p.read(HERE/'reviews/proposal.json')
    attachment = n.p.read(HERE/'reviews/attachment.json')
    detail = n.p.read(HERE/'reviews/conflict_detail.json')
    frames, source = d.inputs()
    _, modules, grouped, rack = p.s.bundles(frames)
    moved, contract = p.placements(modules, grouped)
    fresh = p.carrier()
    saved = n.c.load(n.p.read(ROOT/raw['scene'])['parts'])
    names = {z['name'] for z in moved} | {rack['name']}
    unchanged = {key: [z for z in rows if z['name'] not in names] for key, rows in frames.items()}
    result = dict(completed=False, target_commit=args.commit, differences=[], native_samples=[],
        geometry_comparisons=[], placements=[], attachment=[], manifest_files=len(manifest['files']),
        frozen_files=len(frozen), OCC_pool_threads=1, same_agent_not_independent_audit=True,
        current_installed_candidate_replaced=False, whole_drive_or_exterior_approved=False)
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
    assert len(contract) == len(raw['placements']) == 16
    for got, want in zip(contract, raw['placements']):
        error = max(float(np.max(abs(np.array(got[k])-want[k]))) for k in
                    ['transform_from_current', 'proposed_installation_transform', 'proposed_center_mm'])
        ok = got['module'] == want['module'] and got['rows'] == want['rows'] and error < 1e-10
        result['placements'].append(dict(module=got['module'], rows=len(got['rows']), error=error, matched=bool(ok)))
        if not ok:
            result['differences'].append('placement_'+got['module'])
    all_unchanged = [z for rows in unchanged.values() for z in rows]
    def compare(label, got, want):
        a, b = d.overlap_map(got), d.overlap_map(want)
        error = max([abs(a[k]['material_intersection_mm3']-b[k]['material_intersection_mm3'])
                     for k in a.keys() & b.keys()]+[0.])
        ok = a.keys() == b.keys() and error < 1e-5 and got['operation_failures'] == want['operation_failures'] and got['pairs_in_scope'] == want['pairs_in_scope']
        result['native_samples'].append(dict(id=label, native=got, volume_difference_mm3=error, matched=bool(ok)))
        if not ok:
            result['differences'].append(label)
        save()
        print(label, bool(ok), flush=True)
    for label, left, right in [
        ('moved409__unchanged452_neutral', moved, all_unchanged),
        ('new11__all_retained861_neutral', saved, moved+all_unchanged),
        ('new_internal_0', saved[:1], saved[1:])]:
        compare(label, n.c.material(left, right), raw['native'][label])
    arm = n.p.read(ROOT/'cad/modular_roots_v9/design.json')['arm']
    for state in raw['motion']:
        T = d.transforms(state['angles_deg'], arm)
        assert max(np.max(abs(T[key]-np.array(state['transforms'][key]))) for key in T) < 1e-10
        left = [dict(z, shape=n.g.original.transform(z['shape'], T['forearm'])) for z in moved+saved]
        for frame in state['compared_frames']:
            label = state['id']+'__changed420__'+frame
            right = [dict(z, shape=n.g.original.transform(z['shape'], T[frame])) for z in unchanged[frame]]
            compare(label, n.c.material(left, right), raw['native'][label])
    grouped_moved = {z['module']: [] for z in contract}
    for z in moved:
        grouped_moved[z['proposal_module']].append(z)
    row_neighbors = {(p.ORDER[j][0]+'_actuator', p.ORDER[j][1]+'_actuator') for j in range(4)}
    selected = [z for z in raw['inter_module_groups'] if z['native']['overlaps'] or z['native']['operation_failures']
                or (z['left'], z['right']) in row_neighbors or (z['right'], z['left']) in row_neighbors]
    result['inter_module_selection'] = 'All source-positive/failure groups plus first two modules in each of four rows, deduplicated. Other empty groups not rerun.'
    for test in selected:
        left, right = test['left'], test['right']
        compare('modules__'+left+'__'+right, n.c.material(grouped_moved[left], grouped_moved[right]), test['native'])
    target_names = {z['part'] for z in raw['native']['new11__all_retained861_neutral']['overlaps']}
    original_parts = [z for rows in frames.values() for z in rows if z['name'] in target_names]
    compare('old_rack_only_five_corresponding_pairs', n.c.material([rack], original_parts),
            detail['old_rack_only_five_corresponding_pairs'])
    beam = next(z['shape'] for z in unchanged['forearm'] if z['name'] == 'Forearm_box_beam')
    lookup = {z['name']: z for z in saved}
    for expected in attachment['actual_contact_components']:
        shape = lookup[expected['name']]['shape']
        query = BRepExtrema_DistShapeShape(shape.wrapped, beam.wrapped)
        query.SetMultiThread(False)
        query.Perform()
        assert query.IsDone()
        error = abs(query.Value()-expected['actual_native_distance_to_beam_mm'])
        ok = error < 1e-5
        result['attachment'].append(dict(name=expected['name'], distance_mm=query.Value(), difference_mm=error, matched=ok))
        if not ok:
            result['differences'].append('contact_'+expected['name'])
    result.update(completed=True, initial_core_passed=not result['differences'],
        native_groups=len(result['native_samples']), native_matches=sum(z['matched'] for z in result['native_samples']),
        geometry_matches=sum(z['matched'] for z in result['geometry_comparisons']),
        placement_matches=sum(z['matched'] for z in result['placements']),
        attachment_matches=sum(z['matched'] for z in result['attachment']),
        input_hash_guard=guard(), checkout_clean=not git('status', '--porcelain=v1'),
        all138_native_groups_or_rendering_or_continuous_motion_replayed=False,
        all_surface_equations_trim_or_Boolean_equivalence_proved=False,
        original_ARM051_or_LINK050_reexecuted=False)
    save()


if __name__ == '__main__':
    main()
