"""Validate reports without turning preserved mechanical failures into passes."""
import argparse, json, math
from pathlib import Path


def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--audit',type=Path,required=True)
    ap.add_argument('--geometry',type=Path,required=True);ap.add_argument('--reference',type=Path)
    a=ap.parse_args();audit=read(a.audit);geo=read(a.geometry)
    assert audit['completed'] and audit['input_hash_guard']['passed']
    assert geo['completed'] and geo['source_inputs_unchanged']
    assert not audit['whole_hand_motion_approved'] and not audit['manufacturing_release']
    assert audit['part_count']==geo['part_count']==364 and audit['unchanged_parts']==363
    assert audit['summary']['new_palm_collision_observations']==0
    # Existing Boolean failure stays visible; no new calculation failure may be
    # hidden among the reused observations or new whole-hand endpoints.
    expected=read(Path(__file__).resolve().parents[1]/'whole_hand_motion_v1/reviews/latest_hand.json')
    old={r['id']:r for r in expected['cases']}
    for c in audit['cases']:
        source=old[c['id']]
        inherited=[e for e in source['operation_failures'] if 'Palm_tapered_chassis' not in [e['a'],e['b']]]
        assert c['operation_failures']==inherited, 'New/unreconciled calculation failure'
        assert not c['new_reportable_pairs']
    assert all(not c['result']['operation_failures'] for c in audit['extra_extension_endpoints'])
    assert len(audit['local_path_bounds'])==4
    assert all(p['local_goal_met'] and p['continuous_nominal_gap_lower_bound_mm']>=.5 for p in audit['local_path_bounds'])
    assert not any(audit['geometry']['protected_removed_mm3'].values())
    assert geo['assembly']['valid_solids']==364 and geo['assembly']['sorted_volume_max_difference_mm3']<.001
    assert not any(geo['palm_mesh_topology'][k] for k in ['boundary_edges','nonmanifold_edges','degenerate_triangles'])
    assert all(r['original_overlap_mm3']>.02 and r['candidate_overlap_mm3']<1e-4 and r['candidate_gap_mm']>=.5 for r in geo['before_after'])
    if a.reference:
        ref=read(a.reference);by_id={r['id']:r for r in ref['cases']}
        def signature(c):return {tuple(sorted([r['a'],r['b']])):r['material_intersection_mm3'] for r in c['changed_palm_collisions']+c['inherited_invariant_collisions']}
        for c in audit['cases']:
            assert c['id'] in by_id
            original=signature(by_id[c['id']]);current=signature(c)
            assert original.keys()==current.keys(), 'Replay collision pairs changed'
            assert all(abs(original[k]-current[k])<1e-5 for k in original), 'Replay volume changed'
        for current,original in zip(audit['local_path_bounds'],ref['local_path_bounds']):
            assert current['finger']==original['finger']
            assert abs(current['continuous_nominal_gap_lower_bound_mm']-original['continuous_nominal_gap_lower_bound_mm'])<1e-5
    print(json.dumps(dict(checkpoint_reports_consistent=True,local_palm_relief_goal_met=True,
        baseline_failures_preserved=True,whole_hand_motion_approved=False,manufacturing_release=False),indent=2))


if __name__=='__main__':main()
