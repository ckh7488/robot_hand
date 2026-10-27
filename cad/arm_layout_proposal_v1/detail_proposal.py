"""Locate the new carrier conflict and compare only its five old rack pairs."""
import sys
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))
import proposal as p

n, np, ROOT, HERE = p.n, p.np, p.ROOT, p.HERE
v = n.m.drive.module('_arm052_detail_metadata', ROOT/'cad/drive_packaging_review_v1/inspect_render.py')


def main():
    out = HERE/'reviews/conflict_detail.json'
    assert not out.exists()
    raw = n.p.read(HERE/'reviews/proposal.json')
    source = {'cad/arm_layout_proposal_v1/detail_proposal.py': n.m.drive.sha(Path(__file__)),
              'cad/arm_layout_proposal_v1/reviews/proposal.json': n.m.drive.sha(HERE/'reviews/proposal.json')}
    records = raw['native']['new11__all_retained861_neutral']['overlaps']
    assert len(records) == 5 and all(z['lead'] == 'arm052_short_wide_carrier' for z in records)
    _, metadata, _ = v.metadata()
    names = {z['part'] for z in records} | {raw['omitted_old_rack_only']}
    old = n.c.load([z for z in metadata if z['name'] in names], source)
    assert len(old) == 6
    rack = next(z for z in old if z['name'] == raw['omitted_old_rack_only'])
    others = [z for z in old if z['name'] != rack['name']]
    baseline = n.c.material([rack], others)
    carrier = n.c.load([z for z in n.p.read(ROOT/raw['scene'])['parts']
                        if z['name'] == 'arm052_short_wide_carrier'], source)[0]
    worst = max(records, key=lambda z: z['material_intersection_mm3'])
    original = next(z for z in others if z['name'] == worst['part'])
    placement = next(z for z in raw['placements'] if original['name'] in z['rows'])
    moved = dict(original, shape=n.g.original.transform(original['shape'], np.array(placement['transform_from_current'])))
    common = carrier['shape'].intersect(moved['shape'], tol=1e-6)
    scene = n.a.a.prior.export([dict(name='arm052_carrier_guide_actual_common', shape=common,
        body='forearm', color=[.97, .04, .03], evidence_only=True,
        installed_part_candidate=False)], HERE/'generated/actual_common')
    n.a.save(out, dict(completed=True, source_sha256=source, new_carrier_record=worst,
        actual_common_scene=scene, actual_common_bbox_mm=n.c.m.pack._bbox(common),
        actual_common_volume_mm3=common.Volume(), moved_part=original['name'],
        moved_part_transform=placement['transform_from_current'],
        old_rack_only_five_corresponding_pairs=baseline,
        original_ARM051_groups_or_all_old_internal_pairs_reexecuted=False,
        baseline_scope='Only old rack against the five exact original parts involved in the new carrier report. Not every old contact or prior approval.',
        input_hash_guard=all(n.m.drive.sha(ROOT/f) == h for f, h in source.items())))
    print('old five-pair records', baseline['overlaps'], flush=True)
    print('new actual common', common.Volume(), n.c.m.pack._bbox(common), flush=True)


if __name__ == '__main__':
    main()
