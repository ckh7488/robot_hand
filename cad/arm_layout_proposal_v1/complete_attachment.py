"""Measure the unselected second cap, preserving the original named queries."""
import sys
from pathlib import Path

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))
import proposal as p
from OCP.BRepExtrema import BRepExtrema_DistShapeShape


def main():
    out = p.HERE/'reviews/attachment.json'
    assert not out.exists()
    raw = p.n.p.read(p.HERE/'reviews/proposal.json')
    assert raw['completed']
    source = {'cad/arm_layout_proposal_v1/complete_attachment.py': p.n.m.drive.sha(Path(__file__)),
              'cad/arm_layout_proposal_v1/reviews/proposal.json': p.n.m.drive.sha(p.HERE/'reviews/proposal.json')}
    scene = p.n.p.read(p.ROOT/raw['scene'])
    row = next(z for z in scene['parts'] if z['name'] == 'arm052_beam_cap_1')
    loaded = p.n.c.load([row], source)[0]
    beamfile = 'cad/modular_roots_v9/parts/Forearm_box_beam.step'
    source[beamfile] = p.n.m.drive.sha(p.ROOT/beamfile)
    beam = p.cq.importers.importStep(str(p.ROOT/beamfile)).val()
    query = BRepExtrema_DistShapeShape(loaded['shape'].wrapped, beam.wrapped)
    query.SetMultiThread(False)
    query.Perform()
    assert query.IsDone()
    kept = [z for z in raw['attachment'] if z['name'] in ['arm052_short_wide_carrier', 'arm052_beam_cap_0']]
    assert len(kept) == 2
    additional = dict(name=row['name'], actual_native_distance_to_beam_mm=query.Value(),
        positive_volume_test='new11__all_retained861_neutral', normal_contact_not_axial_friction_approval=True)
    p.n.a.save(out, dict(completed=True, source_sha256=source,
        actual_contact_components=kept+[additional], inherited_distance_results=2, new_distance_queries=1,
        initial_query_scope_note='Initial new[:3] selected carrier, first cap, then a bolt. Named bolt query remains preserved in proposal.json and is not called collar contact. Only the missing second cap is measured here.',
        no_native_positive_volume_tests_reexecuted=True, no_installed_candidate_replacement=True,
        assembly_preload_friction_or_strength_approved=False,
        input_hash_guard=all(p.n.m.drive.sha(p.ROOT/f) == h for f, h in source.items())))
    print('contact distances', [z['actual_native_distance_to_beam_mm'] for z in kept+[additional]], flush=True)


if __name__ == '__main__':
    main()
