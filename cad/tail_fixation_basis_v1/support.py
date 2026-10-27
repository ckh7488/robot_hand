"""Check actual tail support along the existing 5 mm child tail, without edits."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import basis as b


def specifications(case):
    result=[]
    for original in b.specifications(case)[2:]:
        for fraction in [.2,.8]:
            result.append(dict(original,id=original['id']+'_s'+str(fraction),
                center=b.np.array(original['edge'].positionAt(fraction).toTuple()),
                expected_gaps_mm=[0.,None],fraction_of_existing_tail=fraction))
    return result


def main():
    out=b.HERE/'reviews/support.json';assert not out.exists()
    case,source,rows=b.context();lookup={z['name']:z for z in rows};checks=[];probes=[]
    for spec in specifications(case):
        got,probe=b.contact(spec,lookup);checks.append(got);probes.append(probe)
        print(got['id'],[z['distance_mm'] for z in got['surfaces']],flush=True)
    source['cad/tail_fixation_basis_v1/support.py']=b.m.drive.sha(Path(__file__))
    # This boundary refers only to the known untrimmed torus and the flat-floor
    # point at x=track, y=7.4; it is not a full trimmed-surface contact solution.
    z=(8.1**2-7.4**2)**.5
    b.a.save(out,dict(completed=True,source_sha256=source,contacts=checks,
        scene=b.export(probes,'additional_support_material'),
        original_midpoint_reused_from='cad/tail_fixation_basis_v1/reviews/check.json',
        new_contacts=4,new_native_distance_queries=8,midpoint_native_replays=0,
        untrimmed_torus_outer_radius_mm=8.1,nominal_flat_floor_y_mm=7.4,
        torus_outer_intersection_at_flat_floor_tangent_mm=z,
        boundary_scope='Reference torus/flat-floor point intersection; not proof of every trimmed surface or contact stress.',
        actual_preload_friction_retention_verified=False,hardware_or_material_geometry_changed=False,
        input_hash_guard=all(b.m.drive.sha(b.ROOT/f)==h for f,h in source.items())))


if __name__=='__main__':main()
