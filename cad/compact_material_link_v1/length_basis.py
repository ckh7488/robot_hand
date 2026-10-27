"""Necessary chord bounds for required movements before any length choice."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import compact_model as a
n,np,ROOT,HERE=a.n,a.np,a.ROOT,a.HERE


def main():
    out=HERE/'reviews/length_basis.json';assert not out.exists()
    case,x,weights,budgets,seed=a.neutral_seed();cases,target=a.motion_cases();bounds=[]
    for entry in cases:
        lines=[]
        for i in [0,1]:
            p=a.ports(entry['case'],i);up=float(np.linalg.norm(p['up_B']-p['up_A']));down=float(np.linalg.norm(p['down_B']-p['down_A']))
            lines.append(dict(line=i,up_chord_mm=up,down_chord_mm=down,necessary_free_contour_lower_bound_mm=up+down))
        bounds.append(dict(**entry,lines=lines))
    maxima=[max(row['lines'][i]['necessary_free_contour_lower_bound_mm'] for row in bounds) for i in [0,1]]
    result=dict(completed=True,neutral_case=case,neutral_parameters=x,arc_length_weights=weights,neutral_seed_free_contours_mm=budgets,
        neutral_seed_geometry=seed,required_movement_chord_bounds=bounds,maximum_required_chord_bound_mm=maxima,target_parent_angles_deg=target,
        neutral_seed_contour_excess_over_maximum_chord_mm=budgets-maxima,
        necessary_only=True,independent_moving_pose_curve_solutions=0,native_or_CAD_exports=0,
        full_constant_material_motion_or_force_or_supplier_lengths_approved=False,
        source_sha256={f:n.m.drive.sha(ROOT/f) for f in ['cad/compact_material_link_v1/compact_model.py','cad/compact_material_link_v1/length_basis.py',
            'cad/compact_turnback_pack_v1/reviews/check.json','cad/coupled_material_continuation_v1/reviews/analysis.json','cad/biarc_supported_link_v1/reviews/analysis.json']})
    n.a.save(out,result);print('neutral free',budgets,'max required chord',maxima,'excess',budgets-maxima,flush=True)


if __name__=='__main__':main()
