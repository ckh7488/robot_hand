"""Analytic endpoint construction, without length closure or solver search."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import link_model as l
a,m,p,np,ROOT,HERE=l.a,l.m,l.p,l.np,l.ROOT,l.HERE

def main():
    out=HERE/'reviews/analysis.json';assert not out.exists();target=np.asarray(next(z for z in p.read(ROOT/'cad/thumb_guide_assembly_v2/reviews/static.json')['cases'] if z['id']=='parent_full')['angles_deg']);cases=[]
    for j,(f,flex,yaw) in enumerate([(0,0,0),(.25,0,0),(.5,0,0),(.75,0,0),(1,0,0),(0,-20,-15),(0,20,15)]):
        case=l.geometry(target*f,flex,yaw);case.update(id=j,parent_factor=f,local_thick_tube_condition=all(z['minimum_free_radius_mm']>1.5 for z in case['lines']));cases.append(case);print(j,[(z['housing_length_mm'],z['minimum_free_radius_mm']) for z in case['lines']],flush=True)
    report=dict(completed=True,cases=cases,source_sha256={f:m.drive.sha(ROOT/f) for f in ['cad/biarc_supported_link_v1/link_model.py','cad/biarc_supported_link_v1/analyze.py','cad/support_attachment_basis_v1/bend_model.py','cad/thumb_guide_assembly_v2/reviews/static.json']},new_length_targets_set=False,old_lengths_controls_or_solvers_reused=False,constant_material_length_or_motion_approved=False,new_support_at_arc_join=False,contact_or_elastic_equilibrium_solved=False)
    a.save(out,report)
if __name__=='__main__':main()
