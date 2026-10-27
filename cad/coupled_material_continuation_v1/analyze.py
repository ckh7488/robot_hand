"""Continue both installed contours from the saved neutral configuration."""
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import continuation as n
from OCP.OSD import OSD_ThreadPool


def main():
    out=n.HERE/'reviews/analysis.json';assert not out.exists();started=time.monotonic()
    base,x,weights,budgets=n.initial();e0=n.endpoints(x,base,weights,budgets)
    assert max(abs(e0))<1e-10,e0
    # Seed geometry is an exact subdivision of the installed neutral arcs,
    # not a rerun of the discarded independent rod/biarc pose families.
    proof=[]
    for i in [0,1]:
        for kind in ['housing','cord']:
            got=n.middle(x,base,weights,budgets,i,kind);want=n.l.middle_edges(base,i,kind)
            L=sum(z.Length() for z in got);L0=sum(z.Length() for z in want)
            samples=[float(s) for s in n.np.linspace(0,L0,73)]
            error=max(n.np.linalg.norm(n.np.array(n.t.point_at(got,s))-n.t.point_at(want,s)) for s in samples)
            assert abs(L-L0)<1e-5 and error<1e-5
            proof.append(dict(line=i,kind=kind,length_error_mm=L-L0,maximum_sample_error_mm=error,sample_count=len(samples)))
    data,pack,guide,wrist=n.a.s.prior.inputs();hardware=n.c.load([z for z in n.p.read(n.ROOT/'cad/support_attachment_basis_v1/generated/checked/scene.json')['parts'] if z['installed_part_candidate']],data[0])
    pack=[z for z in pack if z['name'] not in [v['replaced_part'] for v in hardware]]
    support=n.t.prior.support
    def obstacles(case):
        cc=dict(angles_deg=case['q'],solution=dict(wrist_P=case['P'],wrist_H=case['H'],parent_F=case['F']))
        groups=support.kobs(data,pack,guide,wrist,cc)
        return [(key,rows+hardware if key=='packaging' else rows) for key,rows in groups]
    target=n.np.array(n.p.read(n.ROOT/'cad/material_length_motion_v1/reviews/analysis.json')['cases'][4]['angles_deg'])
    result=dict(completed=False,source_sha256=data[0],initial_parameters=x,arc_length_weights=weights,free_contour_budgets_mm=budgets,
        neutral_subdivision_comparison=proof,steps=[],target_parent_angles_deg=target,actual_hardware_rows=862,
        method='Coupled minimum incremental-coordinate projection with actual native point/shell contact; exact per-line contour budget and free guide material allocation. No independent pose or bending equilibrium solve.',
        node_contact_samples_per_arc=4,point_sampling_not_continuous_contact_proof=True,
        tail_no_slip_conditional=True,EI_gravity_twist_friction_preload_tension_or_equilibrium_solved=False,
        OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),new_hardware=0)
    for rel in ['cad/coupled_material_continuation_v1/continuation.py','cad/coupled_material_continuation_v1/analyze.py',
                'cad/elastic_housing_link_v1/arc_rod.py','cad/whole_cord_sliding_v1/slide_model.py',
                'cad/material_length_motion_v1/reviews/analysis.json','cad/biarc_supported_link_v1/reviews/analysis.json']:
        result['source_sha256'][rel]=n.m.drive.sha(n.ROOT/rel)
    def save():n.a.save(out,result)
    save()
    # First natural approach interval; a failed coupled continuation is not
    # evidence that the hand or all fixed-length alternatives are impossible.
    for fraction in [.05,.10,.15,.20,.25]:
        case=n.case_at(target*fraction);groups=obstacles(case)
        allhw=[z for _,rows in groups for z in rows];assert len(allhw)==862
        x,rec=n.advance(x,case,weights,budgets,allhw);rec['parent_fraction']=fraction
        result['steps'].append(rec);save();print('parent',fraction,'solved',rec['solved'],'residual',rec['endpoint_scaled_residual'],'inequality',rec['minimum_inequality'],'native_queries',rec['native_point_shell_queries'],flush=True)
        if not rec['solved']:break
    result.update(completed=True,all_requested_increments_solved=len(result['steps'])==5 and all(z['solved'] for z in result['steps']),
        whole_parent_or_wrist_motion_approved=False,native_swept_material_checks_completed=0,
        input_hash_guard=all(n.m.drive.sha(n.ROOT/f)==h for f,h in data[0].items()),elapsed_s=time.monotonic()-started)
    save()


if __name__=='__main__':main()
