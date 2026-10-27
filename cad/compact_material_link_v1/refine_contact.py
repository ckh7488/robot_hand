"""One native-evidence-driven refinement of the actually blocked arc.

It adds collision sampling only on the recorded motor-blocking housing arc.
No hardware, radius, material budget or hand target is changed.
"""
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import run_link as check
a,n,k,np,ROOT,HERE=check.a,check.n,check.k,check.np,check.ROOT,check.HERE
from OCP.OSD import OSD_ThreadPool


def install_witnesses(witness):
    original=n.samples
    def sampled(x,case,weights,budgets):
        points,owners=original(x,case,weights,budgets);arc=n.curves(x,case,weights,budgets)[witness['line']][witness['side']][witness['arc_index']]
        P=np.array(arc['start']);T=np.array(arc['tangent_start']);theta=arc['angle_rad'];ell=arc['length_mm']
        normal=np.array(arc['mid'])-P-ell*T*(np.sin(theta/2)/theta if theta>1e-9 else .5);normal/=max(np.linalg.norm(normal),1e-20)
        extra=[]
        for f in witness['extra_fractions']:
            v=theta*f;extra.append(P+ell*(T*(np.sin(v)/theta if theta>1e-9 else f)+normal*((1-np.cos(v))/theta if theta>1e-9 else theta*f*f/2)))
            owners.append((witness['line'],witness['side'],witness['arc_index'],f))
        return np.vstack([points,extra]),owners
    n.samples=sampled


def main():
    out=HERE/'reviews/refined.json';assert not out.exists();start=time.monotonic()
    raw=n.p.read(HERE/'reviews/check.json');assert raw['completed'] and raw['stop_reason']=='neutral_native_block' and len(raw['states'])==1
    initial=raw['states'][0];pair=next(z['pair'] for z in initial['new_blocking_records'] if z['pair']['lead']=='compact_housing_0_5')
    assert pair['part']=='middle_dip_actuator_motor'
    weights=[[np.array(z) for z in row] for row in raw['arc_length_weights']];budgets=np.array(raw['free_contour_budgets_mm'])
    x=np.array(initial['parameters']);x0=np.array(raw['neutral_parameters']);ctx,hw=a.context();source=dict(ctx[0][0]);groups=a.pose(ctx,hw,initial['case'])
    motor=next(z for _,rows in groups for z in rows if z['name']==pair['part'])
    edge=a.paths(x,initial['case'],weights,budgets,0,'housing')[5];fractions=np.linspace(0,1,257)
    field=n.NativeField([motor]);points=np.array([edge.positionAt(float(f)).toTuple() for f in fractions]);distances=field.evaluate(points)
    assert distances;deepest=min(distances,key=lambda z:z[2]);minimum_fraction=float(fractions[deepest[0]])
    extra=sorted({j/16 for j in range(1,16) if j%4}|{minimum_fraction})
    witness=dict(line=0,side='up',arc_index=2,material_row=pair['lead'],obstacle=pair['part'],
        local_native_point_shell_probe_points=257,actual_probe_queries=field.calls,minimum_signed_center_distance_mm=deepest[2],
        minimum_fraction=minimum_fraction,extra_fractions=extra,
        fractions_are_numerical_contact_samples_not_supports_or_fixed_material_points=True)
    install_witnesses(witness)
    for rel in ['cad/compact_material_link_v1/refine_contact.py','cad/compact_material_link_v1/reviews/check.json']:
        source[rel]=n.m.drive.sha(ROOT/rel)
    report=dict(completed=False,source_sha256=source,seed_report='cad/compact_material_link_v1/reviews/check.json',
        original_blocking_records=initial['new_blocking_records'],contact_witness=witness,
        neutral_parameters=x0,arc_length_weights=weights,free_contour_budgets_mm=budgets,length_choice=raw['length_choice'],
        increments=[],states=[],profiles=[],stop_reason=None,original_native_groups_reexecuted=0,
        actual_original_failed_material_candidate_preserved=True,hardware_or_material_budgets_changed=False,
        planned_new_parent_increments=[5,10,15,20,25],unexecuted_original_full_motion_plan_is_not_completed=True,
        native_material_states_do_not_prove_continuous_motion=True,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads())
    def save():n.a.save(out,report)
    save()
    entries=[dict(id='neutral_contact_refined',phase='refine',case=initial['case'])]
    target=np.array(raw['target_parent_angles_deg'])
    entries += [dict(id='new_parent_'+str(p),phase='approach',case=n.case_at(target*p/100.)) for p in [5,10,15,20,25]]
    for entry in entries:
        groups=a.pose(ctx,hw,entry['case']);print('begin',entry['id'],flush=True)
        fresh,rec=n.advance(x,entry['case'],weights,budgets,[z for _,rows in groups for z in rows])
        rec.update(id=entry['id'],phase=entry['phase']);report['increments'].append(rec);save();print('solve',entry['id'],rec['solved'],flush=True)
        if not rec['solved']:report['stop_reason']='one_refined_projection_failed_'+entry['id'];break
        profiles=[check.profile(fresh,entry['case'],x0,weights,budgets,i,kind) for kind in ['housing','cord'] for i in [0,1]]
        report['profiles'].append(dict(id=entry['id'],materials=profiles));save()
        # Every new parent increment receives an actual thickness check here.
        rows=a.build(fresh,entry['case'],weights,budgets);state=dict(**entry,parameters=fresh,rows=len(rows),native={});report['states'].append(state);save()
        for key,obs in groups:
            state['native'][key]=n.c.material(rows,obs);save();print('native',entry['id'],key,len(state['native'][key]['overlaps']),flush=True)
        state['native']['self']=k.flat([n.c.material([z],rows[j+1:]) for j,z in enumerate(rows)])
        check.classify(state,rows);state['scene']=a.export(rows,entry['id']);save()
        if not state['conditional_geometric_clear']:report['stop_reason']='new_native_block_'+entry['id'];break
        x=fresh
    tests=[test for state in report['states'] for test in state['native'].values()];agg=k.flat(tests)
    report.update(completed=True,refined_neutral_and_five_parent_increments_completed=report['stop_reason'] is None and len(report['states'])==6,
        increments_attempted=len(report['increments']),increments_solved=sum(z['solved'] for z in report['increments']),native_material_states=len(report['states']),
        pairs_in_scope=agg['pairs_in_scope'],AABB_native_pairs=agg['exact_pairs'],raw_overlap_records=len(agg['overlaps']),operation_failures=len(agg['operation_failures']),
        input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()),elapsed_s=time.monotonic()-start)
    save();print('finished',report['stop_reason'],report['native_material_states'],flush=True)


if __name__=='__main__':main()
