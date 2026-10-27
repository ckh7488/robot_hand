"""Use native whole-arc distance for the actual between-sample motor block."""
import sys,time,math
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import refine_contact as refine
check,a,n,k,np,ROOT,HERE=refine.check,refine.a,refine.n,refine.k,refine.np,refine.ROOT,refine.HERE
from OCP.BRepExtrema import BRepExtrema_DistShapeShape


def arc_point(arc,f):
    A=np.array(arc['start']);T=np.array(arc['tangent_start']);theta=arc['angle_rad'];ell=arc['length_mm']
    normal=np.array(arc['mid'])-A-ell*T*(np.sin(theta/2)/theta if theta>1e-9 else .5);normal/=max(np.linalg.norm(normal),1e-20)
    return A+ell*(T*(np.sin(theta*f)/theta if theta>1e-9 else f)+normal*((1-np.cos(theta*f))/theta if theta>1e-9 else theta*f*f/2))


def install_continuous(motor,records):
    original=n.inequalities
    def augmented(x,case,weights,budgets,field,jacobian):
        values,rows,labels=original(x,case,weights,budgets,field,jacobian)
        arc=n.curves(x,case,weights,budgets)[0]['up'][2];edge=n.rod.edge(arc)
        tool=BRepExtrema_DistShapeShape(edge.wrapped,motor['shape'].wrapped);tool.SetMultiThread(False);tool.Perform()
        assert tool.IsDone() and tool.NbSolution()
        d=tool.Value();P=np.array(tool.PointOnShape1(1).Coord());Q=np.array(tool.PointOnShape2(1).Coord())
        A=np.array(arc['start']);T=np.array(arc['tangent_start']);theta=arc['angle_rad'];ell=arc['length_mm']
        normal=np.array(arc['mid'])-A-ell*T*(np.sin(theta/2)/theta if theta>1e-9 else .5);normal/=max(np.linalg.norm(normal),1e-20)
        if theta<1e-8:f=float(np.clip((P-A)@T/ell,0,1))
        else:
            R=ell/theta;phi=math.atan2(float((P-A)@T/R),float(1-(P-A)@normal/R))
            if phi<-1e-8:phi+=2*math.pi
            f=float(np.clip(phi/theta,0,1))
        reconstruction=float(np.linalg.norm(P-arc_point(arc,f)));assert reconstruction<1e-5,reconstruction
        records.append(dict(distance_mm=d,curve_fraction=f,point_on_arc_mm=P,point_on_motor_mm=Q,point_reconstruction_mm=reconstruction,jacobian_requested=jacobian))
        values=np.r_[values,(d-1.5)/100.];labels.append(('whole_arc_native_distance',0,'up',2,motor['name'],d,f))
        if jacobian:
            assert d>1e-9,'Whole-arc centerline intersects motor; no differentiable positive-distance contact direction.'
            JP=n.approx_derivative(lambda y:arc_point(n.curves(y,case,weights,budgets)[0]['up'][2],f),x)
            rows=np.vstack([rows,((P-Q)/d)@JP/100.])
        return values,rows,labels
    n.inequalities=augmented


def main():
    out=HERE/'reviews/continuous.json';assert not out.exists();start=time.monotonic()
    previous=n.p.read(HERE/'reviews/refined.json');raw=n.p.read(HERE/'reviews/check.json')
    assert previous['completed'] and previous['stop_reason']=='new_native_block_neutral_contact_refined'
    state=previous['states'][-1];x=np.array(state['parameters']);x0=np.array(previous['neutral_parameters'])
    weights=[[np.array(z) for z in line] for line in previous['arc_length_weights']];budgets=np.array(previous['free_contour_budgets_mm'])
    ctx,hw=a.context();source=dict(ctx[0][0]);groups=a.pose(ctx,hw,state['case']);motor=next(z for _,rows in groups for z in rows if z['name']=='middle_dip_actuator_motor')
    assert motor['name'] not in ctx[-1]
    refine.install_witnesses(previous['contact_witness']);distance_records=[];install_continuous(motor,distance_records)
    for f in ['continuous_contact.py','reviews/refined.json','reviews/check.json']:
        source[(HERE/f).relative_to(ROOT).as_posix()]=n.m.drive.sha(HERE/f)
    result=dict(completed=False,source_sha256=source,seed_report='cad/compact_material_link_v1/reviews/refined.json',
        original_initial_and_sample_refinement_failures_preserved=True,old_native_groups_reexecuted=0,hardware_or_material_budget_changed=False,
        neutral_parameters=x0,arc_length_weights=weights,free_contour_budgets_mm=budgets,length_choice=raw['length_choice'],
        continuous_contact_scope='Only recorded lower up arc2 vs middle_dip_actuator_motor uses native whole-edge distance. Other free curves retain sampled contacts; full material native Boolean still required.',
        continuous_native_distance_records=distance_records,increments=[],states=[],profiles=[],stop_reason=None,
        no_extra_mechanical_clearance_mm=True,contact_centerline_radius_mm=1.5,whole_hand_or_force_or_exterior_approved=False)
    def save():n.a.save(out,result)
    target=np.array(raw['target_parent_angles_deg']);entries=[dict(id='whole_arc_neutral',phase='native_arc_contact',case=state['case'])]
    entries += [dict(id='whole_arc_parent_'+str(p),phase='approach',case=n.case_at(target*p/100.)) for p in [5,10,15,20,25]]
    save()
    for entry in entries:
        groups=a.pose(ctx,hw,entry['case']);print('begin',entry['id'],flush=True)
        fresh,rec=n.advance(x,entry['case'],weights,budgets,[z for _,rows in groups for z in rows]);rec.update(id=entry['id'],phase=entry['phase'])
        result['increments'].append(rec);save();print('solve',entry['id'],rec['solved'],flush=True)
        if not rec['solved']:result['stop_reason']='whole_arc_projection_failed_'+entry['id'];break
        result['profiles'].append(dict(id=entry['id'],materials=[check.profile(fresh,entry['case'],x0,weights,budgets,i,kind) for kind in ['housing','cord'] for i in [0,1]]));save()
        rows=a.build(fresh,entry['case'],weights,budgets);got=dict(**entry,parameters=fresh,rows=len(rows),native={});result['states'].append(got);save()
        for key,obs in groups:
            got['native'][key]=n.c.material(rows,obs);save();print('native',entry['id'],key,len(got['native'][key]['overlaps']),flush=True)
        got['native']['self']=k.flat([n.c.material([z],rows[j+1:]) for j,z in enumerate(rows)])
        check.classify(got,rows);got['scene']=a.export(rows,entry['id']);save()
        if not got['conditional_geometric_clear']:result['stop_reason']='whole_arc_new_native_block_'+entry['id'];break
        x=fresh
    agg=k.flat([test for got in result['states'] for test in got['native'].values()])
    result.update(completed=True,neutral_and_five_new_parent_increments_completed=result['stop_reason'] is None and len(result['states'])==6,
        increments_attempted=len(result['increments']),increments_solved=sum(z['solved'] for z in result['increments']),native_material_states=len(result['states']),
        pairs_in_scope=agg['pairs_in_scope'],AABB_native_pairs=agg['exact_pairs'],raw_overlap_records=len(agg['overlaps']),operation_failures=len(agg['operation_failures']),
        input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()),elapsed_s=time.monotonic()-start)
    save();print('finished',result['stop_reason'],result['native_material_states'],flush=True)


if __name__=='__main__':main()
