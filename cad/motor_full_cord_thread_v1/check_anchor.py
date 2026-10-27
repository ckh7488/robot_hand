"""Anchor-bore-first alternative; preserve raw same-cord junction overlaps."""
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import anchor_model as n
import check_cord as first
t,w,l,a,m,g,c,p,np,cq,ROOT,HERE=n.t,n.w,n.l,n.a,n.m,n.g,n.c,n.p,n.np,n.cq,n.ROOT,n.HERE
from OCP.OSD import OSD_ThreadPool


def main():
    out=HERE/'reviews/anchor_check.json';assert not out.exists();case=t.neutral();data,groups=t.prior.inputs(case)
    form=p.read(ROOT/'cad/progressive_tail_assembly_v1/reviews/restoration.json');assert form['conditional_CAD_sequence_clear']
    housing=c.load(p.read(ROOT/form['final']['scene'])['parts'],data[0]);retained=[];start=time.monotonic()
    report=dict(completed=False,source_sha256=data[0],cases=[],plans=[],all_hardware_present={k:[z['name'] for z in zz] for k,zz in groups},omitted_hardware=[],housing_scene=form['final']['scene'],housing_names=[z['name'] for z in housing],
                start_condition='All862 current hardware and two FORM042 seated whole housings present, cords absent. Lead tip enters real slanted motor anchor bore from declared external tail exit, then open R10 groove/free18mm/R5/adjuster/fullhousing/distalroute. No anchor-tail side rotation.',
                manual_holding_of_incoming_tail_and_open_wraps=True,nominal_anchor_screw_positions_unchanged=True,
                actual_clamp_screw_looseness_compression_thread_torque_force_verified=False,actual_tail_fixation_verified=False,
                EI_gravity_torsion_friction_tension_preload_or_physical_motion_solved=False,new_hardware_rows=0,
                reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads())
    for i in [0,1]:
        path,labels,length=n.specification(case,i);report['plans'].append(dict(line=i,tips_mm=n.samples(case,i),labels=labels,material_length_mm=length,
            anchor_bore_entry_mm=path[0].startPoint().toTuple(),anchor_bore_exit_at_wrap_mm=path[0].endPoint().toTuple(),incoming_tangent=path[0].tangentAt(0).toTuple()))
    for rel in ['cad/motor_full_cord_thread_v1/anchor_model.py','cad/motor_full_cord_thread_v1/check_anchor.py','cad/motor_full_cord_thread_v1/cord_model.py','cad/motor_full_cord_thread_v1/check_cord.py','cad/motor_full_cord_thread_v1/reviews/check.json','cad/biarc_supported_link_v1/whole_cord.py','cad/biarc_supported_link_v1/link_model.py','cad/biarc_supported_link_v1/reviews/check.json','cad/motor_tendon_terminal_v1/terminal_selected.py','cad/progressive_tail_assembly_v1/reviews/restoration.json',form['final']['scene']]:report['source_sha256'][rel]=m.drive.sha(ROOT/rel)
    def save():a.save(out,report)
    ok=True;save()
    for plan in report['plans']:
        i=plan['line'];tips=plan['tips_mm']
        for j,tip in enumerate(tips):
            rows,info=n.build(case,i,tip);tests={key:c.material(rows,zz) for key,zz in groups};tests['whole_housings']=c.material(rows,housing);tests['earlier_cord']=c.material(rows,retained);tests['self']=first.aggregate([c.material([z],rows[q+1:]) for q,z in enumerate(rows)])
            raw_clear=all(m.drive.clear(z) for z in tests.values());known=[];blocking=[];lookup={z['name']:z for z in rows}
            for key,zz in tests.items():
                for pair in zz['overlaps']:
                    left,right=lookup.get(pair['lead']),lookup.get(pair['part'])
                    adjacent=key=='self' and left and right and {left['path_segment'],right['path_segment']}=={0,1}
                    if adjacent:
                        gap=float(np.linalg.norm(np.asarray(left['endpoint_end_mm'])-right['endpoint_start_mm']))
                        assert gap<1e-5
                        known.append(dict(group=key,pair=pair,endpoint_gap_mm=gap,physical_line=i,classification='Adjacent motor anchor-tail/wrap pieces of same physical cord. Raw overlap retained; no smooth bend, local elastic equilibrium or fixation approval.'))
                    else:blocking.append(dict(group=key,pair=pair))
            ok=not blocking and not any(z['operation_failures'] for z in tests.values())
            record=dict(id='anchor_'+str(i)+'_'+str(j),**info,native=tests,raw_native_clear=raw_clear,known_adjacent_motor_joint_records=known,
                        blocking_overlap_records=blocking,bounded_sequence_can_continue=ok,retained_cord_names=[z['name'] for z in retained],rows=len(rows))
            report['cases'].append(record);save()
            if j in [0,1,len(tips)-1] or not ok:record['scene']=first.export(rows,record['id']);save()
            print(record['id'],raw_clear,'blocking',len(blocking),'adjacent',len(known),flush=True)
            if not ok:break
        if not ok:break
        retained+=rows
    agg=first.aggregate([z for rec in report['cases'] for z in rec['native'].values()])
    report.update(completed=True,bounded_sequence_clear_of_other_overlaps=ok,raw_native_all_clear=all(z['raw_native_clear'] for z in report['cases']),
        pairs_in_scope=agg['pairs_in_scope'],AABB_native_pairs=agg['exact_pairs'],overlap_records=len(agg['overlaps']),operation_failures=len(agg['operation_failures']),
        actual_whole_cords_threaded=False,physical_motor_or_distal_tails_fixed=False,whole_hand_drive_approved=False,elapsed_s=time.monotonic()-start,
        input_hash_guard=all(m.drive.sha(ROOT/f)==h for f,h in report['source_sha256'].items()));save()


if __name__=='__main__':main()
