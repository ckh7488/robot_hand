"""Additional new-pose hardware pairs, without repeating material checks."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import run_motion as d
n,k,ROOT,HERE=d.n,d.k,d.ROOT,d.HERE
from OCP.OSD import OSD_ThreadPool


def sets(ctx,case):
    groups=k.pose(ctx,case);rotor=ctx[-1]
    child={z['name'] for z in n.p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')['hand'] if z['body']=='thumb_ip'}
    allhw=[z for _,rows in groups for z in rows];changed=[z for z in allhw if z['name'] in rotor|child];rest=[z for z in allhw if z['name'] not in rotor|child]
    assert len(rotor)==9 and len(child)==13 and len(changed)==22 and len(rest)==840
    return changed,rest,[z for z in changed if z['name'] in rotor],[z for z in changed if z['name'] in child]


def main(ctx=None):
    out=HERE/'reviews/moving_hardware.json';assert not out.exists();raw=n.p.read(HERE/'reviews/check.json');assert raw['completed'];ctx=k.context() if ctx is None else ctx
    baseline=n.p.read(ROOT/'cad/motor_spool_coupling_v1/reviews/assessment.json');assert baseline['stock_three_solids_geometry_unchanged'];volume=baseline['baseline_stock_motor_common_mm3']
    source=dict(ctx[0][0])
    for rel in ['cad/parent_wrist_material_motion_v1/moving_hardware.py','cad/parent_wrist_material_motion_v1/reviews/check.json','cad/motor_spool_coupling_v1/reviews/assessment.json']:
        source[rel]=n.m.drive.sha(ROOT/rel)
    report=dict(completed=False,source_sha256=source,states=[],baseline_stock_motor_common_mm3=volume,material_pairs_reexecuted=0,
        order='After material motion checks; later material states, if any, were already computed. A hardware stop does not erase them or approve their combined motion.',
        rotor_internal_and_child_internal_rigid_pairs_reexecuted=0,all_existing_hardware_pairs_reexecuted=False,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),
        reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6)
    def save():n.a.save(out,report)
    save()
    for rec in raw['states']:
        changed,rest,rotor,child=sets(ctx,rec['case']);state=dict(id=rec['id'],native={},changed_names=[z['name'] for z in changed],other_hardware_count=840)
        report['states'].append(state);save()
        state['native']['actuated22_vs_other840']=n.c.material(changed,rest);save()
        state['native']['rotor9_vs_child13']=n.c.material(rotor,child);save()
        reference=[];blocking=[]
        for key,test in state['native'].items():
            for pair in test['overlaps']:
                known=key=='actuated22_vs_other840' and (pair['lead'],pair['part'])==('thumb_ip_stock_central_retainer','thumb_ip_actuator_motor') and abs(pair['material_intersection_mm3']-volume)<1e-5
                (reference if known else blocking).append(dict(group=key,pair=pair))
        state.update(known_source_motor_reference_records=reference,blocking_records=blocking,raw_clear=all(n.m.drive.clear(z) for z in state['native'].values()),
            conditional_clear=not blocking and not any(z['operation_failures'] for z in state['native'].values()))
        save();print('hardware',rec['id'],'blocking',len(blocking),'original_reference',len(reference),flush=True)
        if not state['conditional_clear']:break
    agg=k.flat([z for state in report['states'] for z in state['native'].values()]);report.update(completed=True,native_snapshot_executions=len(report['states']),
        all_material_snapshot_hardware_checks_clear=len(report['states'])==len(raw['states']) and all(z['conditional_clear'] for z in report['states']),
        pairs_in_scope=agg['pairs_in_scope'],AABB_native_pairs=agg['exact_pairs'],overlap_records=len(agg['overlaps']),operation_failures=len(agg['operation_failures']),
        raw_all_clear=all(z['raw_clear'] for z in report['states']),input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()),
        vendor_internal_fit_thread_torque_or_physical_approval=False)
    save()


if __name__=='__main__':main()
