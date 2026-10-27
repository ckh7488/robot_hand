"""Sequential whole-housing/whole-cord feed candidate, never a force simulation."""
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import thread_model as t
a,m,c,p,ROOT,HERE=t.a,t.m,t.c,t.p,t.ROOT,t.HERE
from OCP.OSD import OSD_ThreadPool


def aggregate(tests):return dict(pairs_in_scope=sum(z['pairs_in_scope'] for z in tests),exact_pairs=sum(z['exact_pairs'] for z in tests),overlaps=[v for z in tests for v in z['overlaps']],operation_failures=[v for z in tests for v in z['operation_failures']])


def main():
    out=HERE/'reviews/check.json';assert not out.exists();start=time.monotonic();case=t.neutral();data,groups=t.prior.inputs(case);retained=[];records=[]
    report=dict(completed=False,source_sha256=data[0],cases=records,plans=[],all_hardware_present={k:[z['name'] for z in zz] for k,zz in groups},omitted_hardware=[],initial_housings_and_cords_absent=True,actual_clamp_tightening_not_modeled=True,manual_holding_of_tails_and_downstream_curve=True,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,whole_motor_installation_or_motion_approved=False)
    for kind in ['housing','cord']:
        for i in [0,1]:report['plans'].append(dict(kind=kind,line=i,tips_mm=t.samples(case,i,kind),material_length_mm=t.specification(case,i,kind)[1]))
    def save():
        for name in ['thread_model.py','check_thread.py']:report['source_sha256']['cad/whole_line_threading_v1/'+name]=m.drive.sha(HERE/name)
        for name in ['cad/biarc_supported_link_v1/reviews/analysis.json','cad/biarc_supported_link_v1/reviews/check.json','cad/biarc_supported_link_v1/link_model.py','cad/biarc_supported_link_v1/whole_cord.py','cad/biarc_supported_link_v1/details.py']:report['source_sha256'][name]=m.drive.sha(ROOT/name)
        a.save(out,report)
    save();failed=False
    for plan in report['plans']:
        tips=plan['tips_mm'];kind,i=plan['kind'],plan['line']
        for j,tip in enumerate(tips):
            rows,info=t.state(case,i,kind,tip);tests={key:c.material(rows,zz) for key,zz in groups};tests['earlier_material']=c.material(rows,retained);tests['self']=aggregate([c.material([z],rows[k+1:]) for k,z in enumerate(rows)]);clear=all(m.drive.clear(z) for z in tests.values());key=kind+'_'+str(i)+'_'+str(j)
            record=dict(id=key,**info,native=tests,clear=clear,retained_material_names=[z['name'] for z in retained],rows=len(rows))
            if j in [0,len(tips)-1] or not clear:record['scene']=a.a.prior.export(rows,HERE/'generated'/key)
            records.append(record);save();print(key,'tip',tip,'clear',clear,'records',sum(len(z['overlaps']) for z in tests.values()),flush=True)
            if not clear:failed=True;break
        if failed:break
        retained+=rows
    alltests=[z for record in records for z in record['native'].values()];report.update(completed=True,planned_samples=sum(len(z['tips_mm']) for z in report['plans']),completed_samples=len(records),sequence_geometry_clear=not failed,pairs_in_scope=sum(z['pairs_in_scope'] for z in alltests),AABB_native_pairs=sum(z['exact_pairs'] for z in alltests),overlap_records=sum(len(z['overlaps']) for z in alltests),operation_failures=sum(len(z['operation_failures']) for z in alltests),whole_housing_motor_end_seated=False,actual_whole_cords_threaded=False,whole_hand_drive_approved=False,retained_final_material_names=[z['name'] for z in retained],input_hash_guard=all(m.drive.sha(ROOT/f)==h for f,h in data[0].items()),elapsed_s=time.monotonic()-start);save()
if __name__=='__main__':main()
