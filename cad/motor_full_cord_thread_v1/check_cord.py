"""Actual seated hardware and two whole housings stay present at every feed."""
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import cord_model as n
t,w,l,a,m,g,c,p,np,cq,ROOT,HERE=n.t,n.w,n.l,n.a,n.m,n.g,n.c,n.p,n.np,n.cq,n.ROOT,n.HERE
from OCP.OSD import OSD_ThreadPool


def aggregate(tests):return dict(pairs_in_scope=sum(z['pairs_in_scope'] for z in tests),exact_pairs=sum(z['exact_pairs'] for z in tests),overlaps=[v for z in tests for v in z['overlaps']],operation_failures=[v for z in tests for v in z['operation_failures']])


def export(rows,label):
    skip={'step','mesh','sha256_step','sha256_stl'}
    return a.a.prior.export([{k:v for k,v in z.items() if k not in skip} for z in rows],HERE/'generated'/label)


def main():
    out=HERE/'reviews/check.json';assert not out.exists();case=t.neutral();data,groups=t.prior.inputs(case)
    form=p.read(ROOT/'cad/progressive_tail_assembly_v1/reviews/restoration.json');assert form['completed'] and form['conditional_CAD_sequence_clear']
    housing=c.load(p.read(ROOT/form['final']['scene'])['parts'],data[0]);retained=[];start=time.monotonic()
    report=dict(completed=False,source_sha256=data[0],cases=[],plans=[],all_hardware_present={k:[z['name'] for z in zz] for k,zz in groups},
                omitted_hardware=[],housing_names=[z['name'] for z in housing],housing_scene=form['final']['scene'],
                start_condition='FORM042 two whole housings geometrically seated, all862 hardware present, no cords. Feed from motor R10 wrap start; open wraps and long incoming straight tail manually held.',
                whole_housing_moved_or_passed_through_motor_stop=False,manual_holding=True,actual_knot_or_clamp_force_verified=False,
                EI_gravity_torsion_friction_preload_force_or_motion_solved=False,reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads())
    for i in [0,1]:
        path,labels,length,anchor=n.specification(case,i);report['plans'].append(dict(line=i,tips_mm=n.samples(case,i),labels=labels,material_length_mm=length,anchor_tail_length_mm=anchor.Length(),start_motor_wrap_mm=path[0].startPoint().toTuple(),incoming_tangent=path[0].tangentAt(0).toTuple()))
    for rel in ['cad/motor_full_cord_thread_v1/cord_model.py','cad/motor_full_cord_thread_v1/check_cord.py','cad/biarc_supported_link_v1/whole_cord.py','cad/biarc_supported_link_v1/link_model.py','cad/biarc_supported_link_v1/reviews/check.json','cad/motor_tendon_terminal_v1/terminal_selected.py','cad/progressive_tail_assembly_v1/reviews/restoration.json',form['final']['scene']]:report['source_sha256'][rel]=m.drive.sha(ROOT/rel)
    def save():a.save(out,report)
    def test(i,tip,fraction,label,store):
        rows,info=n.build(case,i,tip,fraction);tests={key:c.material(rows,zz) for key,zz in groups};tests['whole_housings']=c.material(rows,housing);tests['earlier_cord']=c.material(rows,retained);tests['self']=aggregate([c.material([z],rows[j+1:]) for j,z in enumerate(rows)])
        raw_clear=all(m.drive.clear(z) for z in tests.values());known=[];blocking=[]
        for key,zz in tests.items():
            for pair in zz['overlaps']:
                if key=='self' and fraction is not None and {pair['lead'],pair['part']}=={'motorfeed_cord_'+str(i)+'_0','motorfeed_cord_'+str(i)+'_1'}:
                    known.append(dict(group=key,pair=pair,classification='Same physical cord, adjacent motor-tail/wrap model joint. Raw native overlap retained; no physical smooth-bend or fixation approval.'))
                else:blocking.append(dict(group=key,pair=pair))
        proceed=not blocking and not any(z['operation_failures'] for z in tests.values())
        record=dict(id=label,**info,native=tests,raw_native_clear=raw_clear,blocking_overlap_records=blocking,known_adjacent_motor_joint_records=known,
                    bounded_sequence_can_continue=proceed,retained_cord_names=[z['name'] for z in retained],rows=len(rows))
        report['cases'].append(record);save()
        if store or not proceed or not raw_clear:record['scene']=export(rows,label);save()
        print(label,raw_clear,'blocking',len(blocking),'adjacent',len(known),flush=True)
        return proceed,rows
    ok=True
    for plan in report['plans']:
        i=plan['line'];tips=plan['tips_mm']
        for j,tip in enumerate(tips):
            ok,rows=test(i,tip,None,'feed_'+str(i)+'_'+str(j),j in [0,len(tips)-1])
            if not ok:break
        if not ok:break
        for j,fraction in enumerate([.5,1.]):
            ok,rows=test(i,tips[-1],fraction,'tail_'+str(i)+'_'+str(j),True)
            if not ok:break
        if not ok:break
        retained+=rows
    agg=aggregate([z for rec in report['cases'] for z in rec['native'].values()]);report.update(completed=True,bounded_sequence_clear_of_blocking_overlaps=ok,
        raw_native_all_clear=all(z['raw_native_clear'] for z in report['cases']),pairs_in_scope=agg['pairs_in_scope'],AABB_native_pairs=agg['exact_pairs'],
        overlap_records=len(agg['overlaps']),operation_failures=len(agg['operation_failures']),new_hardware_rows=0,actual_whole_cords_threaded=False,
        physical_motor_or_distal_tails_fixed=False,whole_hand_drive_approved=False,elapsed_s=time.monotonic()-start,
        input_hash_guard=all(m.drive.sha(ROOT/f)==h for f,h in report['source_sha256'].items()));save()


if __name__=='__main__':main()
