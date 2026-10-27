"""Bounded sequential forming candidate, keeping the actual cradle present."""
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import progressive_model as n
sys.path.insert(0,str(n.ROOT/'cad/motor_guide_prethread_v1'))
import evidence as old
s,t,a,m,g,c,p,np,cq,ROOT,HERE=n.s,n.t,n.a,n.m,n.g,n.c,n.p,n.np,n.cq,n.ROOT,n.HERE
from OCP.OSD import OSD_ThreadPool


def export(rows,label):
    skip={'step','mesh','sha256_step','sha256_stl','assembly_transform'}
    return a.a.prior.export([{k:v for k,v in z.items() if k not in skip} for z in rows],HERE/'generated'/label)


def main():
    out=HERE/'reviews/check.json';assert not out.exists();prior=p.read(ROOT/'cad/motor_guide_prethread_v1/reviews/check.json')
    case,data,full,groups,hw=old.inputs(prior);cradle=next(z for k,zz in full for z in zz if z['name']=='Elbow_fixed_cradle')
    proof=p.read(ROOT/'cad/motor_guide_prethread_v1/reviews/overlap_evidence.json')
    common=c.load(p.read(ROOT/proof['evidence_scene'])['parts'])[0]
    totals=[case['lines'][i]['up']['length_mm'] for i in [0,1]]
    report=dict(completed=False,source_sha256=data[0],cases=[],restoration=[],initial_missing_clamp_names=prior['initial_missing_clamp_names'],
                fixed_hardware_names={k:[z['name'] for z in zz] for k,zz in groups},
                survey=dict(saved_SEAT041_common_bbox_mm=c.m.pack._bbox(common['shape']),actual_cradle_bbox_mm=c.m.pack._bbox(cradle['shape']),
                            saved_common_values=proof['pairs'][0]['adaptive_source_common_values'],old_native_replays=0),
                start_condition='SEAT041 completed prethread endpoint: both real guides loose, both full housings, twelve clamp parts detached, other850 hardware present; no cords.',
                sequence='Form lower tail from actual PAIR mouth toward free motor guide; hold it, then form upper. Curvature of formed material is saved neutral curvature, unformed remainder straight. No uniform curvature scaling.',
                manual_holding=True,EI_gravity_torsion_contact_force_friction_preload_solved=False,cords_absent=True,
                actual_cradle_retained=True,new_hardware_rows=0,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),
                reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6)
    for rel in ['cad/motor_guide_prethread_v1/reviews/check.json','cad/motor_guide_prethread_v1/reviews/overlap_evidence.json','cad/motor_guide_prethread_v1/seat_model.py',proof['evidence_scene'],*['cad/progressive_tail_assembly_v1/'+f for f in ['progressive_model.py','check_progressive.py']]]:
        report['source_sha256'][rel]=m.drive.sha(ROOT/rel)
    report['source_sha256'].update({z[k]:z[h] for z in p.read(ROOT/proof['evidence_scene'])['parts'] for k,h in [('step','sha256_step'),('mesh','sha256_stl')]})
    report['boundaries']=[]
    for i in [0,1]:
        _,T0,z0=n.edges(case,i,0.);_,T1,z1=n.edges(case,i,totals[i]);_,oldT,oldz=s.edges(case,i,'feed',s.definitions(case,i)[4])
        err=max(float(np.max(abs(T0-oldT))),float(np.max(abs(T1-np.eye(4)))),max(float(np.max(abs(np.asarray(x['world_mm'])-y['world_mm']))) for x,y in zip(z0['material_points'],oldz['material_points'])))
        report['boundaries'].append(dict(line=i,maximum_start_and_declared_target_error=err,matched=bool(err<1e-5),target_is_1D_only=True));assert err<1e-5
    def save():a.save(out,report)
    save();ok=True;start=time.monotonic();last=None
    for i in [0,1]:
        for j,fraction in enumerate([0.,.125,.25,.375,.5,.625,.75,.875,1.]):
            values=[totals[0] if i else totals[0]*fraction,totals[1]*fraction if i else 0.]
            built=[n.build(case,k,values[k],hw[k]) for k in [0,1]];moving=sum([z[0] for z in built],[])+[z[1] for z in built]
            tests={key:c.material(moving,zz) for key,zz in groups};tests['internal']=old.k.combine([c.material([z],moving[q+1:]) for q,z in enumerate(moving)])
            ok=all(m.drive.clear(z) for z in tests.values());record=dict(id='form_'+str(i)+'_'+str(j),active_line=i,fraction=fraction,formed_mm=values,lines=[z[2] for z in built],native=tests,clear=ok)
            report['cases'].append(record);save()
            if j in [0,4,8] or not ok:record['scene']=export(moving,record['id']);save()
            print(record['id'],ok,sum(len(z['overlaps']) for z in tests.values()),flush=True);last=moving
            if not ok:break
        if not ok:break
    report.update(completed=True,forming_sequence_clear=ok,actual_motor_seating_or_fastener_restoration_approved=False,
                  physical_installation_or_force_motion_approved=False,elapsed_s=time.monotonic()-start)
    agg=old.k.combine([z for rec in report['cases'] for z in rec['native'].values()])
    report.update(pairs_in_scope=agg['pairs_in_scope'],AABB_native_pairs=agg['exact_pairs'],overlap_records=len(agg['overlaps']),operation_failures=len(agg['operation_failures']))
    report['input_hash_guard']=all(m.drive.sha(ROOT/f)==h for f,h in report['source_sha256'].items());save()


if __name__=='__main__':main()
