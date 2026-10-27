"""Native tests only for prescribed wrist cases meeting local radius necessity."""
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import material_model as a
m,g,c,p,np,cq,ROOT,HERE=a.m,a.g,a.c,a.p,a.np,a.cq,a.ROOT,a.HERE
from OCP.OSD import OSD_ThreadPool
def obstacles(data,pack,guide,wrist,case):
    q=case['angles_deg'];P=np.asarray(case['solution']['wrist_P']);H=np.asarray(case['solution']['wrist_H']);F=np.asarray(case['solution']['parent_F']);hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,q)
    upper=c.load([z for z in p.read(ROOT/'cad/thumb_entry_bend_v1/generated/parts/scene.json')['parts'] if z['installed_part_candidate']],data[0]);hand+=[dict(z,shape=g.original.transform(z['shape'],F)) for z in upper];hand=[a.feed.transform(z,H) for z in hand];assert len(hand)==397
    return [('hand',hand),('packaging',a.scope.posed_pack(pack,H)),('electrical',data[6]),('arm',a.feed.posed_arm(data[7],P,H)),('wrist_guide',[a.feed.transform(z,H) for z in wrist])]
def main():
    out=HERE/'reviews/check.json';assert not out.exists();start=time.monotonic();analysis=p.read(HERE/'reviews/analysis.json');assert analysis['completed'];data,pack,guide,wrist=a.s.prior.inputs();data[0].update(analysis['source_sha256']);report=dict(completed=False,source_sha256=data[0],cases=[],neutral_native_not_repeated=True,neutral_controls_identical_and_old_scene_reused=True,neutral_reference='cad/contact_housing_link_v1/generated/neutral/scene.json',parent_cases_not_swept_due_local_radius_failure=[z['id'] for z in analysis['cases'] if not z['local_radius_necessary_condition']],planned_new_native_case_ids=[9,10],new_hardware_rows=0,whole_hand_drive_approved=False,physical_housing_installation_or_equilibrium_verified=False,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6)
    for case in [analysis['cases'][j] for j in [9,10]]:
        assert case['solution']['closed'] and case['local_radius_necessary_condition'];rows,lengths=a.build(case);scene=a.a.prior.export(rows,HERE/'generated'/('pose_'+str(case['id'])));got=dict(id=case['id'],scene=scene,geometric_lengths_mm=lengths,native={})
        for key,ob in obstacles(data,pack,guide,wrist,case):got['native'][key]=c.material(rows,ob);print(case['id'],key,len(got['native'][key]['overlaps']),flush=True)
        got['internal']=[c.material([z],rows[j+1:]) for j,z in enumerate(rows)];got['clear']=all(m.drive.clear(z) for z in [*got['native'].values(),*got['internal']]);report['cases'].append(got);a.save(out,report);print('case',case['id'],got['clear'],flush=True)
        if not got['clear']:break
    tests=[z for case in report['cases'] for z in [*case['native'].values(),*case['internal']]]
    for name in ['material_model.py','analyze.py','check_motion.py']:data[0]['cad/material_length_motion_v1/'+name]=m.drive.sha(HERE/name)
    report.update(completed=True,new_native_cases=len(report['cases']),new_wrist_cases_all_clear=len(report['cases'])==2 and all(z['clear'] for z in report['cases']),parent_motion_candidate_admissible=False,all_motion_candidate_admissible=False,pairs_in_scope=sum(z['pairs_in_scope'] for z in tests),AABB_native_pairs=sum(z['exact_pairs'] for z in tests),overlap_records=sum(len(z['overlaps']) for z in tests),operation_failures=sum(len(z['operation_failures']) for z in tests),input_hash_guard=all(m.drive.sha(ROOT/f)==v for f,v in data[0].items()),elapsed_s=time.monotonic()-start);a.save(out,report)
if __name__=='__main__':main()
