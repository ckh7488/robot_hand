"""One neutral full-material candidate with current actual hardware."""
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import whole_cord as w
l,a,m,g,c,p,np,cq,ROOT,HERE=w.l,w.a,w.m,w.g,w.c,w.p,w.np,w.cq,w.ROOT,w.HERE
support=m.drive.module('_biarc039_current_obstacles',ROOT/'cad/support_attachment_basis_v1/check.py')
from OCP.OSD import OSD_ThreadPool

def main():
    out=HERE/'reviews/check.json';assert not out.exists();start=time.monotonic();analysis=p.read(HERE/'reviews/analysis.json');case=analysis['cases'][0];assert case['local_thick_tube_condition'];data,pack,guide,wrist=a.s.prior.inputs();sm=p.read(ROOT/'cad/support_attachment_basis_v1/generated/checked/scene.json')['parts'];hw=c.load([z for z in sm if z['installed_part_candidate']],data[0]);removed=[z['replaced_part'] for z in hw];pack_old=[z for z in pack if z['name'] not in removed];currentcase=dict(angles_deg=case['q'],solution=dict(wrist_H=case['H'],wrist_P=case['P'],parent_F=case['F']));obstacles=support.kobs(data,pack_old,guide,wrist,currentcase);obstacles=[(key,zz+hw if key=='packaging' else zz) for key,zz in obstacles];rows,lengths,chains=w.build(case);scene=a.a.prior.export(rows,HERE/'generated/neutral');report=dict(completed=False,source_sha256=data[0],scene=scene,native={},internal=[],chains=chains,new_CAD_rows=len(rows),physical_housings=2,physical_whole_modeled_cords=2,new_hardware_rows=0,geometric_lengths=lengths,all_modeled_centerline_endpoint_joins_close=True,fixed_material_length_motion_approved=False,actual_threading_preload_force_or_installation_verified=False,whole_hand_drive_approved=False,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6)
    def save():
        for name in ['link_model.py','analyze.py','whole_cord.py','check.py']:report['source_sha256']['cad/biarc_supported_link_v1/'+name]=m.drive.sha(HERE/name)
        for name in ['cad/biarc_supported_link_v1/reviews/analysis.json','cad/motor_tendon_terminal_v1/terminal_selected.py','cad/thumb_sheath_terminal_v2/offset_model.py']:report['source_sha256'][name]=m.drive.sha(ROOT/name)
        a.save(out,report)
    save()
    for key,zz in obstacles:report['native'][key]=c.material(rows,zz);save();print(key,len(report['native'][key]['overlaps']),flush=True)
    report['internal']=[c.material([z],rows[j+1:]) for j,z in enumerate(rows)];tests=[*report['native'].values(),*report['internal']];report.update(completed=True,native_candidate_clear=all(m.drive.clear(z) for z in tests),candidate_adopted=False,pairs_in_scope=sum(z['pairs_in_scope'] for z in tests),AABB_native_pairs=sum(z['exact_pairs'] for z in tests),overlap_records=sum(len(z['overlaps']) for z in tests),operation_failures=sum(len(z['operation_failures']) for z in tests),input_hash_guard=all(m.drive.sha(ROOT/f)==h for f,h in data[0].items()),elapsed_s=time.monotonic()-start);save();print('neutral',report['native_candidate_clear'],report['overlap_records'],flush=True)
if __name__=='__main__':main()
