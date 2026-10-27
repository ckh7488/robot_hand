"""Fit the lower captured ferrule and jaws on a separate bench first."""
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import remaining_service as a
m,g,c,p,np,cq,ROOT,HERE=a.m,a.g,a.c,a.p,a.np,a.cq,a.ROOT,a.HERE
from OCP.OSD import OSD_ThreadPool

def main():
    out=HERE/'reviews/selected_service.json';assert not out.exists();start=time.monotonic();raw=p.read(HERE/'reviews/service.json');assert raw['completed'] and not raw['sampled_paths_clear'];assert all(m.drive.sha(ROOT/z)==h for z,h in raw['source_sha256'].items())
    data,allrows,by=a.inputs();names=[z['name'] for z in allrows];ports=p.read(ROOT/'cad/sheath_retention_v1/selected/reviews/check.json')['design']['ports'];tag='distal_housing_clamp_0';axis=np.asarray(next(z for z in ports if z['tag']==tag)['Q'])[:3,2];Q=g.original.layout()[0]['socket_transform'];R=g.original.WORLD[:3,:3];x,u,z=R@Q[:3,0],R@Q[:3,1],-R@Q[:3,2]
    kit=['distal_flanged_ferrule_0_captured','distal_short_housing_0']+[n for n in names if n.startswith(tag+'_')];assert len(kit)==8
    shift=np.array([500.,0,0]);benchby={n:dict(by[n],shape=by[n]['shape'].translate(tuple(shift))) for n in kit};bench=a.Study(benchby,[kit[0]],'lower_bench_kit')
    bench.straight('short_end_reference','distal_short_housing_0',u,[-30,-15,-5,-1,0]);ok=bench.paths[-1]['clear']
    if ok:
        for suffix,direction,ds in [('jaw_-1',-axis,[14,7,2,0]),('jaw_1',axis,[14,7,2,0])]+[('nut_'+str(j),-axis,[10,5,0]) for j in [0,1]]+[('bolt_'+str(j),axis,[10,5,0]) for j in [0,1]]:
            if not bench.straight(suffix,tag+'_'+suffix,direction,ds):ok=False;break
    lower=a.Study(by,raw['studies'][1]['initial_installed'],'lower_preassembled_kit');
    if ok:
        for label,name,axis0,ds in [('remove_original_bolt','liner_keeper_M2x10_0',z,[0,4,8]),('remove_original_nut','liner_keeper_M2_nut_0',x,[0,-6,-12]),('remove_original_keeper','liner_flange_keeper_0',z,[0,7,14])]:
            if not lower.straight(label,name,axis0,ds,'remove'):ok=False;break
    if ok:ok=lower.path('captured_ferrule_clamp_kit_axial',kit,[u*d for d in [-25,-15,-5,-1,0]])
    if ok:ok=lower.path('shoe_side_then_radial',['distal_housing_shoe_0'],[x*d-z*3 for d in [-20,-10,-5,0]]+[z*d for d in [-2,-1,-.5,0]])
    if ok:
        seq=[('keeper','distal_housing_keeper_0',z,[14,7,3,1,0]),('original_nut','liner_keeper_M2_nut_0',x,[-12,-6,0]),('original_bolt','liner_keeper_M2x10_0',z,[8,4,0])]+[('shoe_nut_'+str(j),'distal_shoe_M2_nut_0_'+str(j),x,[-12,-6,0]) for j in [1,0]]+[('shoe_bolt_'+str(j),'distal_shoe_M2x10_0_'+str(j),z,[8,4,0]) for j in [1,0]]
        for label,name,axis0,ds in seq:
            if not lower.straight(label,name,axis0,ds):ok=False;break
    # The failed radial entry is retained as actual intersection evidence.
    failed=raw['studies'][1]['paths'][-1];sample=failed['samples'][0];moving=dict(by[failed['moving'][0]],shape=by[failed['moving'][0]]['shape'].translate(tuple(sample['translation_world_mm'])));worst=max(sample['native']['overlaps'],key=lambda v:v['material_intersection_mm3']);common=moving['shape'].intersect(by[worst['part']]['shape'],tol=1e-6)
    export=m.drive.module('_assembly032_evidence_export',ROOT/'cad/wrist_housing_guide_v1/wrist_survey.py').export
    evidence=export([dict(name='actual_lower_jaw_radial_entry_common',shape=common,color=[1.,.08,.05],inspection_only=True)],HERE/'generated/actual_common')
    studies=[raw['studies'][0],bench.report(),lower.report()];tests=[s['native'] for study in studies for path in study['paths'] for s in path['samples']]
    fresh=[s['native'] for study in studies[1:] for path in study['paths'] for s in path['samples']]
    data[0]['cad/remaining_retainer_assembly_v1/preassemble_lower.py']=m.drive.sha(Path(__file__));data[0]['cad/remaining_retainer_assembly_v1/reviews/service.json']=m.drive.sha(HERE/'reviews/service.json')
    r=dict(completed=True,source_sha256=data[0],studies=studies,sampled_paths_clear=bool(ok and all(z['clear'] for z in studies)),selected_finite_samples=len(tests),new_finite_samples=len(fresh),same_motor_samples_reused=20,reused_motor_input_hashes_verified=True,bench_translation_world_mm=shift.tolist(),bench_kit_names=kit,bench_study_id='lower_bench_kit',hand_study_id='lower_preassembled_kit',pairs_in_scope=sum(z['pairs_in_scope'] for z in tests),AABB_native_pairs=sum(z['exact_pairs'] for z in tests),overlap_records=sum(len(z['overlaps']) for z in tests),operation_failures=sum(len(z['operation_failures']) for z in tests),final_current_configuration_restored=set(lower.installed)==set(names),OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,actual_failed_common_scene=evidence,failed_pair=worst,failed_translation_world_mm=sample['translation_world_mm'],hand_angles_deg=[0.]*20,wrist_flex_deviation_deg=[0,0],independent_conditional_studies=True,manual_holding_of_loose_parts_required=True,physical_fixture_tools_threads_torque_strength_verified=False,grip_compression_friction_pullout_verified=False,long_housing_installed=False,cord_threading_tested=False,whole_hand_drive_approved=False,geometry_changed=False,input_hash_guard=all(m.drive.sha(ROOT/z)==h for z,h in data[0].items()),elapsed_s=time.monotonic()-start)
    p.write(out,r);print('finished',r['sampled_paths_clear'],r['selected_finite_samples'],r['overlap_records'],flush=True)
if __name__=='__main__':main()
