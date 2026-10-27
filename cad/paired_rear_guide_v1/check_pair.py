"""New paired-channel hardware and local references, with current frames."""
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import pair_model as a
m,g,c,p,np,cq,ROOT,HERE=a.m,a.g,a.c,a.p,a.np,a.cq,a.ROOT,a.HERE
base=m.drive.module('_pair034_inputs',ROOT/'cad/long_housing_route_v1/build_check.py')
scope=m.drive.module('_pair034_pack_scope',ROOT/'cad/wrist_external_feed_v1/correct_wrist_scope.py')
export=m.drive.module('_pair034_export',ROOT/'cad/wrist_housing_guide_v1/wrist_survey.py').export
from OCP.OSD import OSD_ThreadPool

def main():
    out=HERE/'reviews/static.json';assert not out.exists();start=time.monotonic();new,retained,design=a.build();rows=new+retained;data,pack,guide=base.inputs();scene=export(new,HERE/'generated/parts');p.write(HERE/'generated/assembly_scene.json',dict(new_parts=p.read(ROOT/scene)['parts'],retained_parts=[{k:v for k,v in z.items() if k!='shape'} for z in retained],replaced_parts=['wrist_external_feed_clip'],old_local_housing_and_cord_refs_replaced=True,whole_link_installed=False))
    r=dict(completed=False,source_sha256=data[0],design=design,scene=scene,cases=[],internal=[c.material([z],rows[j+1:]) for j,z in enumerate(rows)],OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,whole_hand_drive_approved=False,whole_flexible_link_connected=False,wrist_samples_are_not_angle_limits=True,new_actual_support_not_virtual_waypoints=True)
    for case in scope.b.cases():
        P,H=a.a.wrist_frames(case['wrist_flex_deg'],case['wrist_deviation_deg']);hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,case['angles_deg']);hand=[a.a.transform(z,H) for z in hand];moving=[a.a.transform(z,H) for z in rows]
        tests={k:c.material(moving,ob) for k,ob in [('hand',hand),('packaging',scope.posed_pack(pack,H)),('electrical',data[6]),('arm',a.a.posed_arm(data[7],P,H))]};got=dict(**case,native=tests,clear=all(m.drive.clear(v) for v in tests.values()));r['cases'].append(got);p.write(out,r);print(case['id'],got['clear'],{k:v['overlaps'] for k,v in tests.items()},flush=True)
        if not got['clear']:break
    tests=[*r['internal'],*[v for z in r['cases'] for v in z['native'].values()]]
    for n in ['pair_model.py','check_pair.py','survey_pair.py']:r['source_sha256']['cad/paired_rear_guide_v1/'+n]=m.drive.sha(HERE/n)
    r['source_sha256']['cad/modular_roots_v9/design.json']=m.drive.sha(ROOT/'cad/modular_roots_v9/design.json');r.update(completed=True,sampled_candidate_clear=len(r['cases'])==16 and all(m.drive.clear(z) for z in tests),static_cases_completed=len(r['cases']),pairs_in_scope=sum(z['pairs_in_scope'] for z in tests),AABB_native_pairs=sum(z['exact_pairs'] for z in tests),overlap_records=sum(len(z['overlaps']) for z in tests),operation_failures=sum(len(z['operation_failures']) for z in tests),input_hash_guard=all(m.drive.sha(ROOT/z)==h for z,h in r['source_sha256'].items()),elapsed_s=time.monotonic()-start);p.write(out,r)
if __name__=='__main__':main()
