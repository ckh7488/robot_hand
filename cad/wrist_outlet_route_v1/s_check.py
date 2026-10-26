import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import s_exit_model as a
m,g,c,p,np,cq,ROOT,HERE=a.m,a.g,a.c,a.p,a.np,a.cq,a.ROOT,a.HERE
base=m.drive.module('_outlet029_s_base',ROOT/'cad/long_housing_route_v1/build_check.py')
export=m.drive.module('_outlet029_s_export',ROOT/'cad/wrist_housing_guide_v1/wrist_survey.py').export
from OCP.OSD import OSD_ThreadPool

def main():
    out=HERE/'selected/reviews/check.json';assert not out.exists();data,pack,guide=base.inputs();hardware=c.load([z for z in p.read(ROOT/'cad/wrist_housing_guide_v1/selected2/generated/parts/scene.json')['parts'] if z['installed_part_candidate']],data[0]);r=dict(completed=False,source_sha256=data[0],cases=[],OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,full_housing_connection_approved=False)
    cases=p.read(ROOT/'cad/thumb_guide_assembly_v2/reviews/static.json')['cases'];cases=[cases[4],cases[0],*cases[1:4],*cases[5:]]
    for index,case in enumerate(cases):
        rows,design=a.build(case['angles_deg']);hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,case['angles_deg']);native={key:c.material(rows,ob) for key,ob in [('hand',hand),('packaging',pack),('electrical',data[6]),('arm',data[7]),('wrist_guide_hardware',hardware)]};internal=[c.material([z],rows[i+1:]) for i,z in enumerate(rows)];got=dict(id=case['id'],angles_deg=case['angles_deg'],design=design,scene=export(rows,HERE/'selected/generated'/('pose_'+str(index))),native=native,internal=internal,clear=all(m.drive.clear(z) for z in list(native.values())+internal));r['cases'].append(got);p.write(out,r);print(case['id'],{k:z['overlaps'] for k,z in native.items()},flush=True)
        if not got['clear']:break
    tests=[z for case in r['cases'] for z in list(case['native'].values())+case['internal']];r.update(completed=True,local_eight_poses_clear=len(r['cases'])==8 and all(x['clear'] for x in r['cases']),pairs_in_scope=sum(z['pairs_in_scope'] for z in tests),AABB_native_pairs=sum(z['exact_pairs'] for z in tests),overlap_records=sum(len(z['overlaps']) for z in tests),operation_failures=sum(len(z['operation_failures']) for z in tests),new_support_hardware_installed=False)
    for name in ['outlet_survey.py','round_exit_model.py','s_exit_model.py','s_check.py']:r['source_sha256']['cad/wrist_outlet_route_v1/'+name]=m.drive.sha(HERE/name)
    p.write(out,r)
if __name__=='__main__':main()
