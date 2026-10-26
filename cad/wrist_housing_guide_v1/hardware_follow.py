"""Finish only hardware poses after the local straight housing was blocked.

The five existing all-ten-shape cases retain both failed housing records.
They already include the six hardware rows, so only the remaining three
hardware cases are new calculations. This does not approve the housing path.
"""
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import final_edge_model as a
from wrist_survey import base
m,g,c,p,np,cq,ROOT,HERE=a.m,a.g,a.c,a.p,a.np,a.cq,a.ROOT,a.HERE
from OCP.OSD import OSD_ThreadPool

def main():
    out=HERE/'selected2/reviews/hardware_follow.json';assert not out.exists();old=p.read(HERE/'selected2/reviews/check.json');assert old['completed'] and len(old['cases'])==5 and not old['static_candidate_clear']
    data,pack,guide=base.inputs();rows=c.load(p.read(ROOT/old['scene'])['parts'],data[0]);hardware=[z for z in rows if z['installed_part_candidate']];names={z['name'] for z in hardware}
    for case in old['cases']:
        for test in case['native'].values():assert not test['operation_failures'] and not any(z['lead'] in names for z in test['overlaps'])
    r=dict(completed=False,source_sha256=data[0],inherited_cases=[dict(id=z['id'],source='cad/wrist_housing_guide_v1/selected2/reviews/check.json',source_sha256=m.drive.sha(HERE/'selected2/reviews/check.json'),all_ten_shapes_tested=True,hardware_overlaps=0) for z in old['cases']],new_cases=[],hardware_rows=6,housing_path_adopted=False,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6)
    for case in p.read(ROOT/'cad/thumb_guide_assembly_v2/reviews/static.json')['cases'][5:]:
        hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,case['angles_deg']);native={key:c.material(hardware,ob) for key,ob in [('hand',hand),('packaging',pack),('electrical',data[6]),('arm',data[7])]};r['new_cases'].append(dict(id=case['id'],angles_deg=case['angles_deg'],native=native));p.write(out,r);print(case['id'],{k:z['overlaps'] for k,z in native.items()},flush=True)
    tests=[z for x in r['new_cases'] for z in x['native'].values()]
    r.update(completed=True,hardware_eight_static_clear=all(m.drive.clear(z) for z in tests),new_pairs_in_scope=sum(z['pairs_in_scope'] for z in tests),new_AABB_native_pairs=sum(z['exact_pairs'] for z in tests),new_overlap_records=sum(len(z['overlaps']) for z in tests),new_operation_failures=sum(len(z['operation_failures']) for z in tests),hardware_static_poses_total=8,inherited_all_ten_poses=5,new_hardware_only_poses=3,whole_motor_thumb_connected=False)
    r['source_sha256']['cad/wrist_housing_guide_v1/hardware_follow.py']=m.drive.sha(Path(__file__));p.write(out,r)
if __name__=='__main__':main()
