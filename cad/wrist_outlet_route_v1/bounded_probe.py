"""Compare two natural round exits; no radius/angle optimizer."""
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import round_exit_model as a
m,g,c,p,np,cq,ROOT,HERE=a.m,a.g,a.c,a.p,a.np,a.cq,a.ROOT,a.HERE
base=m.drive.module('_outlet029_base',ROOT/'cad/long_housing_route_v1/build_check.py')
export=m.drive.module('_outlet029_export',ROOT/'cad/wrist_housing_guide_v1/wrist_survey.py').export
from OCP.OSD import OSD_ThreadPool

def main():
    out=HERE/'reviews/probe.json';assert not out.exists();data,pack,guide=base.inputs();hardware=c.load([z for z in p.read(ROOT/'cad/wrist_housing_guide_v1/selected2/generated/parts/scene.json')['parts'] if z['installed_part_candidate']],data[0]);r=dict(completed=False,source_sha256=data[0],alternatives=[],OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,full_housing_connection_approved=False)
    cases=p.read(ROOT/'cad/thumb_guide_assembly_v2/reviews/static.json')['cases']
    # Start with the actual failing parent pose, then open and the remaining
    # representative poses only if the new local tube clears that first pose.
    cases=[cases[4],cases[0],*cases[1:4],*cases[5:]]
    for sign in [1,-1]:
        rows,design=a.build(sign);label='left' if sign==1 else 'right';item=dict(id=label,design=design,scene=export(rows,HERE/'generated'/label),cases=[],internal=[c.material([z],rows[i+1:]) for i,z in enumerate(rows)]);r['alternatives'].append(item);p.write(out,r)
        for case in cases:
            hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,case['angles_deg']);native={key:c.material(rows,ob) for key,ob in [('hand',hand),('packaging',pack),('electrical',data[6]),('arm',data[7]),('wrist_guide_hardware',hardware)]};got=dict(id=case['id'],angles_deg=case['angles_deg'],native=native,clear=all(m.drive.clear(z) for z in native.values()) and all(m.drive.clear(z) for z in item['internal']));item['cases'].append(got);p.write(out,r);print(label,case['id'],{k:z['overlaps'] for k,z in native.items()},flush=True)
            if not got['clear']:break
        item['local_static8_clear']=len(item['cases'])==8 and all(z['clear'] for z in item['cases']);p.write(out,r)
    alltests=[z for a0 in r['alternatives'] for z in a0['internal']+[v for case in a0['cases'] for v in case['native'].values()]]
    r.update(completed=True,pairs_in_scope=sum(z['pairs_in_scope'] for z in alltests),AABB_native_pairs=sum(z['exact_pairs'] for z in alltests),overlap_records=sum(len(z['overlaps']) for z in alltests),operation_failures=sum(len(z['operation_failures']) for z in alltests))
    for name in ['outlet_survey.py','round_exit_model.py','bounded_probe.py']:r['source_sha256']['cad/wrist_outlet_route_v1/'+name]=m.drive.sha(HERE/name)
    p.write(out,r)
if __name__=='__main__':main()
