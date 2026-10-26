import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import final_edge_model as a
from wrist_survey import base,export
m,g,c,p,np,cq,ROOT,HERE=a.m,a.g,a.c,a.p,a.np,a.cq,a.ROOT,a.HERE
from OCP.OSD import OSD_ThreadPool

def main():
    out=HERE/'selected2/reviews/check.json';assert not out.exists();start=time.monotonic();rows,design=a.build();data,pack,guide=base.inputs()
    tests=[c.material([z],rows[i+1:]) for i,z in enumerate(rows)]
    r=dict(completed=False,design=design,scene=export(rows,HERE/'selected2/generated/parts'),source_sha256=data[0],internal=tests,cases=[],OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6)
    p.write(out,r)
    for case in p.read(ROOT/'cad/thumb_guide_assembly_v2/reviews/static.json')['cases']:
        hand,v,F,N=m.prior.s.k.e.prior.hand_rows(data,guide,case['angles_deg'])
        native={key:c.material(rows,ob) for key,ob in [('hand',hand),('packaging',pack),('electrical',data[6]),('arm',data[7])]}
        got=dict(id=case['id'],angles_deg=case['angles_deg'],native=native,clear=all(m.drive.clear(z) for z in native.values()))
        r['cases'].append(got);p.write(out,r);print(case['id'],{k:z['overlaps'] for k,z in native.items()},flush=True)
        if not got['clear'] or not all(m.drive.clear(z) for z in tests):break
    alltests=tests+[z for case in r['cases'] for z in case['native'].values()]
    r.update(completed=True,static_candidate_clear=len(r['cases'])==8 and all(m.drive.clear(z) for z in alltests),pairs_in_scope=sum(z['pairs_in_scope'] for z in alltests),AABB_native_pairs=sum(z['exact_pairs'] for z in alltests),overlap_records=sum(len(z['overlaps']) for z in alltests),operation_failures=sum(len(z['operation_failures']) for z in alltests),input_hash_guard=all(m.drive.sha(ROOT/z)==h for z,h in data[0].items()),elapsed_s=time.monotonic()-start)
    for name in ['wrist_survey.py','wrist_guide_model.py','saddle_edge_model.py','final_edge_model.py','final_check.py']:r['source_sha256']['cad/wrist_housing_guide_v1/'+name]=m.drive.sha(HERE/name)
    p.write(out,r);print('done',r['static_candidate_clear'],flush=True)
if __name__=='__main__':main()
