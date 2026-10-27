"""Occupancy union of the prescribed fixed-path feed, not installed material."""
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import check_thread as k
t,a,m,c,p,ROOT,HERE=k.t,k.a,k.m,k.c,k.p,k.ROOT,k.HERE
from OCP.OSD import OSD_ThreadPool


def build(case,i,kind):
    edges,length=t.specification(case,i,kind);end=sum(z.Length() for z in edges);pieces=t.window_edges(edges,end,length+end);rows=[]
    for j,ed in enumerate(pieces):
        sh=m.swept(ed,1.5).cut(m.swept(ed,.65)).clean() if kind=='housing' else m.swept(ed,.3)
        assert sh.isValid() and len(sh.Solids())==1
        rows.append(dict(name='feed_occupancy_'+kind+'_'+str(i)+'_'+str(j),shape=sh,kind=kind,physical_line=i,segment=j,length_mm=ed.Length(),installed_part_candidate=False,occupancy_union_only=True,body='palm',color=[.38,.7,.9]))
    return rows,dict(material_length_mm=length,path_end_mm=end,occupancy_coordinate_interval_mm=[-length,end],interval_union_identity='union_{0<=tip<=T}[tip-L,tip]=[-L,T]',fixed_circle_or_annular_cross_section=True,actual_mechanics_or_threading_approved=False)


def main():
    out=HERE/'reviews/envelopes.json';assert not out.exists();raw=p.read(HERE/'reviews/check.json');assert raw['completed'] and raw['sequence_geometry_clear'];case=t.neutral();data,groups=t.prior.inputs(case);report=dict(completed=False,cases=[],source_sha256=data[0],native_group_replays_of_finite_samples=0,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6);retained=[];start=time.monotonic()
    for plan in raw['plans']:
        kind,i=plan['kind'],plan['line'];rows,info=build(case,i,kind);tests={key:c.material(rows,zz) for key,zz in groups};tests['earlier_material']=c.material(rows,retained);tests['self']=k.aggregate([c.material([z],rows[j+1:]) for j,z in enumerate(rows)]);clear=all(m.drive.clear(z) for z in tests.values());scene=a.a.prior.export(rows,HERE/'generated'/('occupancy_'+kind+'_'+str(i)));report['cases'].append(dict(kind=kind,line=i,**info,scene=scene,native=tests,clear=clear,retained_material_names=[z['name'] for z in retained]));print('occupancy',kind,i,clear,flush=True)
        final=next(z for z in reversed(raw['cases']) if z['kind']==kind and z['line']==i);retained+=c.load(p.read(ROOT/final['scene'])['parts'],data[0]);a.save(out,report)
    for f in ['cad/whole_line_threading_v1/feed_envelopes.py','cad/whole_line_threading_v1/thread_model.py','cad/whole_line_threading_v1/reviews/check.json','cad/biarc_supported_link_v1/link_model.py','cad/biarc_supported_link_v1/whole_cord.py']:report['source_sha256'][f]=m.drive.sha(ROOT/f)
    tests=[z for v in report['cases'] for z in v['native'].values()];report.update(completed=True,prescribed_occupancy_clear=all(m.drive.clear(z) for z in tests),pairs_in_scope=sum(z['pairs_in_scope'] for z in tests),AABB_native_pairs=sum(z['exact_pairs'] for z in tests),overlap_records=sum(len(z['overlaps']) for z in tests),operation_failures=sum(len(z['operation_failures']) for z in tests),input_hash_guard=all(m.drive.sha(ROOT/f)==h for f,h in data[0].items()),elapsed_s=time.monotonic()-start);a.save(out,report)
if __name__=='__main__':main()
