"""Finite approach/release candidates against the actual current solids."""
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import route_solver as m
g,c,p,np,cq,ROOT,HERE=m.g,m.c,m.p,m.np,m.cq,m.ROOT,m.HERE
check=m.drive.module('_housing027_inputs',ROOT/'cad/sheath_retention_v1/check.py')
from OCP.OSD import OSD_ThreadPool
def inputs():
    design=p.read(ROOT/'cad/sheath_retention_v1/selected/reviews/check.json')['design'];data,pack,guide,witness=check.inputs(design);retainers=c.load(p.read(ROOT/'cad/sheath_retention_v1/selected/generated/parts/scene.json')['parts'],data[0]);guide+=[z for z in retainers if z['body']=='thumb_mcp'];pack+=[z for z in retainers if z['body']=='forearm']
    for n in ['route_solver.py','build_check.py']:data[0]['cad/long_housing_route_v1/'+n]=m.drive.sha(HERE/n)
    return data,pack,guide
def export(rows,folder):
    folder.mkdir(parents=True,exist_ok=False);meta=[]
    for z in rows:
        st=folder/(z['name']+'.step');me=st.with_suffix('.stl');cq.exporters.export(z['shape'],str(st));cq.exporters.export(z['shape'],str(me),tolerance=.05,angularTolerance=.1);meta.append(dict(**{k:v for k,v in z.items() if k!='shape'},step=st.relative_to(ROOT).as_posix(),mesh=me.relative_to(ROOT).as_posix(),sha256_step=m.drive.sha(st),sha256_stl=m.drive.sha(me)))
    p.write(folder/'scene.json',dict(parts=meta,physical_housings=2,physical_cords=2,CAD_pieces_not_connectors=True));return (folder/'scene.json').relative_to(ROOT).as_posix()
def main():
    out=HERE/'reviews/check.json';assert not out.exists();start=time.monotonic();data,pack,guide=inputs();target=np.asarray(next(z for z in p.read(ROOT/'cad/thumb_guide_assembly_v2/reviews/static.json')['cases'] if z['id']=='parent_full')['angles_deg']);r=dict(completed=False,source_sha256=data[0],cases=[],reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),physical_end_supports='Current motor barrel/retainers and thumbMCP ferrule/retainers only;no midspan guide.Points are curve control coefficients,not installed supports.',elastic_modulus_friction_loads_or_pullout_verified=False,wrist_motion_tested=False,whole_hand_drive_approved=False)
    previous=[None,None]
    for index,factor in enumerate([0.,.25,.5,.75,1.,.75,.5,.25,0.]):
        q=target*factor;solutions=[m.solve(q,i,previous[i]) for i in [0,1]];rows=[];lengths=[]
        for i,solution in enumerate(solutions):
            new,ls=m.tubes(solution);rows+=new;lengths.append(ls);previous[i]=np.asarray(solution['controls'])
        hand,v,F,N=m.prior.s.k.e.prior.hand_rows(data,guide,q);tests={key:c.material(rows,ob) for key,ob in [('hand',hand),('packaging',pack),('electrical',data[6]),('arm',data[7])]};internal=[c.material([z],rows[j+1:]) for j,z in enumerate(rows)];scene=export(rows,HERE/'generated'/('pose_'+str(index)))
        got=dict(id=index,phase='approach' if index<=4 else 'release',factor=factor,angles_deg=q.tolist(),solutions=solutions,geometric_lengths_mm=lengths,scene=scene,native=tests,internal=internal,clear=all(m.drive.clear(z) for z in list(tests.values())+internal));r['cases'].append(got);p.write(out,r);print(index,factor,got['clear'],{k:len(z['overlaps']) for k,z in tests.items()},flush=True)
    alltests=[z for a in r['cases'] for z in list(a['native'].values())+a['internal']];r.update(completed=True,sampled_candidate_clear=all(z['clear'] for z in r['cases']),pairs_in_scope=sum(z['pairs_in_scope'] for z in alltests),AABB_native_pairs=sum(z['exact_pairs'] for z in alltests),overlap_records=sum(len(z['overlaps']) for z in alltests),operation_failures=sum(len(z['operation_failures']) for z in alltests),input_hash_guard=all(m.drive.sha(ROOT/z)==h for z,h in data[0].items()),elapsed_s=time.monotonic()-start);p.write(out,r)
if __name__=='__main__':main()
