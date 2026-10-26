"""Five finite approach/release cases; original native obstacles retained."""
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import avoid_solver as m
base=m.drive.module('_housing027_base',m.HERE/'build_check.py')
g,c,p,np,cq,ROOT=m.g,m.c,m.p,m.np,m.cq,m.ROOT
HERE=m.HERE/'selected'
from OCP.OSD import OSD_ThreadPool
def inputs():return base.inputs()
def export(rows,folder):
    folder.mkdir(parents=True,exist_ok=False);meta=[]
    for z in rows:
        st=folder/(z['name']+'.step');me=st.with_suffix('.stl');cq.exporters.export(z['shape'],str(st));cq.exporters.export(z['shape'],str(me),tolerance=.2,angularTolerance=.35);meta.append(dict(**{k:v for k,v in z.items() if k!='shape'},step=st.relative_to(ROOT).as_posix(),mesh=me.relative_to(ROOT).as_posix(),sha256_step=m.drive.sha(st),sha256_stl=m.drive.sha(me)))
    p.write(folder/'scene.json',dict(parts=meta,physical_housings=2,physical_cords=2,CAD_pieces_not_connectors=True,mesh_linear_deflection_mm=.2,mesh_angular_deflection_rad=.35));return (folder/'scene.json').relative_to(ROOT).as_posix()
def main():
    out=HERE/'reviews/check.json';assert not out.exists();start=time.monotonic();data,pack,guide=inputs();first=p.read(m.HERE/'reviews/check.json');assert not first['completed'] and first['cases'][0]['clear']==False
    names=sorted({z['part'] for key in ['packaging','arm'] for z in first['cases'][0]['native'][key]['overlaps']});by={z['name']:z for z in pack+data[7]};boxes=[c.m.pack._bbox(by[n]['shape']) for n in names]
    for n in ['avoid_solver.py','selected_check.py']:data[0]['cad/long_housing_route_v1/'+n]=m.drive.sha(m.HERE/n)
    target=np.asarray(next(z for z in p.read(ROOT/'cad/thumb_guide_assembly_v2/reviews/static.json')['cases'] if z['id']=='parent_full')['angles_deg']);r=dict(completed=False,source_sha256=data[0],cases=[],optimization_keepouts=dict(names=names,boxes_mm=boxes,reason='Conservative AABBs of actual first-candidate blocking solids;41curve samples at1.5mmhousing radius.Native tests still include ALL current obstacles.No installed midspan guides.'),reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),elastic_equilibrium_friction_tension_or_pullout_verified=False,wrist_motion_tested=False,whole_hand_drive_approved=False)
    previous=[None,None]
    for index,factor in enumerate([0.,.5,1.,.5,0.]):
        q=target*factor;solutions=[m.solve_avoiding(q,i,boxes,previous[i]) for i in [0,1]];got=dict(id=index,factor=factor,phase='approach' if index<=2 else 'release',angles_deg=q.tolist(),solutions=solutions);r['cases'].append(got);p.write(out,r)
        if not all(z['solver_success'] for z in solutions):r.update(stopped_at_solver=index,completed=True,sampled_candidate_clear=False);p.write(out,r);return
        rows=[];lengths=[]
        for i,solution in enumerate(solutions):
            new,ls=m.tubes(solution);rows+=new;lengths.append(ls);previous[i]=np.asarray(solution['controls'])
        got.update(geometric_lengths_mm=lengths,scene=export(rows,HERE/'generated'/('pose_'+str(index))));p.write(out,r)
        hand,v,F,N=m.prior.s.k.e.prior.hand_rows(data,guide,q);tests={key:c.material(rows,ob) for key,ob in [('hand',hand),('packaging',pack),('electrical',data[6]),('arm',data[7])]};internal=[c.material([z],rows[j+1:]) for j,z in enumerate(rows)];got.update(native=tests,internal=internal,clear=all(m.drive.clear(z) for z in list(tests.values())+internal));p.write(out,r);print(index,factor,got['clear'],{k:len(z['overlaps']) for k,z in tests.items()},flush=True)
        if not got['clear']:r.update(completed=True,stopped_at_native=index,sampled_candidate_clear=False);p.write(out,r);return
    alltests=[z for a in r['cases'] for z in list(a['native'].values())+a['internal']];r.update(completed=True,sampled_candidate_clear=True,pairs_in_scope=sum(z['pairs_in_scope'] for z in alltests),AABB_native_pairs=sum(z['exact_pairs'] for z in alltests),overlap_records=sum(len(z['overlaps']) for z in alltests),operation_failures=sum(len(z['operation_failures']) for z in alltests),input_hash_guard=all(m.drive.sha(ROOT/z)==h for z,h in data[0].items()),elapsed_s=time.monotonic()-start);p.write(out,r)
if __name__=='__main__':main()
