"""Recheck changed guide hardware entry; old assembly is not inherited."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import inboard_model as a
m,g,c,p,np,cq,ROOT,HERE=a.m,a.g,a.c,a.p,a.np,a.cq,a.ROOT,a.HERE
base=m.drive.module('_outlet029_service_base',ROOT/'cad/long_housing_route_v1/build_check.py')
from OCP.OSD import OSD_ThreadPool

def sequence():return [('wrist_inboard_guide_clip',[-20,-10,-5,-1,0]),('wrist_edge_clip_front_cap',[12,6,3,1,0])]+[('wrist_edge_clip_bolt_'+str(i),[20,10,5,1,0]) for i in [0,1]]+[('wrist_edge_clip_nut_'+str(i),[-20,-10,-5,-1,0]) for i in [0,1]]
def main():
    out=HERE/'inboard/reviews/service.json';assert not out.exists();static=p.read(HERE/'inboard/reviews/final.json');assert static['completed'] and static['local_eight_static_clear'];data,pack,guide=base.inputs();hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,np.zeros(20));scene=p.read(HERE/'inboard/generated/assembly_scene.json');rows=c.load(scene['new_parts']+scene['retained_parts'],data[0]);fixed=hand+pack+data[6]+data[7];installed=[];r=dict(completed=False,source_sha256=data[0],paths=[],OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,local_housing_cord_references_absent=True,manual_support_before_closing_clamp=True,whole_housing_threading_tested=False)
    for name,offsets in sequence():
        part=next(z for z in rows if z['name']==name);case=dict(name=name,installed_before=[z['name'] for z in installed],absent_hardware=[z['name'] for z in rows if z['installed_part_candidate'] and z['name']!=name and z['name'] not in {x['name'] for x in installed}],samples=[])
        for dy in offsets:case['samples'].append(dict(translation_mm=[0,dy,0],native=c.material([dict(part,shape=part['shape'].translate((0,dy,0)))],fixed+installed)))
        r['paths'].append(case);p.write(out,r);installed.append(part);print(name,[len(z['native']['overlaps']) for z in case['samples']],flush=True)
    clip=next(z['shape'] for z in rows if z['name']=='wrist_inboard_guide_clip');cap=next(z['shape'] for z in rows if z['name']=='wrist_edge_clip_front_cap');saddle=next(z['shape'] for z in data[7] if z['name']=='Palm_wrist_saddle');contacts=[];stops=[]
    for name,point,normal,first in [('rear_face',[50,-6,-52],[0,1,0],clip),('front_face',[50,0,-52],[0,-1,0],cap),('outer_edge',[54,-3,-52],[-1,0,0],clip)]:
        pt=np.asarray(point);n=np.asarray(normal);contacts.append(dict(id=name,point_mm=point,normal=normal,distance_mm=[sh.distance(cq.Vertex.makeVertex(*pt)) for sh in [first,saddle]],material=[[sh.isInside(cq.Vector(*(pt+s*.05*n)),1e-6) for s in [-1,1]] for sh in [first,saddle]]))
    for i in [0,1]:
        housing=next(z for z in rows if z['name']=='inboard_local_housing_'+str(i))
        for sign in [-1,1]:stops.append(dict(line=i,translation_mm=[sign*.2,0,0],native=c.material([dict(housing,shape=housing['shape'].translate((sign*.2,0,0)))],[dict(name='actual_inboard_bore',shape=clip)])))
    tests=[z['native'] for x in r['paths'] for z in x['samples']];r.update(completed=True,required_sampled_hardware_paths_clear=all(m.drive.clear(z) for z in tests),samples=len(tests),pairs_in_scope=sum(z['pairs_in_scope'] for z in tests),AABB_native_pairs=sum(z['exact_pairs'] for z in tests),overlap_records=sum(len(z['overlaps']) for z in tests),operation_failures=sum(len(z['operation_failures']) for z in tests),contacts=contacts,radial_stops=stops,preload_friction_stiffness_or_axial_retention_verified=False)
    r['source_sha256']['cad/wrist_outlet_route_v1/inboard_service.py']=m.drive.sha(Path(__file__));p.write(out,r)
if __name__=='__main__':main()
