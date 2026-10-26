"""Hardware placement only, before threading either long housing."""
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import final_edge_model as a
from wrist_survey import base
m,g,c,p,np,cq,ROOT,HERE=a.m,a.g,a.c,a.p,a.np,a.cq,a.ROOT,a.HERE
from OCP.OSD import OSD_ThreadPool

def sequence():
    return [('wrist_oblique_guide_clip',[-20,-10,-5,-1,0]),('wrist_edge_clip_front_cap',[12,6,3,1,0])]+[('wrist_edge_clip_bolt_'+str(i),[20,10,5,1,0]) for i in [0,1]]+[('wrist_edge_clip_nut_'+str(i),[-20,-10,-5,-1,0]) for i in [0,1]]

def main():
    out=HERE/'selected2/reviews/service.json';assert not out.exists();static=p.read(HERE/'selected2/reviews/check.json');assert p.read(HERE/'selected2/reviews/hardware_follow.json')['hardware_eight_static_clear']
    start=time.monotonic();data,pack,guide=base.inputs();hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,np.zeros(20));rows=c.load(p.read(ROOT/static['scene'])['parts'],data[0]);installed=[]
    r=dict(completed=False,source_sha256=data[0],paths=[],OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,hand_angles_deg=[0.]*20,local_housing_cord_references_absent=True,whole_long_housing_insertion_tested=False,temporary_manual_support='Hold rear clip against saddle edge until cap and fasteners are placed;no physical jig/tool modeled.',all_current_hand_motor_terminal_electrical_parts_remain=True)
    fixed=hand+pack+data[6]+data[7]
    for name,offsets in sequence():
        part=next(z for z in rows if z['name']==name);ob=fixed+installed;case=dict(name=name,installed_before=[z['name'] for z in installed],absent_hardware=[z['name'] for z in rows if z.get('installed_part_candidate') and z['name']!=name and z['name'] not in {x['name'] for x in installed}],samples=[])
        for dy in offsets:
            moved=dict(part,shape=part['shape'].translate((0,dy,0)));got=c.material([moved],ob);case['samples'].append(dict(translation_mm=[0,dy,0],native=got))
        r['paths'].append(case);p.write(out,r);installed.append(part);print(name,[len(z['native']['overlaps']) for z in case['samples']],flush=True)
    alltests=[z['native'] for x in r['paths'] for z in x['samples']]
    # The outside edge and both wall faces are actual material seats, not an
    # inference from a rigid body assignment. Friction/load remains unqualified.
    clip=next(z for z in rows if z['name']=='wrist_oblique_guide_clip')['shape'];cap=next(z for z in rows if z['name']=='wrist_edge_clip_front_cap')['shape'];saddle=next(z['shape'] for z in data[7] if z['name']=='Palm_wrist_saddle')
    contacts=[]
    for label,point,normal,first,second in [('rear_face',[50,-6,-52],[0,1,0],clip,saddle),('front_face',[50,0,-52],[0,-1,0],cap,saddle),('outer_edge',[54,-3,-52],[-1,0,0],clip,saddle)]:
        pt=np.array(point,dtype=float);n=np.array(normal,dtype=float);dist=[first.distance(cq.Vertex.makeVertex(*pt)),second.distance(cq.Vertex.makeVertex(*pt))]
        classifications=[[sh.isInside(cq.Vector(*(pt+s*.05*n)),1e-6) for s in [-1,1]] for sh in [first,second]]
        contacts.append(dict(id=label,point_mm=point,normal=normal,distance_mm=dist,material_at_minus_plus_0p05_mm=classifications,nominal_contact=min(dist)<1e-5 and max(dist)<1e-5 and classifications==[[True,False],[False,True]]))
    stops=[]
    for i in [0,1]:
        housing=next(z for z in rows if z['name']=='wrist_local_housing_'+str(i))
        for sign in [-1,1]:
            delta=sign*.2*np.array([-1,1,0])/np.sqrt(2.);got=c.material([dict(housing,shape=housing['shape'].translate(tuple(delta)))],[dict(name='actual_guide_clip',shape=clip)])
            stops.append(dict(line=i,normal_displacement_mm=delta.tolist(),native=got,geometric_radial_stop=bool(got['overlaps']) and not got['operation_failures']))
    r.update(completed=True,required_sampled_hardware_paths_clear=all(m.drive.clear(z) for z in alltests),samples=len(alltests),pairs_in_scope=sum(z['pairs_in_scope'] for z in alltests),AABB_native_pairs=sum(z['exact_pairs'] for z in alltests),overlap_records=sum(len(z['overlaps']) for z in alltests),operation_failures=sum(len(z['operation_failures']) for z in alltests),seat_contacts=contacts,radial_stop_probes=stops,guide_friction_axial_sliding_or_loaded_stiffness_verified=False,whole_motor_thumb_connection_complete=False,elapsed_s=time.monotonic()-start)
    for name in ['wrist_survey.py','wrist_guide_model.py','saddle_edge_model.py','final_edge_model.py','final_check.py','guide_service.py']:r['source_sha256']['cad/wrist_housing_guide_v1/'+name]=m.drive.sha(HERE/name)
    p.write(out,r);print('done',r['required_sampled_hardware_paths_clear'],[z['nominal_contact'] for z in contacts],flush=True)
if __name__=='__main__':main()
