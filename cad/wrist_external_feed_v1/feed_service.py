"""Actual six-hardware entry and a bounded straight-end insertion stroke."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import check_feed as b
a,m,g,c,p,np,cq,ROOT,HERE=b.a,b.m,b.g,b.c,b.p,b.np,b.cq,b.ROOT,b.HERE
from OCP.OSD import OSD_ThreadPool

def sequence():return [('wrist_external_feed_clip',[-20,-10,-5,-1,0]),('wrist_edge_clip_front_cap',[12,6,3,1,0])]+[('wrist_edge_clip_bolt_'+str(i),[20,10,5,1,0]) for i in [0,1]]+[('wrist_edge_clip_nut_'+str(i),[-20,-10,-5,-1,0]) for i in [0,1]]

def main():
    out=HERE/'reviews/service.json';assert not out.exists();static=p.read(HERE/'reviews/final.json');assert static['completed'] and static['sampled_candidate_clear']
    data,pack,guide=b.base.inputs();hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,np.zeros(20));scene=p.read(HERE/'generated/assembly_scene.json');allrows=c.load(scene['new_parts']+scene['retained_parts'],data[0]);rows=[z for z in allrows if z['installed_part_candidate']];assert len(rows)==6
    fixed=hand+pack+data[6]+data[7];installed=[];r=dict(completed=False,source_sha256=data[0],paths=[],OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,local_housing_and_cord_references_absent=True,manual_hold_until_clamp_closed=True,actual_flexible_whole_housing_installation_tested=False)
    for name,offsets in sequence():
        part=next(z for z in rows if z['name']==name);case=dict(name=name,installed_before=[z['name'] for z in installed],absent_hardware=[z['name'] for z in rows if z['name']!=name and z['name'] not in {v['name'] for v in installed}],samples=[])
        for dy in offsets:case['samples'].append(dict(translation_mm=[0,dy,0],native=c.material([dict(part,shape=part['shape'].translate((0,dy,0)))],fixed+installed)))
        installed.append(part);r['paths'].append(case);p.write(out,r);print(name,[len(z['native']['overlaps']) for z in case['samples']],flush=True)
    clip=next(z['shape'] for z in rows if z['name']=='wrist_external_feed_clip');cap=next(z['shape'] for z in rows if z['name']=='wrist_edge_clip_front_cap');saddle=next(z['shape'] for z in data[7] if z['name']=='Palm_wrist_saddle');contacts=[]
    for name,point,normal,first in [('rear_face',[50,-6,-52],[0,1,0],clip),('front_face',[50,0,-52],[0,-1,0],cap),('outer_edge',[54,-3,-52],[-1,0,0],clip)]:
        pt=np.asarray(point);n=np.asarray(normal);contacts.append(dict(id=name,point_mm=point,normal=normal,distance_mm=[sh.distance(cq.Vertex.makeVertex(*pt)) for sh in [first,saddle]],material=[[sh.isInside(cq.Vector(*(pt+s*.05*n)),1e-6) for s in [-1,1]] for sh in [first,saddle]]))
    insertion=[];evidence=[]
    for i,z in enumerate([-3.,2.]):
        housing=next(v for v in allrows if v['name']=='external_feed_housing_'+str(i));envelope=a.cyl(100,z,-238,8,1.5).cut(a.cyl(100,z,-238.01,8.01,.65));row=dict(name='straight_reference_housing_'+str(i)+'_axial_insertion_envelope',shape=envelope)
        got=c.material([row],fixed+rows);outside=[]
        for dy in [-128,0]:
            cut=housing['shape'].translate((0,dy,0)).cut(envelope,tol=1e-6);outside.append(cut.Volume() if cut.Solids() else 0.)
        insertion.append(dict(line=i,reference_length_mm=118,translation_y_mm=[-128,0],envelope_y_mm=[-238,8],native=got,start_end_material_outside_envelope_mm3=outside,
            proof='Uniform annular extrusion along its own axis;all intermediate translations are subsets of this exact axial extrusion.Only a straight118mmend reference,not entire flexible housing or cordthreading.'))
        evidence.append(dict(row,body='palm',color=[.68,.3,.8],installed_part_candidate=False,evidence_only=True))
    envelope_scene=b.export(evidence,HERE/'generated/insertion_envelopes');bound=p.read(HERE/'reviews/survey.json')['wrist_carrier_rotation_bound'];envelope=cq.Solid.makeCylinder(bound['radius_mm'],110,cq.Vector(32,-65,-60),cq.Vector(0,0,1));sleeves=[]
    for i,z in enumerate([-3.,2.]):
        shell=a.cyl(100,z,-100,-4,2.5).cut(a.cyl(100,z,-100.01,-3.99,1.6));sleeves.append(dict(name='evidence_sleeve_only_'+str(i),shape=shell))
    rotation=c.material(sleeves,[dict(name='two_wrist_parts_R60_bound',shape=envelope)])
    tests=[z['native'] for v in r['paths'] for z in v['samples']]
    r.update(completed=True,required_sampled_hardware_paths_clear=all(m.drive.clear(z) for z in tests),samples=len(tests),pairs_in_scope=sum(z['pairs_in_scope'] for z in tests),AABB_native_pairs=sum(z['exact_pairs'] for z in tests),overlap_records=sum(len(z['overlaps']) for z in tests),operation_failures=sum(len(z['operation_failures']) for z in tests),contacts=contacts,straight_end_insertion=insertion,insertion_envelope_scene=envelope_scene,straight_end_continuous_axial_stroke_clear=all(m.drive.clear(z['native']) and max(z['start_end_material_outside_envelope_mm3'])<1e-8 for z in insertion),sleeve_vs_wrist_carrier_rotation_bound=rotation,sleeve_bound_not_whole_bracket_rotation_approval=True,actual_flexible_housing_bending_friction_pullout_lifetime_or_strength_verified=False)
    r['source_sha256']['cad/wrist_external_feed_v1/feed_service.py']=m.drive.sha(Path(__file__));p.write(out,r)
if __name__=='__main__':main()
