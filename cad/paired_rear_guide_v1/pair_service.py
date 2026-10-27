"""Actual hardware entry and a prescribed feed interval inside real channels."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import check_pair as b
a,m,g,c,p,np,cq,ROOT,HERE=b.a,b.m,b.g,b.c,b.p,b.np,b.cq,b.ROOT,b.HERE
from OCP.OSD import OSD_ThreadPool

def sequence():return [('wrist_paired_rear_S_guide',[-20,-10,-5,-1,0]),('wrist_edge_clip_front_cap',[12,6,3,1,0])]+[('wrist_edge_clip_bolt_'+str(i),[20,10,5,1,0]) for i in [0,1]]+[('wrist_edge_clip_nut_'+str(i),[-20,-10,-5,-1,0]) for i in [0,1]]

def main():
    out=HERE/'reviews/service.json';assert not out.exists();static=p.read(HERE/'reviews/static.json');assert static['completed'] and static['sampled_candidate_clear'];data,pack,guide=b.base.inputs();hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,np.zeros(20));fixed=hand+pack+data[6]+data[7];scene=p.read(HERE/'generated/assembly_scene.json');rows=c.load(scene['new_parts']+scene['retained_parts'],data[0]);hardware=[z for z in rows if z['installed_part_candidate']];installed=[];r=dict(completed=False,source_sha256=data[0],paths=[],feeding_samples=[],OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,hand_angles_deg=[0.]*20,wrist_flex_deviation_deg=[0,0],local_references_absent_during_hardware_assembly=True,manual_holding_until_clamp_closed=True,actual_tools_jigs_threads_torque_strength_verified=False,whole_link_installed_or_cord_threading_approved=False)
    for name,offsets in sequence():
        part=next(z for z in hardware if z['name']==name);samples=[dict(translation_world_mm=[0,dy,0],native=c.material([dict(part,shape=part['shape'].translate((0,dy,0)))],fixed+installed)) for dy in offsets];got=dict(name=name,installed_before=[z['name'] for z in installed],absent_hardware=[z['name'] for z in hardware if z['name'] not in [v['name'] for v in installed]+[name]],samples=samples,clear=all(m.drive.clear(z['native']) for z in samples));r['paths'].append(got);p.write(out,r);print('assembly',name,got['clear'],flush=True)
        if not got['clear']:r.update(completed=True,required_hardware_paths_clear=False,feeding_not_executed=True);p.write(out,r);return
        installed.append(part)
    L=static['design']['centerline_length_mm'];tips=[0.,20.,40.,60.,80.,100.,130.,160.,L,L+10.];envelopes=[a.window(i,L+10,L+230) for i in [0,1]]
    for i,z in enumerate(envelopes):z['name']='prescribed_feed_union_envelope_'+str(i);z['evidence_only']=True
    envtests=[c.material([z],fixed+hardware) for z in envelopes];envpair=c.material([envelopes[0]],[envelopes[1]]);envscene=b.export(envelopes,HERE/'generated/feed_envelopes')
    for tip in tips:
        refs=[a.window(i,tip) for i in [0,1]];tests=[c.material([z],fixed+hardware) for z in refs];pair=c.material([refs[0]],[refs[1]]);outside=[]
        for z,env in zip(refs,envelopes):sh=z['shape'].cut(env['shape'],tol=1e-6);outside.append(sh.Volume() if sh.Solids() else 0.)
        r['feeding_samples'].append(dict(tip_material_coordinate_mm=tip,reference_length_mm=220.,native=tests,between_lines=pair,material_outside_union_envelope_mm3=outside,clear=all(m.drive.clear(v) for v in tests+[pair])));p.write(out,r);print('feed',tip,[len(v['overlaps']) for v in tests],len(pair['overlaps']),outside,flush=True)
    contacts=[];clip=hardware[0]['shape'];cap=next(z['shape'] for z in hardware if z['name']=='wrist_edge_clip_front_cap');saddle=next(z['shape'] for z in data[7] if z['name']=='Palm_wrist_saddle')
    for name,point,normal,first in [('rear_face',[50,-6,-52],[0,1,0],clip),('front_face',[50,0,-52],[0,-1,0],cap),('outer_edge',[54,-3,-52],[-1,0,0],clip)]:
        pt=np.asarray(point);n=np.asarray(normal);contacts.append(dict(id=name,point_mm=point,normal=normal,distance_mm=[sh.distance(cq.Vertex.makeVertex(*pt)) for sh in [first,saddle]],material=[[sh.isInside(cq.Vector(*(pt+s*.05*n)),1e-6) for s in [-1,1]] for sh in [first,saddle]]))
    tests=[z['native'] for v in r['paths'] for z in v['samples']];feedtests=[t for z in r['feeding_samples'] for t in z['native']+[z['between_lines']]]
    r.update(completed=True,required_hardware_paths_clear=all(m.drive.clear(z) for z in tests),hardware_finite_samples=len(tests),hardware_pairs_in_scope=sum(z['pairs_in_scope'] for z in tests),hardware_AABB_native_pairs=sum(z['exact_pairs'] for z in tests),hardware_overlap_records=sum(len(z['overlaps']) for z in tests),hardware_operation_failures=sum(len(z['operation_failures']) for z in tests),feeding_samples_completed=len(tips),feeding_reference_parts_per_sample=2,feeding_pairs_in_scope=sum(z['pairs_in_scope'] for z in feedtests),feeding_AABB_native_pairs=sum(z['exact_pairs'] for z in feedtests),feeding_overlap_records=sum(len(z['overlaps']) for z in feedtests),feeding_operation_failures=sum(len(z['operation_failures']) for z in feedtests),feed_union_envelope_scene=envscene,feed_union_envelope_native=envtests,between_union_envelopes=envpair,contacts=contacts,prescribed_feed_interval_mm=[0.,L+10.],prescribed_feed_union_clear=all(m.drive.clear(v) for v in envtests+[envpair]),feed_centerline_enforced_by_actual_channels=True,geometry_inclusion_not_friction_or_elastic_feeding_equilibrium=True,whole_flexible_housing_or_cord_threading_verified=False)
    r['source_sha256']['cad/paired_rear_guide_v1/pair_service.py']=m.drive.sha(Path(__file__));r['input_hash_guard']=all(m.drive.sha(ROOT/z)==h for z,h in r['source_sha256'].items());p.write(out,r)
if __name__=='__main__':main()
