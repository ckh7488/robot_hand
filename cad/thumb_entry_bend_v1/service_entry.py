"""Fresh bench/hand sequence for the new jaw and a prescribed housing end feed."""
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import check_entry as b
a,m,g,c,p,np,cq,ROOT,HERE=b.a,b.m,b.g,b.c,b.p,b.np,b.cq,b.ROOT,b.HERE
from OCP.OSD import OSD_ThreadPool
def main():
    out=HERE/'reviews/service.json';assert not out.exists();static=p.read(HERE/'reviews/static.json');assert static['completed'] and static['sampled_candidate_clear'];data,pack,guide,wrist=b.inputs();hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,np.zeros(20));new=c.load(p.read(HERE/'generated/parts/scene.json')['parts'],data[0]);body=next(z for z in new if z['installed_part_candidate']);hand.append(body);assert len(hand)==397;by={z['name']:z for z in hand};tag='distal_housing_clamp_1';kit=[z['name'] for z in hand if z['name'].startswith(tag+'_')]+[body['name'],'distal_flanged_ferrule_1_captured','distal_housing_shoe_1'];assert len(kit)==8
    axis=a.frame()[:3,2];benchfixed=[dict(by[n],shape=by[n]['shape'].translate((500,0,0))) for n in kit[-2:]];installed=[];r=dict(completed=False,source_sha256=data[0],bench_paths=[],hand_paths=[],feeding_samples=[],kit_names=kit,bench_translation_world_mm=[500,0,0],hand_angles_deg=[0.]*20,wrist_flex_deviation_deg=[0,0],manual_holding_until_bolts_closed=True,actual_jigs_tools_thread_torque_force_verified=False,whole_link_or_cord_threading_approved=False,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6)
    seq=[(tag+'_jaw_-1',-axis,[14,7,2,0]),(body['name'],axis,[14,7,2,0])]+[(tag+'_nut_'+str(j),-axis,[10,5,0]) for j in [0,1]]+[(tag+'_bolt_'+str(j),axis,[10,5,0]) for j in [0,1]]
    for name,direction,ds in seq:
        samples=[]
        for d in ds:
            delta=np.array([500.,0,0])+direction*d;z=by[name];samples.append(dict(translation_world_mm=delta.tolist(),native=c.material([dict(z,shape=z['shape'].translate(tuple(delta)))],benchfixed+[dict(by[n],shape=by[n]['shape'].translate((500,0,0))) for n in installed])))
        got=dict(moving=[name],installed_before=list(installed),fixed_bench=kit[-2:],samples=samples,clear=all(m.drive.clear(v['native']) for v in samples));r['bench_paths'].append(got);p.write(out,r);print('bench',name,got['clear'],flush=True)
        if not got['clear']:r.update(completed=True,bench_clear=False,hand_not_executed=True);p.write(out,r);return
        installed.append(name)
    omitted=['distal_housing_keeper_1','liner_keeper_M2x10_1','liner_keeper_M2_nut_1','distal_shoe_M2x10_1_0','distal_shoe_M2x10_1_1','distal_shoe_M2_nut_1_0','distal_shoe_M2_nut_1_1'];assert all(n in by for n in omitted);fixed=[z for z in hand if z['name'] not in kit+omitted]+pack+data[6]+data[7]+wrist;installed=[];Q=g.original.layout()[1]['socket_transform'];wx=g.original.WORLD[:3,:3]@Q[:3,0];wz=g.original.WORLD[:3,:3]@Q[:3,2]
    seq=[('upper_empty_kit',kit,wx,[20,10,5,1,0]),('upper_keeper_after_kit',['distal_housing_keeper_1'],wz,[14,7,3,1,.5,0]),('original_nut',['liner_keeper_M2_nut_1'],wx,[-12,-6,0]),('original_bolt',['liner_keeper_M2x10_1'],wz,[8,4,0])]+[('new_nut_'+str(j),['distal_shoe_M2_nut_1_'+str(j)],wx,[12,6,0]) for j in [0,1]]+[('new_bolt_'+str(j),['distal_shoe_M2x10_1_'+str(j)],wz,[8,4,0]) for j in [0,1]]
    for label,names,direction,ds in seq:
        samples=[]
        for d in ds:
            delta=direction*d;samples.append(dict(translation_world_mm=delta.tolist(),native=c.material([dict(by[n],shape=by[n]['shape'].translate(tuple(delta))) for n in names],fixed+[by[n] for n in installed])))
        got=dict(id=label,moving=names,installed_before=list(installed),absent_hand_parts=[n for n in kit+omitted if n not in installed+names],samples=samples,clear=all(m.drive.clear(v['native']) for v in samples));r['hand_paths'].append(got);p.write(out,r);print('hand',label,got['clear'],flush=True)
        if not got['clear']:r.update(completed=True,bench_clear=True,hand_clear=False,feeding_not_executed=True);p.write(out,r);return
        installed+=names
    assert set(installed)==set(kit+omitted);L=a.LEAD+np.pi*a.RADIUS/2;tips=[0.,12.,22.,32.,L,L+11.,L+24.,L+25.];allfixed=hand+pack+data[6]+data[7]+wrist
    for tip in tips:
        ref=a.feed_window(tip);test=c.material([ref],allfixed);r['feeding_samples'].append(dict(tip_material_coordinate_mm=tip,reference_length_mm=100.,native=test,clear=m.drive.clear(test)));p.write(out,r);print('feed',tip,test['overlaps'],flush=True)
        if not m.drive.clear(test):break
    ts=[s['native'] for path in r['bench_paths']+r['hand_paths'] for s in path['samples']];fs=[s['native'] for s in r['feeding_samples']]
    r.update(completed=True,bench_clear=True,hand_clear=True,bench_finite_samples=sum(len(z['samples']) for z in r['bench_paths']),hand_finite_samples=sum(len(z['samples']) for z in r['hand_paths']),assembly_pairs_in_scope=sum(z['pairs_in_scope'] for z in ts),assembly_AABB_native_pairs=sum(z['exact_pairs'] for z in ts),assembly_overlap_records=sum(len(z['overlaps']) for z in ts),assembly_operation_failures=sum(len(z['operation_failures']) for z in ts),feeding_samples_completed=len(fs),feeding_clear=len(fs)==8 and all(m.drive.clear(z) for z in fs),feeding_pairs_in_scope=sum(z['pairs_in_scope'] for z in fs),feeding_AABB_native_pairs=sum(z['exact_pairs'] for z in fs),feeding_overlap_records=sum(len(z['overlaps']) for z in fs),feeding_operation_failures=sum(len(z['operation_failures']) for z in fs),final_hand_configuration_restored=True,prescribed_feeding_not_elastic_or_friction_solution=True,whole_link_installed=False)
    r['source_sha256']['cad/thumb_entry_bend_v1/service_entry.py']=m.drive.sha(Path(__file__));r['input_hash_guard']=all(m.drive.sha(ROOT/z)==h for z,h in r['source_sha256'].items());p.write(out,r)
if __name__=='__main__':main()
