"""Bring each unchanged M2 nut from above/below, then seat along its real axis."""
import importlib.util,sys,time
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm077_check',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,d,ROOT,HERE=m.n,m.np,m.d,m.ROOT,m.HERE
from OCP.OSD import OSD_ThreadPool


def main():
    out=HERE/'reviews/check.json';assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    start=time.monotonic();rows,_,source,packets=m.context();lookup={z['name']:z for z in rows};final=set(lookup);parts=m.split_nuts(lookup);lookup.update({z['name']:z for z in parts});old=n.p.read(ROOT/'cad/preloaded_rack_nut_module_v1/reviews/selected_check.json');survey=n.p.read(HERE/'reviews/survey.json');world=set(old['packet_final_present_names']);placed=set();assert len(world)==717 and m.BASE not in world
    source[Path(__file__).relative_to(ROOT).as_posix()]=n.m.drive.sha(Path(__file__))
    raw=dict(completed=False,source_sha256=source,native={},paths=[],original16packets=packets,original_special_packet=old['original_special_packet'],mapped_special_packet=old['mapped_special_packet'],explicit_replacement={m.OLD:m.NAME},final886_names=sorted(final),initial_world717_names=sorted(world),preserved_previous_world717_names=old['packet_final_present_names'],restored_before=old['restored_modules'],new_restored_modules=[],original_base_compound=m.BASE,original_nut_solid_count=4,nut_solids=[dict(name=z['name'],source_compound=m.BASE,source_solid_index=z['source_solid_index'],bbox=n.c.m.pack._bbox(z['shape']),volume_mm3=z['shape'].Volume()) for z in parts],initial_completed_solid_names=[],old_native_groups_repeated=0,old_common_Boolean_repeated=0,inherited_successful_prefix_repeated=0,new_hardware_geometry=0,current862_replaced=False,proposal886_adopted=False,old_M3_cut_or216points_repeated=0,preserved_upper_M3_whole_cut=old['preserved_upper_M3_whole_cut'],reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,OCC_pool_threads=1,
        scope='Each original solid is independently moved against actual717 and every already seated originalnut. Originalcompound row is absent during partial placement,not also fixed. Original geometry/axes/locations unchanged. After all4 identity poses,compact exact originalsolids into originalcompound row718,not four added hardware rows. Fulloutside +/-Z approach with X4 then original-X axial seating;4mm is a test route,not minimumclearance. Projected AABB entry/mid/exit event samples augment20mm stepping. Manualholding/tools/threads/torque/friction/preload/captive and continuous path unverified. Previous714/717 preserved;old106/162/9railfaces/7events/M3false216/615PCD not rerun.')
    bounds={k:np.array(n.c.m.pack._bbox(z['shape'])) for k,z in lookup.items()}
    def save():n.a.save(out,raw)
    for part in parts:
        name=part['name'];idx=part['source_solid_index'];fixednames=world|placed;fixed=[lookup[k] for k in sorted(fixednames)];bb=bounds[name];axis=np.array([0.,0.,1. if (bb[0,2]+bb[1,2])>0 else -1.]);offset=np.array([4.,0.,0.]);outside=m.outside_start([part],fixed,axis,offset);events=[];candidates=[];mb=bb+offset
        for k in sorted(fixednames):
            fb=bounds[k]
            if mb[1,0]<fb[0,0] or fb[1,0]<mb[0,0] or mb[1,1]<fb[0,1] or fb[1,1]<mb[0,1]:continue
            moving_proj=sorted([axis[2]*mb[0,2],axis[2]*mb[1,2]]);fixed_proj=sorted([axis[2]*fb[0,2],axis[2]*fb[1,2]]);lo=fixed_proj[0]-moving_proj[1];hi=fixed_proj[1]-moving_proj[0];selected=[float(x) for x in [lo,(lo+hi)/2.,hi] if 0<x<outside];events+=selected;candidates.append(dict(name=k,bbox=fb,projection_translation_interval_mm=[lo,hi],selected_event_samples_mm=selected))
        levels=sorted(set([float(x) for x in m.inward_steps(outside)]+[10.,5.,0.]+[round(x,9) for x in events]),reverse=True)
        states=[dict(phase='outside_then_side_alignment',translation_mm=offset+x*axis,phase_boundary=i==0 or i==len(levels)-1 or any(abs(x-v)<1e-7 for v in events)) for i,x in enumerate(levels)]
        states += [dict(phase='original_axis_seating',translation_mm=np.array([x,0.,0.]),phase_boundary=True) for x in [2.,1.,0.]]
        path=dict(id='individual_M2_nut_'+str(idx),moving_names=[name],native_present_names=sorted(fixednames),canonical_present_names=sorted(world),canonical_absent_names=sorted(final-world),completed_original_solid_names=sorted(placed),absent_original_solid_names=sorted({z['name'] for z in parts}-placed),original_compound_absent_from_fixed=m.BASE not in fixednames,approach_axis=axis,prealignment_X_translation_mm=4.,projected_obstacle_events=candidates,samples=[],completed=False);raw['paths'].append(path);save();blocked=False
        for i,state in enumerate(states):
            M=m.translate(state['translation_mm']);key=path['id']+'_'+str(i);moving=m.moved([part],M);q=n.c.material(moving,fixed);raw['native'][key]=q;path['samples'].append(dict(id=key,transform=M,**state))
            if i==0:
                mbp=m.project_bounds(moving,axis);fbp=m.project_bounds(fixed,axis);path['initial_projection_separation_mm']=max(fbp[0]-mbp[1],mbp[0]-fbp[1]);assert path['initial_projection_separation_mm']>0
            save();print(key,state['phase'],state['translation_mm'].tolist(),q['pairs_in_scope'],q['exact_pairs'],len(q['overlaps']),flush=True)
            if q['overlaps'] or q['operation_failures']:raw['first_blocked_sample']=key;raw['first_blocked_path']=path['id'];blocked=True;break
        if blocked:break
        path['completed']=True;placed.add(name);save()
    done=len(placed)==4
    if done:
        assert m.BASE not in world and placed=={z['name'] for z in parts};world.add(m.BASE);raw['new_restored_modules']=['thumb_ip_actuator'];raw['compound_restoration']=dict(original_name=m.BASE,source_solid_indices=[0,1,2,3],original_source_compound_reused=True,completed_solids_at_original_identity=True,partial_virtual_solids_removed_from_native_inventory=True,new_hardware_rows=0)
    total=d.k.flat(list(raw['native'].values()));raw.update(completed=True,new_native_groups=len(raw['native']),pairs_in_scope=total['pairs_in_scope'],AABB_native_pairs=total['exact_pairs'],overlap_records=len(total['overlaps']),operation_failures=len(total['operation_failures']),raw_new_tests_clear=not total['overlaps'] and not total['operation_failures'],final_completed_solid_names=sorted(placed),original_base_compound_restored=done,mapped_special48_restoration_completed=done,packet_final_present_names=sorted(world),packet_final_pending_names=sorted(final-world),restored_modules=old['restored_modules']+raw['new_restored_modules'],input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()),elapsed_s=time.monotonic()-start)
    save();print('complete',raw['new_native_groups'],'records',raw['overlap_records'],'nuts',len(placed),'compound',done,'world',len(world),flush=True)


if __name__=='__main__':main()
