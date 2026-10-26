"""Assembly with all deliberately absent hand parts restored in explicit paths."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from probe import *
from material_cache import install
install(c)
from OCP.OSD import OSD_ThreadPool

def main():
    out=HERE/'reviews/final_service.json';assert not out.exists();start=time.monotonic()
    prior=p.read(HERE/'reviews/service.json');assert not prior['completed']
    assert all(g.g.drive.sha(ROOT/z)==h for z,h in prior['source_sha256'].items())
    prior_paths={z['id']:z for z in prior['paths']}
    static=p.read(HERE/'reviews/static.json');assert static['completed'] and static['all_nominal_snapshots_clear']
    design=static['design'];parts=c.load(p.read(ROOT/static['scene'])['parts']);by={z['name']:dict(z,local_shape=g.original.transform(z['shape'],g.original.INV)) for z in parts}
    data=check.inputs(design);v,cat,scene,hand=data[1:5];neutral=v.frames(cat['design'],np.zeros(20))
    data[0]['cad/thumb_guide_assembly_v2/service.py']=g.g.drive.sha(HERE/'service.py');data[0]['cad/thumb_guide_assembly_v2/service_resume.py']=g.g.drive.sha(Path(__file__));data[0]['cad/thumb_guide_assembly_v2/material_cache.py']=g.g.drive.sha(HERE/'material_cache.py');data[0]['cad/thumb_guide_assembly_v2/reviews/static.json']=g.g.drive.sha(HERE/'reviews/static.json')
    for z in parts:data[0][z['step']]=z['sha256_step']
    child=[z['name'] for z in scene['hand'] if z['body']=='thumb_ip'];thrust=['thumb_distal2_'+s+'_thrust_washer' for s in ['left','right']]
    hardware=['thumb_joint_pin_2']+['thumb_distal2_'+z for z in ['left_pin_washer','right_pin_washer','left_thrust_washer','right_thrust_washer','retaining_washer','retaining_screw']]
    missing=child+hardware;installed=[];q=np.zeros(20)
    record=dict(completed=False,source_sha256=data[0],paths=[],roll_probes=[],child_module=child,joint_hardware=hardware,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,physical_fixture_verified=False,full_hand_drive_complete=False)
    def save():p.write(out,record)
    def world_local(s):return g.original.transform(s,g.original.WORLD)
    def path(label,names,Ts,scope):
        absent=list(missing);previous=list(installed)
        obstacles=[dict(name=z['name'],shape=hand[z['name']]) for z in scene['hand'] if z['name'] not in missing+design['replaced_hand_parts']+names]
        obstacles+=data[5]+data[6]+data[7]+[dict(name=n,shape=by[n]['shape']) for n in installed if n not in names]
        if label in prior_paths:
            got=prior_paths[label]
            assert got['moving']==list(names) and got['missing_hand_parts']==absent and got['installed_new']==previous and got['hand_angles_deg']==q.tolist()
            assert len(Ts)==len(got['samples']) and all(np.max(np.abs(np.asarray(z['transform_local'])-T))<1e-12 and z['native']['pairs_in_scope']==len(names)*len(obstacles) for z,T in zip(got['samples'],Ts))
            got=dict(got,reused_from='reviews/service.json');record['paths'].append(got);save();print(label,'same completed result reused',flush=True);return got
        rows=[]
        for T in Ts:
            moving=[]
            for n in names:
                s=by[n]['shape'] if n in by else hand[n]
                moving.append(dict(name=n,shape=g.original.transform(s,g.original.WORLD@T@g.original.INV)))
            rows.append(dict(transform_local=T.tolist(),native=c.material(moving,obstacles)))
        got=dict(id=label,moving=list(names),missing_hand_parts=absent,installed_new=previous,hand_angles_deg=q.tolist(),scope=scope,samples=rows,clear=all(g.g.drive.clear(z['native']) for z in rows));record['paths'].append(got);save();print(label,got['clear'],flush=True);return got
    path('carrier_before_tip',['paired_liner_carrier'],[translate([x,0,0]) for x in [35,25,15,5,0]],'Child13 and seven distal pin-stack parts are not installed;cover and washers manually supported.Existing source hand parts are not cut.')
    installed.append('paired_liner_carrier')
    for name in ['guide_cover_M2x12_1','guide_cover_M2x12_2']:
        path(name,[name],[translate([0,0,z]) for z in [8,6,4,2,0]],'Retained skin/profiled washers/captive nuts in their nominal seats;longer bolt envelope only.No physical jig/thread qualification.')
        installed.append(name)
    for row in g.original.layout()[::-1]:
        i=row['cord'];direction=row['socket_transform'][:3,2]*row['side']
        for name,travel in [('flanged_liner_'+str(i),[20,15,10,5,0]),('liner_flange_keeper_'+str(i),[14,10,6,3,0]),('liner_keeper_M2_nut_'+str(i),[-8,-6,-4,-2,0]),('liner_keeper_M2x10_'+str(i),[8,6,4,2,0])]:
            path(name+'_radial',[name],[translate(direction*d) for d in travel],'Radial outside approach into open half socket.Cords not yet threaded.Child/pin absent.Manual retention until nominal nut/bolt engaged;physical tools/preload unverified.')
            installed.append(name)
    names=child+thrust
    path('restore_child_and_inner_thrust_washers',names,[translate([0,y,0]) for y in [40,30,20,10,5,2,0]],'Distal link/cap/bushings/tendon-clamp are a preassembled13part child reference;two inner thrust washers manually held with it.The seven remaining parent pin-stack rows are not silently assumed installed.No claim vendor subassembly/washer holding fixture is verified.')
    for n in names:missing.remove(n)
    for side,sign in [('left',-1),('right',1)]:
        name='thumb_distal2_'+side+'_pin_washer'
        path('restore_'+name,[name],[translate([sign*x,0,0]) for x in [10,7.5,5,2.5,0]],'Outer washer positioned before the pin;held at its seat pending pin/retainer assembly.')
        missing.remove(name)
    path('restore_pin',['thumb_joint_pin_2'],[translate([x,0,0]) for x in [-35,-25,-15,-10,-5,0]],'Left-side pin insertion through retained parent bushings,newly restored child and washers;retaining washer/screw installed later.')
    missing.remove('thumb_joint_pin_2')
    for name,travel in [('thumb_distal2_retaining_washer',[10,7.5,5,2.5,0]),('thumb_distal2_retaining_screw',[8,6,4,2,0])]:
        path('restore_'+name,[name],[translate([x,0,0]) for x in travel],'Right-side original keeper/screw envelope approach;helical engagement/tool/physical torque unverified.')
        missing.remove(name)
    assert not missing
    for row in g.original.layout():
        name='flanged_liner_'+str(row['cord']);Q=row['socket_transform']
        others=[dict(name=n,shape=z['local_shape']) for n,z in by.items() if n!=name]
        for a in [-5.,0.,5.]:
            T=rotate(Q[:3,3],Q[:3,1],a);got=c.material([dict(name=name,shape=g.original.transform(by[name]['local_shape'],T))],others)
            tab=g.original.transform(g.original.box([1.6,0,-.5],[2.6,2,.5]),Q)
            tab_test=c.material([dict(name=name+'_tab_evidence_only',shape=g.original.transform(tab,T))],others)
            record['roll_probes'].append(dict(id=name+'_'+str(a),angle_deg=a,transform_local=T.tolist(),native=got,tab_only_native=tab_test))
        save();print('indexed roll',name,flush=True)
    record.update(completed=True,new_required_samples=sum(len(z['samples']) for z in record['paths'] if 'reused_from' not in z),same_required_samples_reused=sum(len(z['samples']) for z in record['paths'] if 'reused_from' in z),required_samples=sum(len(z['samples']) for z in record['paths']),required_sampled_paths_clear=all(z['clear'] for z in record['paths']),all_missing_hand_parts_restored=True,
        bilateral_5degree_index_stop=all((len(z['native']['overlaps'])>0 and len(z['tab_only_native']['overlaps'])>0) if z['angle_deg'] else (g.g.drive.clear(z['native']) and g.g.drive.clear(z['tab_only_native'])) for z in record['roll_probes']),input_hash_guard=all(g.g.drive.sha(ROOT/z)==h for z,h in data[0].items()),elapsed_s=time.monotonic()-start)
    save();assert record['input_hash_guard']
if __name__=='__main__':main()
