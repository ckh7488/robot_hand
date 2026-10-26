import sys,time,math
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import build_check as check
import guide_candidate3 as g
check.g=g;check.HERE=g.HERE
c,b,m,p,cq,np,HERE,ROOT=g.c,g.b,g.m,g.p,g.cq,g.np,g.HERE,g.ROOT
from OCP.OSD import OSD_ThreadPool

def main():
    out=HERE/'reviews/service.json';assert not out.exists();start=time.monotonic()
    report=p.read(HERE/'reviews/check.json');assert report['completed'] and report['all_nominal_snapshots_clear']
    parts,design=g.build();data=check.inputs(design);source=data[0];source['cad/thumb_root_guide_v1/guide_service.py']=g.g.drive.sha(Path(__file__))
    source['cad/thumb_root_guide_v1/selected2/reviews/check.json']=g.g.drive.sha(HERE/'reviews/check.json')
    record=dict(completed=False,source_sha256=source,paths=[],contacts=[],constraints=[],OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),fixture_complete=False,whole_hand_drive_complete=False)
    def save():p.write(out,record)
    by={z['name']:z for z in parts};v=data[1];cat=data[2];handmeta=data[3]['hand'];hand=data[4];neutral=v.frames(cat['design'],np.zeros(20))
    def world_local(q,local):
        F=v.frames(cat['design'],q)['thumb_mcp'];base=neutral['thumb_mcp']
        return v.relocate(g.original.transform(local,g.original.WORLD),F,base)
    def path(label,names,transforms,installed,q,scope):
        current=v.frames(cat['design'],q)
        obstacles=[dict(name=z['name'],shape=v.relocate(hand[z['name']],current[z['body']],neutral[z['body']])) for z in handmeta if z['name'] not in design['replaced_hand_parts']]
        obstacles+=data[5]+data[6]+data[7]+[dict(name=n,shape=world_local(q,by[n]['local_shape'])) for n in installed if n not in names]
        samples=[]
        for T in transforms:
            moving=[dict(name=n,shape=world_local(q,g.original.transform(by[n]['local_shape'],T))) for n in names]
            samples.append(dict(transform_local=T.tolist(),native=c.material(moving,obstacles)))
        got=dict(id=label,moving=names,already_installed_new_parts=installed,hand_angles_deg=list(map(float,q)),scope=scope,samples=samples,clear=all(g.g.drive.clear(z['native']) for z in samples))
        record['paths'].append(got);save();print(label,got['clear'],flush=True);return got
    def translation(vector):
        T=np.eye(4);T[:3,3]=vector;return T
    q0=np.zeros(20);carrier='paired_liner_carrier';face=['guide_cover_M2x12_1','guide_cover_M2x12_2']
    path('carrier_side_entry',[carrier],[translation([x,0,0]) for x in [35,25,15,5,0]],[],q0,'Original two cover bolts removed;skin/washers manually held.Retaining caps/liners and their hardware not yet fitted.No physical holding fixture verified.')
    installed=[carrier]
    for name in face:
        path(name+'_insert',[name],[translation([0,0,z]) for z in [8,6,4,2,0]],installed,q0,'Original captive nut and profiled washer retained;nominal longer shaft keeps original tip.No supplier thread/tool/preload approval.')
        installed.append(name)
    for index,ip in [(1,-15.),(0,85.)]:
        q=q0.copy();q[19]=ip;row=g.original.layout()[index];Q=row['socket_transform'];axis=Q[:3,1];name='flanged_liner_'+str(index)
        path(name+'_axial_insert',[name],[translation(axis*x) for x in [20,15,10,5,0]],installed,q,'Different manual IP pose exposes each side;cords not threaded yet.Guide tube/collar is one nominal component;forming/strength/supply dimensions unverified.')
        installed.append(name)
        cap='liner_flange_keeper_'+str(index)
        if index==1:
            path('initial_upper_keeper_straight_side',[cap],[translation([x,0,0]) for x in [12,9,6,3,0]],installed,q,'Original simple side sweep;preserved even if side-bridge obstruction occurs.Not a proof the assembly is impossible.')
        transforms=[translation(np.array([x,0,0])+axis*6) for x in [12,9,6,3,0]]+[translation(axis*y) for y in [4.5,3,1.5,0]]
        path(cap+'_forward_then_side_then_seat',[cap],transforms,installed,q,'Keeper is6mm in front of its seat during side approach,then seats axially.U-slot accepts the straight tube before the bend starts.Manual retention until bolt is fitted.')
        installed.append(cap)
        nut='liner_keeper_M2_nut_'+str(index);bolt='liner_keeper_M2x10_'+str(index)
        path(nut+'_insert',[nut],[translation(-axis*x) for x in [8,6,4,2,0]],installed,q,'Rear nut must be held until the bolt engages.No physical tool/fixture verified.')
        installed.append(nut)
        path(bolt+'_insert',[bolt],[translation(axis*x) for x in [8,6,4,2,0]],installed,q,'Nominal smooth bolt/nut envelopes with no thread mechanics.Thread engagement and driver/hand access unverified.')
        installed.append(bolt)
    # Nominal load-path evidence, not friction/preload or material strength.
    for seat in design['seats']:
        washer=g.original.local_hand()[0][seat['washer']];s=by[carrier]['local_shape'];z=seat['bottom_z_mm'];points=[]
        for angle in [0,math.pi/2,math.pi,3*math.pi/2]:
            point=np.array([seat['x_mm']+1.7*math.cos(angle),seat['y_mm']+1.7*math.sin(angle),z]);a=point+np.array([0,0,.02]);d=point-np.array([0,0,.02])
            points.append(dict(point_mm=point.tolist(),carrier_above=bool(s.isInside(cq.Vector(*a),1e-6)),carrier_below=bool(s.isInside(cq.Vector(*d),1e-6)),washer_above=bool(washer.isInside(cq.Vector(*a),1e-6)),washer_below=bool(washer.isInside(cq.Vector(*d),1e-6))))
        record['contacts'].append(dict(id=seat['washer'],distance_mm=s.distance(washer),classification=points))
    for row in g.original.layout():
        Q=row['socket_transform'];i=row['cord'];liner=by['flanged_liner_'+str(i)]['local_shape'];cap=by['liner_flange_keeper_'+str(i)]['local_shape'];base=by[carrier]['local_shape'];checks=[]
        for plane in [0.,2.]:
            for angle in [math.pi/4,math.pi/2,3*math.pi/2,7*math.pi/4]:
                x,z=math.cos(angle),math.sin(angle);point=(Q@np.array([x,plane,z,1]))[:3];a=point+.02*Q[:3,1];d=point-.02*Q[:3,1]
                checks.append(dict(plane_y_mm=plane,point_mm=point.tolist(),liner_above=bool(liner.isInside(cq.Vector(*a),1e-6)),liner_below=bool(liner.isInside(cq.Vector(*d),1e-6)),support_above=bool((base if plane==0 else cap).isInside(cq.Vector(*a),1e-6)),support_below=bool((base if plane==0 else cap).isInside(cq.Vector(*d),1e-6))))
        record['contacts'].append(dict(id='flange_'+str(i),rear_seat_distance_mm=liner.distance(base),front_keeper_distance_mm=liner.distance(cap),classification=checks,axial_capture_nominal_only=True))
    foot_ok=all(z['distance_mm']<1e-5 and all([a['carrier_above'],a['carrier_below'],a['washer_above'],a['washer_below']]==[True,False,False,True] for a in z['classification']) for z in record['contacts'] if 'distance_mm' in z)
    flange_ok=all(z['rear_seat_distance_mm']<1e-5 and z['front_keeper_distance_mm']<1e-5 and all([a['liner_above'],a['liner_below'],a['support_above'],a['support_below']]==([True,False,False,True] if a['plane_y_mm']==0 else [False,True,True,False]) for a in z['classification']) for z in record['contacts'] if 'rear_seat_distance_mm' in z)
    record['nominal_contacts_verified']=foot_ok and flange_ok
    target=next(z for z in p.read(ROOT/'cad/palm_motor_layout_v1/reviews/check2.json')['cases'] if z['id']=='pinch25')['angles_deg']
    lengths=[]
    for fraction in [i/16 for i in range(17)]:
        q=np.asarray(target)*fraction;q[19]=0.;cords=g.cord_pieces(q)
        lengths.append(dict(fraction=fraction,parent_angles_deg=q.tolist(),local_lengths_mm=[sum(z['length_mm'] for z in cords if z['cord']==i) for i in range(2)]))
    maximum=max(abs(np.asarray(z['local_lengths_mm'])-lengths[0]['local_lengths_mm']).max() for z in lengths)
    record['constraints']=dict(guide_centers_fixed_body='thumb_mcp',guides_are_rigid_parts=True,guide_body_center_drift_mm=0.,parent_only_distal_length_study=lengths,parent_only_local_length_max_change_mm=float(maximum),scope='Only terminal-to-IP cord length,not motor-to-terminal/Bowden length or tension.Different guide bodies are not assumed to maintain the former full free-curve lengths.')
    required=[z for z in record['paths'] if z['id']!='initial_upper_keeper_straight_side']
    record.update(completed=True,required_sampled_paths_clear=all(z['clear'] for z in required),required_samples=sum(len(z['samples']) for z in required),initial_cap_sweep_clear=next(z for z in record['paths'] if z['id']=='initial_upper_keeper_straight_side')['clear'],input_hash_guard=all(g.g.drive.sha(ROOT/z)==h for z,h in source.items()),elapsed_s=time.monotonic()-start)
    save();assert record['input_hash_guard'];print('service done',record['required_sampled_paths_clear'],record['required_samples'],flush=True)
if __name__=='__main__':main()
