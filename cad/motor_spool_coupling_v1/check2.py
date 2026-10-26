"""Correct the new drum only; prove its complete rotational space envelope."""
import sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import candidate2 as g
from OCP.OSD import OSD_ThreadPool
c,b,m,p,cq,np,HERE,ROOT=g.c,g.b,g.m,g.p,g.cq,g.np,g.HERE,g.ROOT


def main():
    output=HERE/'selected/reviews/check.json'
    assert not output.exists()
    old=p.read(HERE/'reviews/check.json');assert len(old['rotations'])==37
    started=time.monotonic();source=dict(old['source_sha256'])
    for z in ['cad/motor_spool_coupling_v1/candidate2.py','cad/motor_spool_coupling_v1/check2.py','cad/motor_spool_coupling_v1/reviews/check.json']:source[z]=g.sha(ROOT/z)
    rows,design=g.build();fixed,wires,arm,retained,replaced=g.fixed_inputs(source)
    first,_=g.original.build()
    oldspool=next(z for z in g.moved(first,140.) if z['name']=='thumb_ip_two_tail_spool')
    adapter=next(z for z in fixed if z['name']==g.NAME+'_adapter')
    common=oldspool['shape'].intersect(adapter['shape'],tol=1e-6)
    evidence=HERE/'selected/generated/visual_evidence';evidence.mkdir(parents=True,exist_ok=False)
    ev=[]
    for suffix in ['step','stl']:
        path=evidence/('first_tail_boss_adapter_common.'+suffix);cq.exporters.export(common,str(path));ev.append(path.relative_to(ROOT).as_posix())
    report=dict(completed=False,design=design,source_sha256=source,hand_geometry_changed=False,motor_positions_changed=False,
        reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),
        full_hand_complete=False,tendon_to_finger_connected=False,rotations=[],scope='New7custom coupling parts only,unmodified stock3 split reference parts kept.Motor internal source contact preserved separately.Full rotational bounds for new custom solids,finite exact angles and assembly paths.No tendon-to-finger or physical/strength approval.')
    report['original_failed_pair']=dict(angle_deg=140.,lead=oldspool['name'],part=adapter['name'],common_volume_mm3=common.Volume(),visual_evidence=ev,evidence_is_installed_part=False)
    def save():p.write(output,report)
    report['internal_pairs']=g.pairs(rows);report['thread_contacts']=g.thread_contacts(rows)
    report['rotating_envelopes'],description=g.custom_envelopes(rows)
    envelope=report.pop('rotating_envelopes')
    report['full_rotation_envelope_description']=description
    report['full_rotation_packaging']=c.material(envelope,fixed)
    report['full_rotation_wires']=c.material(envelope,wires)
    report['full_rotation_arm']=c.material(envelope,arm)
    save()
    custom=g.custom(rows);stock=[z for z in rows if z not in custom]
    horn=next(z for z in stock if z['name']=='thumb_ip_stock_output_horn')['shape']
    spool=next(z for z in custom if z['name']=='thumb_ip_two_tail_spool')['shape']
    report['horn_spool_distance_mm']=horn.distance(spool);report['horn_spool_common_mm3']=horn.intersect(spool,tol=1e-6).Volume()
    # The unchanged rear seating face has the same R9 outer boundary, tool
    # bore and four clearances; measure actual common planar support again.
    seat=cq.Face.makeFromWires(cq.Wire.makeCircle(7.9,cq.Vector(0,0,0),cq.Vector(1,0,0)))
    localhorn=next(z for z in stock if z['name']=='thumb_ip_stock_output_horn')['local_shape']
    localspool=next(z for z in custom if z['name']=='thumb_ip_two_tail_spool')['local_shape']
    report['shared_seat_area_mm2']=localhorn.intersect(seat,tol=1e-6).intersect(localspool,tol=1e-6).Area()
    # Include original failed angles; never restrict commanded angles to hide
    # the obstruction. The axisymmetric bound above covers all other angles.
    for angle in [0.,110.,140.,180.,270.,360.]:
        now=g.moved(custom,angle);rotatedstock=[z for z in g.moved(stock,angle)]
        got=dict(angle_deg=angle,packaging=c.material(now,fixed),wires=c.material(now,wires),arm=c.material(now,arm),
            stock_reference=c.material(now,rotatedstock))
        report['rotations'].append(got);save();print('corrected angle',angle,len(got['packaging']['overlaps']),flush=True)
    report['spool_install']=[];report['screw_install']=[]
    screws=[z for z in custom if z['role']=='nominal_tapping_screw_not_vendor_cad'];spoolgroup=[z for z in custom if z not in screws]
    for shift in [20.,16.,12.,8.,4.,0.]:
        for label,group,obstacles in [('spool_install',spoolgroup,fixed+wires+arm+stock),('screw_install',screws,fixed+wires+arm+stock+spoolgroup)]:
            if label=='screw_install' and shift not in [20.,0.]:continue
            report[label].append(dict(shift_x_mm=shift,native=c.material(g.moved(group,0.,shift),obstacles)))
        save()
    # Bound the whole straight screw insertion, with simple cylinders instead
    # of repeatedly intersecting coincident detailed screw-drive topology.
    swept=[];bores=[]
    for i,(y,z) in enumerate([(6.,0.),(0.,6.),(-6.,0.),(0.,-6.)],1):
        shaft=cq.Solid.makeCylinder(1.,26.,cq.Vector(-2.5,y,z),cq.Vector(1,0,0))
        head=cq.Solid.makeCylinder(2.1,21.6,cq.Vector(3.5,y,z),cq.Vector(1,0,0))
        swept.append(dict(name='screw_insertion_sweep_'+str(i),shape=g.pose(shaft.fuse(head))))
        bores.append(dict(screw=i,shaft_radius_mm=1.,shaft_clearance_radius_mm=1.1,head_radius_mm=2.1,counterbore_radius_mm=2.3,
            entry_range_mm=[0.,20.],full_path_within_coaxial_bores=True,
            proof='Shaft X[-2.5+s,3.5+s],head X[3.5+s,5.1+s];s in[0,20].Below X0 is intended tap engagement only.From X0 to3.5 R1 fits R1.1 clearance;from3.5 onward entire R2.1 head fits open R2.3 counterbore.No screw length or motor-hole depth change.'))
    report['screw_insertion_swept_against_fixed']=c.material(swept,fixed+wires+arm+stock)
    report['screw_insertion_coaxial_bore_proof']=bores
    tools=[dict(name='provisional_driver_stem_'+str(i),shape=g.pose(cq.Solid.makeCylinder(1.05,18.,cq.Vector(5.1,y,z),cq.Vector(1,0,0)))) for i,(y,z) in enumerate([(6.,0.),(0.,6.),(-6.,0.),(0.,-6.)],1)]
    report['driver_stem_space']=c.material(tools,fixed+wires+arm+stock+spoolgroup)
    checker=g.module('_drive2_hand_bounds',ROOT/'cad/remote_bus_v1/bbox_check.py')
    report['five_hand_pose_bounds']=checker.hand_boxes(envelope,source)
    root=HERE/'selected/generated/parts';root.mkdir(parents=True,exist_ok=False)
    saved=[]
    for z in rows:
        st=root/(z['name']+'.step');me=root/(z['name']+'.stl');cq.exporters.export(z['shape'],str(st));cq.exporters.export(z['shape'],str(me),tolerance=.06,angularTolerance=.1)
        saved.append(dict(**{k:v for k,v in z.items() if k not in ['shape','local_shape']},step=st.relative_to(ROOT).as_posix(),mesh=me.relative_to(ROOT).as_posix(),sha256_step=g.sha(st),sha256_stl=g.sha(me),bbox_mm=m.pack._bbox(z['shape'])))
    p.write(HERE/'selected/generated/scene.json',dict(parts=saved,retained_parts=retained,replaced_parts=replaced,
        total_packaging_rows=368,hand_reference='cad/motion_clearance_v2/generated/assembly_catalog.json',
        wires_reference='cad/remote_motor_lead_v2/generated/scene.json',hand_geometry_unchanged=True,motor_positions_unchanged=True,
        installed=False,coupling_candidate_only=True,whole_transmission_complete=False))
    # Keep four intentional major-diameter TAP/minor-bore contacts visible.
    allowed={(z['name'],'thumb_ip_stock_output_horn') for z in screws}
    swept_allowed={('screw_insertion_sweep_'+str(i),'thumb_ip_stock_output_horn') for i in range(1,5)}
    sweep_ok=not report['screw_insertion_swept_against_fixed']['operation_failures'] and all((z['lead'],z['part']) in swept_allowed for z in report['screw_insertion_swept_against_fixed']['overlaps'])
    tests=[report[k] for k in ['full_rotation_packaging','full_rotation_wires','full_rotation_arm','driver_stem_space']]+[z[k] for z in report['rotations'] for k in ['packaging','wires','arm']]+[z['native'] for z in report['spool_install']]
    threadtests=[z['stock_reference'] for z in report['rotations']]+[z['native'] for z in report['screw_install']]
    report['sampled_and_envelope_clear']=sweep_ok and all(g.clear(z) for z in tests) and all(not z['operation_failures'] and all((q['lead'],q['part']) in allowed for q in z['overlaps']) for z in threadtests) and not g.unexpected_internal(report['internal_pairs']) and all(z['acceptable_nominal_thread_envelope'] for z in report['thread_contacts']) and all(z['clear'] for z in report['five_hand_pose_bounds']['cases']) and report['horn_spool_distance_mm']<1e-5 and report['shared_seat_area_mm2']>1.
    report.update(completed=True,input_hash_guard=all(g.sha(ROOT/z)==h for z,h in source.items()),elapsed_s=time.monotonic()-started)
    save();print('corrected complete',report['sampled_and_envelope_clear'],report['elapsed_s'],flush=True)
    assert report['input_hash_guard']


if __name__=='__main__':main()
