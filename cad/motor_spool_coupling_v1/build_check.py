"""Build one explicitly fastened spool in the CURRENT remote motor location."""
import sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import model as g
from OCP.OSD import OSD_ThreadPool
c,b,m,p,cq,np,HERE,ROOT=g.c,g.b,g.m,g.p,g.cq,g.np,g.HERE,g.ROOT


def main():
    output=HERE/'reviews/check.json'
    if output.exists():raise FileExistsError('Preserve completed or partial study')
    started=time.monotonic()
    source={z:g.sha(ROOT/z) for z in ['cad/motor_spool_coupling_v1/model.py','cad/motor_spool_coupling_v1/build_check.py',
        'cad/thumb_ip_drive_v1/motor_adapter.py','cad/palm_motor_mount_v1/selected3/generated/scene.json',
        'cad/palm_motor_layout_v1/selected/generated/layout.json','cad/remote_motor_lead_v2/generated/scene.json',
        'cad/motion_clearance_v2/generated/assembly_catalog.json','cad/modular_roots_v9/sources/x330_horn_screw.png']}
    rows,design=g.build()
    fixed,wires,arm,retained,replaced=g.fixed_inputs(source)
    report=dict(completed=False,source_sha256=source,design=design,hand_geometry_changed=False,motor_positions_changed=False,
        reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),
        whole_hand_drive_complete=False,tendon_to_finger_connected=False,physical_approval=False,
        scope='One thumb-IP motor output coupling only;unmodified stock-reference horn/central retainer/rear idler kept.New spool/four nominal tapping screws/two tail clamps.Current358remaining packaging,64wire/outerboxes,neutral arm10;finite motor output rotations and separate axial assembly paths.No hand joint path,whole tendon routing,tension,physical fastening/strength or supplied screw/driver approval.',rotations=[])
    def save():p.write(output,report)
    report['internal_pairs']=g.pairs(rows)
    report['thread_contacts']=g.thread_contacts(rows)
    horn=next(z for z in rows if z['name']=='thumb_ip_stock_output_horn')['shape']
    spool=next(z for z in rows if z['name']=='thumb_ip_two_tail_spool')['shape']
    report['horn_spool_distance_mm']=spool.distance(horn)
    report['horn_spool_common_mm3']=spool.intersect(horn,tol=1e-6).Volume()
    # Face-to-face contact area is measured independently of common solid volume.
    localhorn=next(z for z in rows if z['name']=='thumb_ip_stock_output_horn')['local_shape']
    localspool=next(z for z in rows if z['name']=='thumb_ip_two_tail_spool')['local_shape']
    seat=cq.Face.makeFromWires(cq.Wire.makeCircle(7.9,cq.Vector(0,0,0),cq.Vector(1,0,0)))
    report['shared_seat_area_mm2']=localhorn.intersect(seat,tol=1e-6).intersect(localspool,tol=1e-6).Area()
    save()
    for angle in range(0,361,10):
        moved=g.moved(rows,angle)
        moving=[z for z in moved if z['moving']]
        idler=[z for z in moved if not z['moving']]
        got=dict(angle_deg=angle,packaging=c.material(moving,fixed+idler),wires=c.material(moving,wires),arm=c.material(moving,arm))
        report['rotations'].append(got);save()
        print('rotation',angle,[len(got[k]['overlaps']) for k in ['packaging','wires','arm']],flush=True)
    # New solid extrema remain remote from hand; use conservative saved STEP
    # boxes with all vertices transformed, as in BUS017, without reimporting364.
    checker=g.module('_coupling_hand_bounds',ROOT/'cad/remote_bus_v1/bbox_check.py')
    sweep=[]
    for angle in range(0,361,10):
        sweep.extend(g.moved(g.custom(rows),angle))
    report['five_hand_pose_bounds']=checker.hand_boxes(sweep,source)
    source.update({z:g.sha(ROOT/z) for z in ['cad/remote_bus_v1/bbox_check.py','cad/remote_bus_v1/model.py',
        'cad/articulation_drive_v2/verify.py','cad/functional_motion_v3/generated/catalog.json',
        'cad/palm_motor_layout_v1/reviews/check2.json']})
    # Install the spool and its two clamps first. Four TAP screws follow through
    # the counterbores. The motor remains at its accepted mounting location.
    report['spool_install']=[];report['screw_install']=[]
    custom=g.custom(rows);stock=[z for z in rows if z not in custom]
    screws=[z for z in custom if z['role']=='nominal_tapping_screw_not_vendor_cad']
    spoolgroup=[z for z in custom if z not in screws]
    obstacles=fixed+wires+arm+stock
    for shift in [20.,16.,12.,8.,4.,0.]:
        got=c.material(g.moved(spoolgroup,0.,shift),obstacles)
        report['spool_install'].append(dict(shift_x_mm=shift,native=got));save()
        got=c.material(g.moved(screws,0.,shift),obstacles+spoolgroup)
        # The same four bounded thread contacts are present only at the end.
        report['screw_install'].append(dict(shift_x_mm=shift,native=got));save()
    tools=[]
    for i,(y,z) in enumerate([(6.,0.),(0.,6.),(-6.,0.),(0.,-6.)],1):
        tools.append(dict(name='provisional_driver_stem_'+str(i),shape=g.pose(cq.Solid.makeCylinder(1.05,18.,cq.Vector(5.1,y,z),cq.Vector(1,0,0)))))
    report['driver_stem_space']=c.material(tools,obstacles+spoolgroup)
    report['driver_note']='R1.05 straight18mm stems starting above nominal head;head/driver SKU,handle and actual tightening unverified.'
    folder=HERE/'generated/parts';folder.mkdir(parents=True,exist_ok=False)
    saved=[]
    for row in rows:
        st=folder/(row['name']+'.step');mesh=folder/(row['name']+'.stl')
        cq.exporters.export(row['shape'],str(st));cq.exporters.export(row['shape'],str(mesh),tolerance=.06,angularTolerance=.1)
        saved.append(dict(**{k:v for k,v in row.items() if k not in ['shape','local_shape']},
            step=st.relative_to(ROOT).as_posix(),mesh=mesh.relative_to(ROOT).as_posix(),
            sha256_step=g.sha(st),sha256_stl=g.sha(mesh),bbox_mm=m.pack._bbox(row['shape'])))
    p.write(HERE/'generated/scene.json',dict(parts=saved,retained_parts=retained,replaced_parts=replaced,
        hand_reference='cad/motion_clearance_v2/generated/assembly_catalog.json',wires_reference='cad/remote_motor_lead_v2/generated/scene.json',
        motor_positions_unchanged=True,hand_geometry_unchanged=True,candidate_installed=False,
        coupling_candidate_only=True,whole_transmission_complete=False))
    tests=[z[k] for z in report['rotations'] for k in ['packaging','wires','arm']]
    report['sampled_external_rotations_clear']=all(g.clear(z) for z in tests)
    report['internal_clear_except_bounded_tap_contacts']=not g.unexpected_internal(report['internal_pairs']) and all(z['acceptable_nominal_thread_envelope'] for z in report['thread_contacts']) and not report['internal_pairs']['operation_failures']
    report['assembly_spool_clear']=all(g.clear(z['native']) for z in report['spool_install'])
    allowed={(z['name'],'thumb_ip_stock_output_horn') for z in screws}
    report['assembly_screws_clear_except_declared_thread_contacts']=all(not z['native']['operation_failures'] and all((q['lead'],q['part']) in allowed for q in z['native']['overlaps']) for z in report['screw_install'])
    # Validate each actual assembly common against its finite tap-hole region;
    # shifted screw penetration is a subset of the same X[-2.5,0] region.
    report['assembly_thread_region_proofs']=[]
    for shift in [20.,16.,12.,8.,4.,0.]:
        for i,(y,z) in enumerate([(6.,0.),(0.,6.),(-6.,0.),(0.,-6.)],1):
            screw=g.moved([screws[i-1]],0.,shift)[0]['shape'];common=screw.intersect(horn,tol=1e-6)
            region=g.pose(cq.Solid.makeCylinder(1.00001,2.50002,cq.Vector(-2.50001,y,z),cq.Vector(1,0,0)))
            report['assembly_thread_region_proofs'].append(dict(shift_x_mm=shift,screw=i,outside_mm3=common.cut(region,tol=1e-6).Volume()))
    report['nominal_coupling_candidate_clear']=report['sampled_external_rotations_clear'] and report['internal_clear_except_bounded_tap_contacts'] and report['assembly_spool_clear'] and report['assembly_screws_clear_except_declared_thread_contacts'] and all(z['outside_mm3']<=.001 for z in report['assembly_thread_region_proofs']) and g.clear(report['driver_stem_space']) and all(z['clear'] for z in report['five_hand_pose_bounds']['cases']) and report['horn_spool_distance_mm']<1e-5 and report['shared_seat_area_mm2']>1.
    report.update(completed=True,input_hash_guard=all(g.sha(ROOT/z)==h for z,h in source.items()),elapsed_s=time.monotonic()-started)
    save();print('complete',report['nominal_coupling_candidate_clear'],report['elapsed_s'],flush=True)
    assert report['input_hash_guard']


if __name__=='__main__':main()
