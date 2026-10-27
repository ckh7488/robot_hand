"""Continue the inherited10-part bench,then restore original motor packets if clear."""
import importlib.util,sys,time
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm067_check',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,d,ROOT,HERE=m.n,m.np,m.d,m.ROOT,m.HERE
from OCP.OSD import OSD_ThreadPool
from OCP.BRepAdaptor import BRepAdaptor_Surface


def main():
    out=HERE/'reviews/check.json';assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    started=time.monotonic();rows,membership,source=m.context();lookup={z['name']:z for z in rows}
    inherited=n.p.read(ROOT/'cad/ordered_roll_root_assembly_v1/reviews/selected_check.json')
    assert inherited['completed'] and not inherited['bench_completed'] and len(inherited['bench_final_present_names'])==10
    packets=m.a.packets(rows);assert sum(len(p['names']) for p in packets)==408
    common_ev=n.p.read(ROOT/'cad/ordered_roll_root_assembly_v1/reviews/evidence.json')
    common_meta=[z for z in n.p.read(ROOT/common_ev['common_scene'])['parts'] if z['name']=='arm066_failed_entry_common_1']
    saved_common=n.c.load(common_meta,source)[0]
    source[Path(__file__).relative_to(ROOT).as_posix()]=n.m.drive.sha(Path(__file__))
    final_names=set(lookup);present=set(inherited['bench_final_present_names']);bench=set(inherited['bench_names'])
    raw=dict(completed=False,source_sha256=source,paths=[],native={},packets=packets,bench_names=sorted(bench),
        initial_bench_present_names=sorted(present),initial_bench_absent_names=sorted(final_names-present),
        inherited_bench10_exact_input_names=inherited['bench_final_present_names'],inherited_ARM066_native_repeated=0,
        original_final886_names=sorted(lookup),current862_replaced=False,proposal886_adopted=False,new_hardware_geometry=0,
        reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,OCC_pool_threads=1,whole_proposal_raw_clear=False,
        inherited_ARM065_common1_topology_mismatch_unresolved=True,packet_restoration_started=False,
        saved_first_bolt_common_read_only=dict(name=saved_common['name'],bounds_mm=m.bbox([saved_common]),source_record=common_ev['common_queries'][1],new_common_Boolean_queries=0),
        front_bolt_geometry_basis=[],
        declared_route_basis='One natural assembly route:45degree tilted bolt translates through the central gap between paired front spokes;tilt reduces while tip enters the lower counterbore,then axial seating. No angle/radius search,installed motor axis/command change or geometry cut.',
        conditional_scope='Inherit the same10 assembled bench rows. Root/beam/caps/nuts manually supported,other876 absent. If bench22 completes,register at original neutral world pose with all nonpacket478 rows assumed assembled and restore16packets at exact original final transforms. Registration/other factory assembly/packet internal bench assembly not newly tested.',
        supplied_tools_threads_torque_strength_continuous_or_physical_approved=False)
    for i,x in enumerate([-25.5,25.5]):
        name='arm063_M3x12_envelope_'+str(i);sh=lookup[name]['shape'];b=m.bbox([lookup[name]])
        cylinders=[]
        for face in sh.Faces():
            if face.geomType()=='CYLINDER':
                cy=BRepAdaptor_Surface(face.wrapped).Cylinder()
                cylinders.append(dict(radius_mm=cy.Radius(),axis_direction=list(cy.Axis().Direction().Coord()),axis_point=list(cy.Axis().Location().Coord())))
        assert abs(b[0,2]+6.)<1e-9 and abs(b[1,2]-8.)<1e-9
        assert sorted(round(z['radius_mm'],8) for z in cylinders)==[1.5,2.85]
        raw['front_bolt_geometry_basis'].append(dict(name=name,bounds_mm=b,cylinders=cylinders,shaft_Z_mm=[-4,8],head_Z_mm=[-6,-4],tip_mm=[x,-162.5,8.],
            existing_counterbore_basis='ARM063 source construction:R2.85,Z[-26,-4],axisX+-25.5/Y-162.5;declared construction,not a new contact or supplierfit test.'))
    def save():n.a.save(out,raw)
    def path(stage,label,names,states,installed):
        assert not set(names)&installed
        rec=dict(id=label,stage=stage,moving_names=list(names),present_names=sorted(installed),absent_names=sorted(final_names-installed),samples=[],completed=False,manual_support=True)
        raw['paths'].append(rec);save()
        for i,s in enumerate(states):
            key=label+'_'+str(i);M=np.array(s['transform']);moving=m.moved([lookup[k] for k in names],M)
            q=n.c.material(moving,[lookup[k] for k in installed]);raw['native'][key]=q
            rec['samples'].append(dict(id=key,**s))
            if i==0:
                bb=m.bbox(moving);ob=m.bbox([lookup[k] for k in installed]);rec['initial_AABB_below_all_present_Z_gap_mm']=float(ob[0,2]-bb[1,2])
            save();print(key,q['pairs_in_scope'],q['exact_pairs'],len(q['overlaps']),flush=True)
            if q['overlaps'] or q['operation_failures']:
                raw['first_blocked_path']=label;raw['first_blocked_sample']=key;return False
        rec['completed']=True;installed.update(names);save();return True
    blocked=False
    for i,x in enumerate([-25.5,25.5]):
        blocked=not path('bench','front_tilt_'+str(i),['arm063_M3x12_envelope_'+str(i)],m.front_route(x),present)
        if blocked:break
    for name in sorted(k for k in bench if 'M3x12' in k and not k.startswith('arm063_')):
        if blocked:break
        states=[dict(phase='rear_bolt_axial',transform=m.translate([0,0,-z])) for z in [100.,80.,60.,40.,20.,10.,0.]]
        blocked=not path('bench','rear_'+name,[name],states,present)
    for name in sorted(k for k in bench if k.startswith('arm064_output_')):
        if blocked:break
        states=[dict(phase='output_bolt_axial',transform=m.translate([0,y,0])) for y in [20.,10.,0.]]
        blocked=not path('bench','output_'+name,[name],states,present)
    raw['bench_completed']=not blocked and present==bench;raw['bench_final_present_names']=sorted(present)
    packet_names={k for p in packets for k in p['names']};world=final_names-packet_names;restored=[]
    raw['packet_assumed_initial_present_names']=sorted(world);raw['packet_initial_absent_names']=sorted(packet_names)
    if not blocked:
        raw['packet_restoration_started']=True;assert bench<=world
        for p in packets:
            theta=np.deg2rad(p['azimuth_deg']);radial=np.array([np.cos(theta),0.,np.sin(theta)]);normal=np.array(p['approach_normal'])
            names=p['body_names'];axial=8.*normal;mb=m.bbox(m.moved([lookup[k] for k in names],m.translate(axial)));ob=m.bbox([lookup[k] for k in world])
            corners=lambda b:np.array([[x,y,z] for x in b[:,0] for y in b[:,1] for z in b[:,2]])
            start=float(max(100.,np.ceil((max(corners(ob)@radial)-min(corners(mb)@radial)+10.)/20.)*20.))
            p['radial_start_mm']=start;p['initial_projection_separation_mm']=min(corners(mb)@radial)+start-max(corners(ob)@radial)
            vectors=[axial+t*radial for t in list(np.arange(start,0.,-20.))+[0.]]+[4.*normal,np.zeros(3)]
            states=[dict(phase='packet_side_then_normal_seating',transform=m.translate(v)) for v in vectors]
            blocked=not path('packet','packet_'+p['module'],names,states,world)
            if blocked:break
            for kind,sign in [('nut',-1.),('screw',1.)]:
                for name in [k for k in p['mounting_names'] if kind in k]:
                    states=[dict(phase='module_rack_'+kind,transform=m.translate(sign*v*normal)) for v in [20.,10.,0.]]
                    blocked=not path('packet','mount_'+name,[name],states,world)
                    if blocked:break
                if blocked:break
            if blocked:break
            restored.append(p['module'])
    raw['restored_modules']=restored;raw['packet_restoration_completed']=raw['packet_restoration_started'] and not blocked and world==final_names
    raw['packet_final_present_names']=sorted(world);raw['packet_final_pending_names']=sorted(final_names-world)
    total=d.k.flat(list(raw['native'].values()))
    raw.update(completed=True,native_groups=len(raw['native']),pairs_in_scope=total['pairs_in_scope'],AABB_native_pairs=total['exact_pairs'],
        overlap_records=len(total['overlaps']),operation_failures=len(total['operation_failures']),raw_new_tests_clear=not total['overlaps'] and not total['operation_failures'],
        input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()),elapsed_s=time.monotonic()-started)
    save();print('complete',raw['native_groups'],raw['overlap_records'],'bench',raw['bench_completed'],'packets',raw['packet_restoration_completed'],flush=True)


if __name__=='__main__':main()
