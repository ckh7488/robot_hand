"""Conditional bench assembly then original packet restoration; stop on blockage."""
import importlib.util,sys,time
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm066_check',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,d,ROOT,HERE=m.n,m.np,m.d,m.ROOT,m.HERE
from OCP.OSD import OSD_ThreadPool


def main():
    out=HERE/'reviews/check.json';assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    started=time.monotonic();rows,membership,source=m.context();lookup={z['name']:z for z in rows};packets=m.packets(rows)
    survey=n.p.read(HERE/'reviews/survey.json');assert survey['completed'] and survey['packets']==packets
    for f in [Path(__file__),HERE/'reviews/survey.json']:source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    final_names=sorted(lookup);packet_names={k for p in packets for k in p['names']}
    rear_mount=[z['name'] for z in rows if z['name'].startswith(('arm065_rear_M3','arm064_clamp_M3','arm064_clamp_nut'))]
    front_mount=[z['name'] for z in rows if z['name'].startswith(('arm063_M3x12','arm063_M3_nut'))]
    mounting=sorted(rear_mount+front_mount);assert len(mounting)==12
    output=sorted(z['name'] for z in rows if z['name'].startswith('arm064_output_M3x8_'));assert len(output)==6
    bench_names={m.a.NAME,m.a.CAP,'arm063_recessed_front_cap','Forearm_box_beam',*mounting,*output};assert len(bench_names)==22
    raw=dict(completed=False,source_sha256=source,paths=[],native={},packets=packets,final_reference_names=final_names,
        bench_names=sorted(bench_names),bench_initial_present_names=[m.a.NAME],bench_initial_absent_names=sorted(set(final_names)-{m.a.NAME}),
        packet_restoration_initial_absent_names=sorted(packet_names),packet_restoration_assumed_present_names=sorted(set(final_names)-packet_names),
        new_hardware_geometry=0,current862_replaced=False,proposal886_adopted=False,previous_ARM065_groups_repeated=0,
        inherited_ARM065_geometry_common1_topology_mismatch_unresolved=True,whole_proposal_raw_clear=False,
        reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,OCC_pool_threads=1,
        conditions='Two conditional starts:bare root bench with other885 absent;then same completed22 bench parts registered at original neutral world positions and other nonpacket rows assumed already assembled. Registration/other factory assembly not tested. Packets are unchanged bench subassemblies excluding their four rack fasteners;manual support before fastening. No permanent omission/placement change.',
        original_module_interfaces='ARM054 placement/module rows;ARM059 replaced thumb support/jaws included explicitly. Actual saved ARM065 world shapes used;old world wiring/service results not inherited as passes.',
        route_basis='Full bounding-clear cap/beam/radial entry starts,finite translational poses only. Radial side entry at8mm axial offset then normal seating;8mm is a declared assembly pose,not installed clearance or a new required gap.',
        new_continuous_or_tool_torque_strength_approval=False)
    def save():n.a.save(out,raw)
    def path(stage,label,names,vectors,present):
        assert not set(names)&set(present)
        rec=dict(id=label,stage=stage,moving_names=list(names),present_names=sorted(present),
            absent_names=sorted(set(final_names)-set(present)),samples=[],completed=False,manual_support=True)
        raw['paths'].append(rec);save()
        for i,v in enumerate(vectors):
            testid=label+'_'+str(i);moving=m.moved([lookup[k] for k in names],v)
            q=n.c.material(moving,[lookup[k] for k in present]);raw['native'][testid]=q
            rec['samples'].append(dict(id=testid,translation_mm=np.array(v).tolist()))
            save();print(testid,q['pairs_in_scope'],q['exact_pairs'],len(q['overlaps']),flush=True)
            if q['overlaps'] or q['operation_failures']:
                raw['first_blocked_path']=label;raw['first_blocked_sample']=testid;return False
        rec['completed']=True;present.update(names);save();return True
    present={m.a.NAME};blocked=False
    # Rear end starts beyond the complete root in +Y. The wider wrist end stays
    # forward of the closed front frame throughout this axial insertion.
    root_bounds=m.bbox([lookup[m.a.NAME]]);bb=m.bbox([lookup['Forearm_box_beam']])
    start=float(np.ceil((root_bounds[1,1]-bb[0,1]+10.)/10.)*10.)
    distances=list(np.arange(start,0.,-20.))+[0.]
    raw['beam_initial_AABB_separation_Y_mm']=bb[0,1]+start-root_bounds[1,1]
    blocked=not path('bench','bench_beam',['Forearm_box_beam'],[[0.,t,0.] for t in distances],present)
    for name in [m.a.CAP,'arm063_recessed_front_cap']:
        if blocked:break
        b=m.bbox([lookup[name]]);ob=m.bbox([lookup[k] for k in present])
        top=float(np.ceil((ob[1,2]-b[0,2]+10.)/10.)*10.)
        vectors=[[0.,0.,z] for z in list(np.arange(top,0.,-20.))+[14.,7.,0.]]
        blocked=not path('bench','bench_'+name,[name],vectors,present)
    for kind in ['nut','M3x12']:
        for name in [k for k in mounting if kind in k]:
            if blocked:break
            sign=1. if kind=='nut' else -1.
            blocked=not path('bench','bench_'+name,[name],[[0.,0.,sign*z] for z in [40.,20.,10.,0.]],present)
        if blocked:break
    for name in output:
        if blocked:break
        blocked=not path('bench','bench_'+name,[name],[[0.,v,0.] for v in [20.,10.,0.]],present)
    raw['bench_completed']=not blocked and present==bench_names;raw['bench_final_present_names']=sorted(present)
    restored=[];world=set(final_names)-packet_names
    if not blocked:
        assert bench_names<=world
        for p in packets:
            names=p['body_names'];theta=np.deg2rad(p['azimuth_deg']);radial=np.array([np.cos(theta),0.,np.sin(theta)])
            normal=np.array(p['approach_normal']);axial=8.*normal
            moving=m.moved([lookup[k] for k in names],axial)
            mb=m.bbox(moving);ob=m.bbox([lookup[k] for k in world])
            corners=lambda b:np.array([[x,y,z] for x in b[:,0] for y in b[:,1] for z in b[:,2]])
            start=float(max(100.,np.ceil((max(corners(ob)@radial)-min(corners(mb)@radial)+10.)/20.)*20.))
            p['radial_start_mm']=start;p['initial_projection_separation_mm']=min(corners(mb)@radial)+start-max(corners(ob)@radial)
            vectors=[axial+t*radial for t in list(np.arange(start,0.,-20.))+[0.]]+[4.*normal,np.zeros(3)]
            blocked=not path('packet','packet_'+p['module'],names,vectors,world)
            if blocked:break
            for kind,sign in [('nut',-1.),('screw',1.)]:
                for name in [k for k in p['mounting_names'] if kind in k]:
                    blocked=not path('packet','mount_'+name,[name],[sign*v*normal for v in [20.,10.,0.]],world)
                    if blocked:break
                if blocked:break
            if blocked:break
            restored.append(p['module'])
    # Record failed candidates honestly:the remaining original names are pending,
    # never silently treated as permanent motor removals or a completed factory.
    raw['restored_modules']=restored;raw['packet_restoration_completed']=not blocked and world==set(final_names)
    raw['packet_final_present_names']=sorted(world);raw['packet_final_pending_names']=sorted(set(final_names)-world)
    raw['reference_final_886_unchanged']=set(final_names)=={z['name'] for z in rows}
    total=d.k.flat(list(raw['native'].values()))
    raw.update(completed=True,native_groups=len(raw['native']),pairs_in_scope=total['pairs_in_scope'],AABB_native_pairs=total['exact_pairs'],
        overlap_records=len(total['overlaps']),operation_failures=len(total['operation_failures']),
        raw_new_tests_clear=not total['overlaps'] and not total['operation_failures'],
        input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()),elapsed_s=time.monotonic()-started)
    save();print('complete',raw['native_groups'],raw['overlap_records'],'bench',raw['bench_completed'],'packets',raw['packet_restoration_completed'],flush=True)


if __name__=='__main__':main()
