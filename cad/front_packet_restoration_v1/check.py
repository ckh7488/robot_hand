"""Check original front modules in their actual basis and cumulative world inventory."""
import importlib.util,sys,time
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm078_check',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,d,ROOT,HERE=m.n,m.np,m.d,m.ROOT,m.HERE
from OCP.OSD import OSD_ThreadPool
from OCP.BRepAdaptor import BRepAdaptor_Surface


def main():
    out=HERE/'reviews/check.json';assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    (HERE/'reviews').mkdir(exist_ok=True);started=time.monotonic();rows,_,source,packets=m.context();lookup={z['name']:z for z in rows};final=set(lookup)
    prior=n.p.read(ROOT/'cad/individual_base_nut_assembly_v1/reviews/check.json');world=set(prior['packet_final_present_names']);assert len(world)==718 and len(prior['restored_modules'])==9 and set(prior['mapped_special_packet']['names'])<=world
    plans=[p for p in packets if p['module'] not in prior['restored_modules']];assert len(plans)==7 and all(p['row']==1 and len(p['names'])==24 for p in plans)
    placements={p['module']:p for p in n.p.read(ROOT/'cad/eight_bank_layout_v1/reviews/check.json')['placements']};guides={p['module']:p for p in n.p.read(ROOT/'cad/wire_guides_v1/generated/scene.json')['groups']}
    boxes={k:m.corners(n.c.m.pack._bbox(z['shape'])) for k,z in lookup.items()}
    source[Path(__file__).relative_to(ROOT).as_posix()]=n.m.drive.sha(Path(__file__))
    survey=dict(completed=True,initial_world_rows=718,planned_front_modules=[p['module'] for p in plans],actual_axes=[],module_frames=[],new_geometry=0,old_native_repeated=0)
    for p in plans:
        name=p['module'];pl=placements[name];P=np.array(pl['proposed_installation_transform']);delta=np.array(pl['transform_from_current']);radial=-P[:3,2];normal=P[:3,0];opening=delta[:3,:3]@np.array([0.,0.,1.]);frame=delta@np.array(guides[name]['frame_world_mm']);assert np.max(abs(opening+normal))<1e-10
        survey['module_frames'].append(dict(module=name,placement=P,delta=delta,radial=radial,normal=normal,opening=opening,guide_frame=frame))
        for suffix,axis in [('_base_screws',radial),('_rack_screw_1',normal)]:
            cy=[]
            for i,face in enumerate(lookup[name+suffix]['shape'].Faces()):
                if face.geomType()=='CYLINDER':
                    s=BRepAdaptor_Surface(face.wrapped).Cylinder();v=np.array(s.Axis().Direction().Coord());assert abs(v@axis)>1.-1e-10
                    cy.append(dict(face=i,radius_mm=s.Radius(),axis=v,point=np.array(s.Axis().Location().Coord())))
            assert cy;survey['actual_axes'].append(dict(name=name+suffix,expected_axis=axis,cylinders=cy))
    n.a.save(HERE/'reviews/survey.json',survey);source['cad/front_packet_restoration_v1/reviews/survey.json']=n.m.drive.sha(HERE/'reviews/survey.json')
    raw=dict(completed=False,source_sha256=source,native={},paths=[],module_contracts=[],original16packets=packets,explicit_replacement={m.OLD:m.NAME},mapped_special_packet=prior['mapped_special_packet'],initial_world718_names=sorted(world),final886_names=sorted(final),planned_front_modules=survey['planned_front_modules'],restored_before=prior['restored_modules'],new_restored_modules=[],new_hardware_geometry=0,current862_replaced=False,proposal886_adopted=False,reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,OCC_pool_threads=1,old_native_repeated=0,
        preserved_upper_M3_whole_cut=prior['preserved_upper_M3_whole_cut'],scope='Original718 including complete special48 and rear8 kept. Each front module bench tested afresh in its actual installation basis;world obstacles accumulate. Originalbody19 enters finalnormal0;M3 before original fourM2nut compound. Projected actual-part AABB entry/mid/exit events enrich each straight segment. Finite sampled paths only. Assumednonpacket478/manualholding/benchtransport/registration/outputinternal/mating/harnessflex/captive/tools/threads/torque/force/exterior remain unverified.')
    def save():n.a.save(out,raw)
    def path(module,stage,label,names,present,vectors,axis):
        names=sorted(names);present=set(present);assert not set(names)&present
        samples,events=m.enrich_vectors(vectors,names,sorted(present),boxes)
        rec=dict(id=module+'__'+label,module=module,stage=stage,moving_names=names,present_names=sorted(present),absent_names=sorted(final-present),approach_axis=axis,requested_vectors=vectors,projected_obstacle_events=events,planned_sample_count=len(samples),samples=[],completed=False);raw['paths'].append(rec);save()
        fixed=[lookup[k] for k in sorted(present)]
        for i,sample in enumerate(samples):
            v=np.array(sample['translation_mm']);M=m.translate(v);key=rec['id']+'_'+str(i);moving=m.moved([lookup[k] for k in names],M);q=n.c.material(moving,fixed);raw['native'][key]=q;rec['samples'].append(dict(id=key,**sample,transform=M))
            if i==0:
                mb=m.project_bounds(moving,axis);fb=m.project_bounds(fixed,axis);rec['initial_projection_separation_mm']=max(fb[0]-mb[1],mb[0]-fb[1]);assert rec['initial_projection_separation_mm']>0.
            save();print(key,q['pairs_in_scope'],q['exact_pairs'],len(q['overlaps']),flush=True)
            if q['overlaps'] or q['operation_failures']:raw['first_blocked_sample']=key;raw['first_blocked_path']=rec['id'];return False
        rec['completed']=True;present_target.update(names);save();return True
    def run(module,stage,label,names,present,vectors,axis):
        nonlocal present_target
        present_target=present;return path(module,stage,label,names,present,vectors,axis)
    def benchpath(name,label,names,present,axis):return run(name,'carrier_bench',label,names,present,m.bench_vectors([lookup[k] for k in names],[lookup[k] for k in present],axis),axis)
    present_target=set();blocked=False
    for p,f in zip(plans,survey['module_frames']):
        name=p['module'];radial=np.array(f['radial']);normal=np.array(f['normal']);opening=np.array(f['opening']);frame=np.array(f['guide_frame']);base=name+'_base_nuts';body=set(p['body_names'])-{base};core={name+s for s in ['_motor','_rotor','_body_screws','_spool']};unit=core|{name+'_cradle',name+'_base_screws'}
        electrical={name+s for s in ['_lead1','_lead2','_lead3','_plug_outer']};adapter={name+'_adapter'};spacers={name+'_guide_spacer_'+str(i) for i in [1,2]};guide=name+'_wire_guide_base';lid=name+'_wire_guide_lid';nuts={name+'_lid_nut_'+str(i) for i in [1,2]};screws={name+'_lid_screw_'+str(i) for i in [1,2]}
        assert body==unit|electrical|adapter|spacers|{guide,lid}|nuts|screws and len(body)==19
        c=dict(**f,body19_names=sorted(body),unit6_names=sorted(unit),core4_names=sorted(core),original24_names=p['names'],mounting_names=p['mounting_names'],world_before_names=sorted(world),bench_inherited=False,completed=False);raw['module_contracts'].append(c);save()
        bench={name+'_cradle'};near=[80.,60.,40.,20.,10.,5.,0.]
        if not run(name,'bench','M2_screws_before_motor',[name+'_base_screws'],bench,[-x*radial for x in near],-radial):blocked=True;break
        if not run(name,'bench','motor_reference_into_cradle',core,bench,[-x*radial for x in near],-radial):blocked=True;break
        carrier=set(adapter)
        if not benchpath(name,'guide_base_side_before_motor',[guide],carrier,radial):blocked=True;break
        for spacer in sorted(spacers):
            if not benchpath(name,'spacer_'+spacer,[spacer],carrier,-opening):blocked=True;break
        if blocked:break
        if not benchpath(name,'unit7_with_own_plug_into_open_guide',unit|{name+'_plug_outer'},carrier,-radial):blocked=True;break
        if not benchpath(name,'lay_three_leads_in_open_groove',electrical-{name+'_plug_outer'},carrier,opening):blocked=True;break
        for nut in sorted(nuts):
            side=(-1. if nut.endswith('_1') else 1.)*frame[:3,1];offset=-4.*opening;start=m.outside_start([lookup[nut]],[lookup[k] for k in carrier],side,offset)
            if not run(name,'carrier_bench','guide_nut_side_then_pocket_'+nut,[nut],carrier,[x*side+offset for x in m.inward_steps(start)]+[-2.*opening,np.zeros(3)],side):blocked=True;break
        if blocked:break
        if not benchpath(name,'guide_lid_after_leads',[lid],carrier,opening):blocked=True;break
        for screw in sorted(screws):
            if not benchpath(name,'guide_M2_'+screw,[screw],carrier,opening):blocked=True;break
        if blocked:break
        assert carrier==body;c['body19_bench_completed']=True
        start=m.outside_start([lookup[k] for k in body],[lookup[k] for k in world],radial,np.zeros(3))
        if not run(name,'world','body19_final_datum_radial',body,world,[x*radial for x in list(np.arange(start,19.,-20.))+[15.,10.,5.,0.]],radial):blocked=True;break
        for nut in [k for k in p['mounting_names'] if '_nut_' in k]:
            offset=-20.*normal;start=m.outside_start([lookup[nut]],[lookup[k] for k in world],radial,offset)
            if not run(name,'world','rack_nut_'+nut,[nut],world,[x*radial+offset for x in m.inward_steps(start)]+[-10.*normal,np.zeros(3)],radial):blocked=True;break
        if blocked:break
        for screw in [k for k in p['mounting_names'] if '_screw_' in k]:
            offset=40.*normal;start=m.outside_start([lookup[screw]],[lookup[k] for k in world],radial,offset)
            if not run(name,'world','rack_M3_'+screw,[screw],world,[x*radial+offset for x in m.inward_steps(start)]+[20.*normal,10.*normal,np.zeros(3)],radial):blocked=True;break
        if blocked:break
        start=m.outside_start([lookup[base]],[lookup[k] for k in world],radial,np.zeros(3))
        if not run(name,'world','four_M2_base_nuts_last',[base],world,[x*radial for x in list(np.arange(start,19.,-20.))+[15.,10.,5.,0.]],radial):blocked=True;break
        assert set(p['names'])<=world;c['completed']=True;c['world_after_names']=sorted(world);raw['new_restored_modules'].append(name);save()
    total=d.k.flat(list(raw['native'].values()));raw.update(completed=True,new_native_groups=len(raw['native']),pairs_in_scope=total['pairs_in_scope'],AABB_native_pairs=total['exact_pairs'],overlap_records=len(total['overlaps']),operation_failures=len(total['operation_failures']),raw_new_tests_clear=not total['overlaps'] and not total['operation_failures'],restored_modules=prior['restored_modules']+raw['new_restored_modules'],packet_final_present_names=sorted(world),packet_final_pending_names=sorted(final-world),all16_packet_restoration_completed=world==final,input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()),elapsed_s=time.monotonic()-started)
    save();print('complete',raw['new_native_groups'],raw['overlap_records'],'world',len(world),'restored',len(raw['restored_modules']),flush=True)


if __name__=='__main__':main()
