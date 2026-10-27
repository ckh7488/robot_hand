"""Read the actual original48 and its interface axes; no old Boolean repeats."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm072_survey',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,ROOT,HERE=m.n,m.np,m.ROOT,m.HERE
old=n.m.drive.module('_arm072_metadata',ROOT/'cad/ordered_roll_root_assembly_v1/render_evidence.py')
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.OSD import OSD_ThreadPool


def main():
    out=HERE/'reviews/survey.json';assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    prior=n.p.read(ROOT/'cad/datum_plane_packet_entry_v1/reviews/check.json');p=next(z for z in prior['original16packets'] if z['module']=='thumb_ip_actuator')
    assert len(p['names'])==48 and len(p['body_names'])==44 and len(prior['packet_final_present_names'])==670
    metas,matrix=old.metadata();meta={z['name']:z for z in metas};source={};records=[]
    placement=next(z for z in n.p.read(ROOT/'cad/eight_bank_layout_v1/reviews/check.json')['placements'] if z['module']==p['module'])
    P=np.array(placement['proposed_installation_transform']);delta=np.array(placement['transform_from_current'])
    guide=next(z for z in n.p.read(ROOT/'cad/wire_guides_v1/generated/scene.json')['groups'] if z['module']==p['module'])
    frame=delta@np.array(guide['frame_world_mm']);radial=-P[:3,2];normal=P[:3,0];opening=delta[:3,:3]@np.array([0.,0.,1.])
    for name in p['names']:
        z=dict(meta[name])
        if 'step' not in z:
            rel='cad/modular_roots_v9/parts/'+name+'.step';z.update(step=rel,sha256_step=n.m.drive.sha(ROOT/rel))
        sh=n.g.original.transform(n.c.load([z],source)[0]['shape'],matrix(name));cy=[];planes=[]
        if name.endswith(('_base_screws','_rack_screw_1','_lid_screw_1')) or name in ['thumb_ip_spool_tap6_1','thumb_ip_barrel_adjuster_0','motor_housing_clamp_0_bolt_0','thumb_ip_actuator_adapter','arm059_integrated_electrical_R5_base']:
            for i,f in enumerate(sh.Faces()):
                s=BRepAdaptor_Surface(f.wrapped)
                if f.geomType()=='CYLINDER':
                    c=s.Cylinder();cy.append(dict(index=i,radius_mm=c.Radius(),axis=c.Axis().Direction().Coord(),point=c.Axis().Location().Coord()))
                elif f.geomType()=='PLANE' and name in ['thumb_ip_actuator_adapter','arm059_integrated_electrical_R5_base']:
                    pl=s.Plane();axis=np.array(pl.Axis().Direction().Coord())
                    if abs(axis@normal)>1.-1e-10:planes.append(dict(index=i,axis=axis,point=pl.Location().Coord(),area_mm2=f.Area(),bbox=n.c.m.pack._bbox(f)))
        records.append(dict(name=name,bbox=n.c.m.pack._bbox(sh),cylinders=cy,normal_planes=planes,actual_transform=matrix(name)))
    for f in [Path(__file__),HERE/'model.py',ROOT/'cad/datum_plane_packet_entry_v1/reviews/check.json',ROOT/'cad/eight_bank_layout_v1/reviews/check.json',ROOT/'cad/wire_guides_v1/generated/scene.json',ROOT/'cad/integrated_motor_end_v1/model.py',ROOT/'cad/integrated_motor_end_v1/README.md',ROOT/'cad/motor_spool_coupling_v1/model.py',ROOT/'cad/motor_spool_coupling_v1/selected/reviews/check.json']:
        source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    result=dict(completed=True,source_sha256=source,original_packet=p,placement=P,delta=delta,radial=radial,normal=normal,guide_opening_axis=opening,guide_actual_frame=frame,actual48_parts=records,
        initial_present_names=prior['packet_final_present_names'],initial_pending_names=prior['packet_final_pending_names'],new_native_groups=0,old_native_groups_repeated=0,new_common_queries=0,new_hardware_geometry=0,current862_replaced=False,proposal886_adopted=False,OCC_pool_threads=1,
        scope='Actual stored final886 geometry and transforms. Original special48 includes body44+rack4;no generic rotor/spool/guide_spacer rows. Read original coupling nominal TAP/vendor assumptions only;no old Booleans rerun. Integrated base/R5 sharedM3 stack retained. Internal vendor/output references,tools/torque/flexible harness/captive retention and wholefactory unverified.')
    result['input_hash_guard']=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items());n.a.save(out,result)
    print('special48',len(records),'radial',radial,'normal',normal,'opening',opening,flush=True)
    for z in records:
        print(z['name'],np.round(z['bbox'],4).tolist(),flush=True)
    for z in records:
        if z['cylinders']:print('axis',z['name'],[(round(c['radius_mm'],4),np.round(c['axis'],4).tolist()) for c in z['cylinders']],flush=True)


if __name__=='__main__':main()
