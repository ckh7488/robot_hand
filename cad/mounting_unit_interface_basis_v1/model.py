"""Whole front mounting unit and source-derived roll output interface,reference only."""
import importlib.util
import sys
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('_arm062_prior',ROOT/'cad/central_reaction_frame_v1/model.py')
p=importlib.util.module_from_spec(spec);spec.loader.exec_module(p)
n,np,cq,d=p.n,p.np,p.cq,p.d
SOURCE='cad/modular_roots_v9/sources/AK60.step'
UNIT_NAMES=['arm054_beam_cap_1']+['arm054_'+kind+'_1_'+str(i) for i in range(2) for kind in ['M3x12_envelope','M3_nut_envelope']]


def context():
    rows,membership,prior,source=p.context()
    raw=n.p.read(ROOT/'cad/central_reaction_frame_v1/reviews/check.json')
    saved=n.c.load(n.p.read(ROOT/raw['scene'])['parts'],source)
    rows=[z for z in rows if z['name']!=p.OLD]+saved
    assert len(rows)==871
    membership[p.NAME]='forearm'
    return rows,membership,source


def mounting_unit(rows):
    lookup={z['name']:z for z in rows}
    geom=n.m.drive.module('_arm062_collar',ROOT/'cad/eight_bank_layout_v1/model.py')
    # The lower half is a functional region of the existing carrier. ARM061's
    # saved material-preservation query is the provenance,not a new cut/retest.
    lower=geom.prior.prior.collar(-162.5,False)
    unit=[dict(name='arm062_existing_front_lower_collar_region',shape=lower,body='forearm',color=[.3,.5,.75])]
    unit += [dict(lookup[name],name='arm062_reference_'+name) for name in UNIT_NAMES]
    entry=[]
    for i,x in enumerate([-31.,31.]):
        for side,z0,rad in [('head',-26.,2.85),('nut',6.4,3.175)]:
            entry.append(dict(name='arm062_'+side+'_entry_'+str(i),
                shape=cq.Solid.makeCylinder(rad,20.,cq.Vector(x,-162.5,z0),cq.Vector(0,0,1)),
                body='forearm',color=[.85,.48,.2],role='reference_entry_space_not_installed_or_selected_tool'))
    assert len(unit)==6 and len(entry)==4
    contract=dict(unit_functional_regions=6,separate_original_hardware_rows=5,lower_collar_part_of_existing_carrier=True,
        lower_collar_preservation_inherited_from='cad/central_reaction_frame_v1/reviews/check.json:seat_queries/lower_beam_collar_1',
        cap_and_M3_names=UNIT_NAMES,bolt_axes_X_mm=[-31.,31.],bolt_axes_Y_mm=-162.5,
        bolt_shaft_Z_mm=[-4.,8.],bolt_head_Z_mm=[-6.,-4.],nut_Z_mm=[4.,6.4],
        entry_directions={'head':[0,0,-1],'nut':[0,0,1]},entry_length_mm=20.,
        entry_radii_mm={'head':2.85,'nut':3.175},
        entry_basis='Bare axial continuation of actual head/nut maximum cross section for20mm. A declared test assumption,not a selected socket/bit/handle envelope or necessary access length.',
        actual_tool_selected=False,entry_pass_not_actual_assembly_approval=True,
        existing_mounting_unit_moved=False,new_installed_hardware=0,carrier_or_motor_geometry_changed=False)
    return unit,entry,contract


def vendor_interface():
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Cylinder,GeomAbs_Plane
    src=cq.importers.importStep(str(ROOT/SOURCE)).val()
    cylinders=[];planes=[]
    for i,f in enumerate(src.Faces()):
        a=BRepAdaptor_Surface(f.wrapped,True)
        if a.GetType()==GeomAbs_Cylinder:
            q=a.Cylinder();v=q.Axis().Direction();c=q.Location()
            cylinders.append(dict(index=i,radius=q.Radius(),axis=[v.X(),v.Y(),v.Z()],
                centre=[c.X(),c.Y(),c.Z()],bbox=n.c.m.pack._bbox(f),area_mm2=f.Area()))
        elif a.GetType()==GeomAbs_Plane:
            q=a.Plane();v=q.Axis().Direction();c=q.Location()
            if abs(v.Z())>.999999:
                planes.append(dict(index=i,z=c.Z(),area_mm2=f.Area(),bbox=n.c.m.pack._bbox(f)))
    main=next(z for z in cylinders if abs(z['radius']-39.5)<1e-7)
    centre=np.array(main['centre'][:2]);bb=np.array(n.c.m.pack._bbox(src))
    # Drawing identifies the small protruding output side as the low-Z end.
    # Output face is the retained joint datum,Y=-230. The unverified roll
    # clock is kept as the exact vendor clock;this is a reference registration.
    front=float(bb[0,2])
    M=np.array([[1.,0.,0.,-centre[0]],[0.,0.,-1.,-230.+front],[0.,1.,0.,-centre[1]],[0.,0.,0.,1.]])
    face_rows=[]
    for q in cylinders:
        rad=float(np.linalg.norm(np.array(q['centre'][:2])-centre))
        q['axis_radius_from_motor_centre_mm']=rad
        if abs(q['radius']-1.5)<1e-6 and abs(rad-10.)<1e-5 and np.min(np.array(q['bbox'])[:,2])<front+6.001:
            face_rows.append(q)
    # STEP splits some cylinders. Group the exact analytic axes,keeping all
    # source face IDs and their trim bounds;do not count seams as extra holes.
    holes={}
    for q in face_rows:
        key=tuple(round(v,7) for v in q['centre'][:2])
        holes.setdefault(key,[]).append(q)
    output=[]
    for key,fs in sorted(holes.items()):
        point=np.r_[fs[0]['centre'][:2],front,1.]
        output.append(dict(source_faces=[z['index'] for z in fs],source_axis_xy_mm=fs[0]['centre'][:2],
            radial_mm=float(np.linalg.norm(np.array(fs[0]['centre'][:2])-centre)),radius_mm=fs[0]['radius'],
            trim_Z_mm=[min(z['bbox'][0][2] for z in fs),max(z['bbox'][1][2] for z in fs)],
            world_front_axis_point_mm=(M@point)[:3].tolist()))
    face_at_front=[z for z in planes if abs(z['z']-front)<1e-5]
    # Preserve open source topology. No sewing/filling or solid material claim.
    world=n.g.original.transform(src,M)
    row=dict(name='arm062_vendor_AK60_output_datum_reference',shape=world,body='elbow',color=[.42,.43,.46],
        role='unadopted_open_vendor_surface_reference_not_native_solid_obstacle')
    result=dict(source_step=SOURCE,source_sha256=n.m.drive.sha(ROOT/SOURCE),source_valid=src.isValid(),source_solids=len(src.Solids()),
        source_shells=len(src.Shells()),source_faces=len(src.Faces()),source_bbox_mm=bb,source_motor_axis_xy_mm=centre,
        source_output_front_Z_mm=front,source_output_axis=[0,0,-1],world_output_datum_mm=[0,-230,0],
        world_output_axis=[0,1,0],reference_transform=M,world_bbox_mm=n.c.m.pack._bbox(world),
        output_M3_cylinder_axes=output,output_front_planes=face_at_front,
        cylinders=cylinders,normal_planes=planes,source_not_repaired_or_solidified=True,
        drawing_interpretation=dict(output_pilot_diameter_mm=25,output_M3_count=6,output_M3_PCD_mm=20,output_M3_depth_mm=6,
            case_M3_PCD_mm=68,front_case_M3_depth_mm=6,rear_case_M3_depth_mm=3.5,
            overall_length_mm=43,case_length_mm=31,output_dowels_count=2,output_dowel_diameter_mm=3,
            output_dowel_plus_tolerance_mm=.02,output_dowel_depth_mm=3,
            drawing_has_no_explicit_part_role_text=True,output_side_identification_is_engineering_inference=True),
        actual_motor_mount_clock_not_verified=True,installed_motor_replaced=False,torque_coupling_designed=False)
    return row,result
