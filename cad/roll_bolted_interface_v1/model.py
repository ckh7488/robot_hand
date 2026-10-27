"""Explicit E-stator and EF-output mounting proposal; current installation unchanged."""
import importlib.util
import sys
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('_arm064_prior',ROOT/'cad/recessed_mounting_root_v1/selected_model.py')
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
n,np,cq,d=a.n,a.np,a.cq,a.d
OLD=['Elbow_roll_carrier','Roll_output_flange']


def context():
    rows,membership,source=a.context()
    first=n.p.read(ROOT/'cad/recessed_mounting_root_v1/reviews/check.json')
    selected=n.p.read(ROOT/'cad/recessed_mounting_root_v1/reviews/selected_check.json')
    initial=n.c.load(n.p.read(ROOT/first['scene'])['parts'],source)
    changed=n.c.load(n.p.read(ROOT/selected['scene'])['parts'],source)
    rows=[z for z in rows if z['name'] not in a.OLD]+[z for z in initial if z['name']!=a.CARRIER]+changed
    assert len(rows)==871
    for z in initial+changed:membership[z['name']]='forearm'
    return rows,membership,source


def box(lo,hi):
    return cq.Solid.makeBox(*(np.array(hi)-lo),cq.Vector(*lo))


def cy(r,y0,y1,x=0.,z=0.):
    return cq.Solid.makeCylinder(r,y1-y0,cq.Vector(x,y0,z),cq.Vector(0,1,0))


def cz(r,z0,z1,x,y):
    return cq.Solid.makeCylinder(r,z1-z0,cq.Vector(x,y,z0),cq.Vector(0,0,1))


def contract():
    raw=n.p.read(ROOT/'cad/mounting_unit_interface_basis_v1/reviews/check.json')['vendor_interface']
    fixed=n.p.read(ROOT/'cad/mounting_unit_interface_basis_v1/reviews/interface_followup.json')
    M=np.array(fixed['exact_plane_reference_transform'])
    taps=fixed['corrected_output_M3_taps']
    # Read only the six cached rear case axes, not the open STEP's 615 faces.
    rear=[q for q in raw['cylinders'] if q['index'] in [59,60,61,62,63,64]]
    def point(q):return (M@np.r_[q['centre'],1.])[:3]
    stator=[dict(face=q['index'],point=point(q).tolist(),radius=q['radius']) for q in rear]
    output=[dict(point=q['world_front_axis_point_mm']) for q in taps]
    assert len(stator)==len(output)==6
    return dict(exact_reference_transform=M,stator_axes=stator,output_axes=output,
        rear_case_plane_Y_mm=-261.,case_PCD_mm=68.,drawing_rear_M3_depth_mm=3.5,
        output_face_Y_mm=-230.,output_PCD_mm=20.,drawing_output_M3_depth_mm=6.,output_pilot_diameter_mm=25.,
        clock='Exact cached vendor source clock, unchanged from ARM062 reference. Physical clock and stator identification remain unqualified.',
        registered_source_not_installed_motor_relocation=True,source615faces_reclassified_or_sewn=False)


def build(before):
    c=contract();lookup={z['name']:z for z in before};rows=[];entry=[]
    def add(name,shape,frame,color,role):
        shape=shape.clean();assert shape.isValid() and len(shape.Solids())==1,(name,shape.isValid(),len(shape.Solids()))
        rows.append(dict(name='arm064_'+name,shape=shape,body=frame,color=color,role=role))
    E=[.48,.49,.53];EF=[.25,.52,.77];bolt=[.72,.73,.76]
    stator=lookup['Elbow_roll_carrier']['shape']
    for q in c['stator_axes']:
        x,y,z=q['point'];assert abs(y+261)<1e-8
        stator=stator.fuse(cy(4.5,-277,-261,x,z))
    for q in c['stator_axes']:
        x,_,z=q['point'];stator=stator.cut(cy(1.7,-283.01,-260.99,x,z),tol=1e-6)
    add('stator_carrier_with_six_posts',stator,'elbow',E,'original elbow carrier plus six16mm posts to actual rear case mounting plane;6clearance bores')
    # Datum plane seats the existing beam end. A shallow lip locates the
    # drawing's 25mm pilot with an explicitly unqualified diametral clearance.
    plate=cy(32,-230,-226).fuse(cy(14.5,-231.25,-230).cut(cy(12.6,-231.26,-229.99)))
    lower=box([-29,-226,-27],[29,-212,-.1]).cut(box([-22,-226.01,-21],[22,-211.99,.2]))
    cap=box([-29,-226,.1],[29,-212,27]).cut(box([-22,-226.01,-.2],[22,-211.99,21]))
    for q in c['output_axes']:
        x,_,z=q['point'];plate=plate.cut(cy(1.7,-230.01,-225.99,x,z),tol=1e-6)
    raw=n.p.read(ROOT/'cad/mounting_unit_interface_basis_v1/reviews/interface_followup.json')
    for q in raw['corrected_output_dowels']:
        x,_,z=q['world_front_axis_point_mm'];plate=plate.cut(cy(1.6,-230.01,-225.99,x,z),tol=1e-6)
    for x in [-25.5,25.5]:
        lower=lower.cut(cz(1.7,-28,28,x,-219)).cut(cz(2.85,-28,-4,x,-219))
        cap=cap.cut(cz(1.7,-28,28,x,-219)).cut(cz(3.175,6.4,28,x,-219))
        hx=cq.Workplane('XY').polygon(6,6.35).extrude(2.4).val().translate((x,-219,4))
        cap=cap.cut(hx)
    add('output_plate_lower_beam_clamp',plate.fuse(lower),'forearm',EF,'6M3 output plate plus pilot lip and lower square beam collar;one-sided positive axial beam end seat')
    add('upper_beam_clamp',cap,'forearm',EF,'removable split cap;opposite axial pullout depends on unverified clamp friction')
    for i,q in enumerate(c['stator_axes']):
        x,_,z=q['point']
        add('stator_M3x25_'+str(i),cy(1.5,-283,-258,x,z).fuse(cy(2.85,-286,-283,x,z)),'elbow',bolt,'unthreaded reference;22mm stack+3mm nominal drawing-depth engagement')
        entry.append(dict(name='arm064_stator_head_entry_'+str(i),shape=cy(2.85,-306,-286,x,z),body='elbow',color=[.9,.54,.17]))
    for i,q in enumerate(c['output_axes']):
        x,_,z=q['point']
        add('output_M3x8_'+str(i),cy(1.5,-234,-226,x,z).fuse(cy(2.85,-226,-223,x,z)),'forearm',bolt,'unthreaded reference;4mm plate+4mm nominal drawing-depth engagement')
        entry.append(dict(name='arm064_output_head_entry_'+str(i),shape=cy(2.85,-223,-203,x,z),body='forearm',color=[.9,.54,.17]))
    for i,x in enumerate([-25.5,25.5]):
        add('clamp_M3x12_'+str(i),cz(1.5,-4,8,x,-219).fuse(cz(2.85,-6,-4,x,-219)),'forearm',bolt,'same nominal M3x12 envelope stack as ARM063;not supply fit')
        nut=cq.Workplane('XY').polygon(6,6.35).extrude(2.4).val().translate((x,-219,4)).cut(cz(1.5,3.99,6.41,x,-219))
        add('clamp_nut_'+str(i),nut,'forearm',bolt,'unthreaded ring reference,coincident nominal shaft bore;not real thread/preload')
        for label,z0,z1,rad in [('head',-26,-6,2.85),('nut',6.4,26.4,3.175)]:
            entry.append(dict(name='arm064_clamp_'+label+'_entry_'+str(i),shape=cz(rad,z0,z1,x,-219),body='forearm',color=[.9,.54,.17]))
    reserved=[dict(name='arm064_source_case_outer_reference',shape=cy(39.5,-261,-231.5),body='elbow',color=E),
        dict(name='arm064_source_driver_outer_reference',shape=cy(28.5,-273,-261),body='elbow',color=E)]
    c.update(replaced_proposal_rows=OLD,new_hardware_rows=19,proposal_rows=888,current_installed_rows=862,
        stator_post_radius_mm=4.5,stator_posts_Y_mm=[-277,-261],stator_bore_diameter_mm=3.4,
        stator_M3x25_stack_and_nominal_engagement_mm=[22,3],output_M3x8_stack_and_nominal_engagement_mm=[4,4],
        output_plate_Y_mm=[-230,-226],pilot_lip_Y_mm=[-231.25,-230],pilot_female_diameter_mm=25.2,
        output_dowel_clearance_diameter_mm=3.2,output_bore_diameter_mm=3.4,
        clamp_outer_XZ_mm=[58,54],clamp_Y_mm=[-226,-212],clamp_cavity_XZ_mm=[44,42],split_gap_mm=.2,
        clamp_axes_X_mm=[-25.5,25.5],clamp_axis_Y_mm=-219.,clamp_head_seat_Z_mm=-4.,clamp_nut_seat_Z_mm=4.,
        beam_material_or_holes_changed=False,motor_deleted_or_moved=False,motorcommands_or_clock_changed=False,
        positive_axial_capture='Beam rear end seats on plate atY=-226 in one direction only. Opposite axial pullout relies on unverified clamp friction;no full six-DOF/torque approval.',
        torque_path='E:elbow carrier->six posts/M3->rear stator face. EF:actual output6M3->plate->square collar/cap/M3->uncut44x42beam;rectangular form bears roll,preload/friction/strength unqualified.',
        pilot_depth_basis='Cached face124 cylinder trim starts sourceZ=-15,0.5mm behind output front. Lip extends1.25mm behind front and stops0.25mm short of nominal actual case front. Fit and alltrimface interference unqualified.',
        tool_reference='16declared20mm axial continuations of head/nut cross sections,not selected bit/socket/handle or required service length.',
        source_outer_references='Two closed external stepped cylindrical references,not repaired vendor motor solids. Do not infer thread clearance from filled outer cylinders.',
        planned_assembly=['Bench:attach six-post E carrier to actual rear stator face with6M3x25 while output/beam group absent.',
            'Bench:attach output plate/lower collar to actual output with6M3x8;20mm head approach faces+Y.',
            'Introduce existing beam toward-Y until rear end seats atY=-226,then upper cap/nuts/M3;beam remains uncut.',
            'Integrate ARM063 rack root only after neutral hardware and access contradictions resolved.'],
        assembly_or_thread_torque_qualified=False,complete_axial_retention_qualified=False)
    assert len(rows)==19 and len(entry)==16 and len(reserved)==2
    return rows,entry,reserved,c
