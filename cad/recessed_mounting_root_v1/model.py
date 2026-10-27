"""Explicit whole front clamp and connected roots,without moving any drive packet."""
import importlib.util
import sys
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('_arm063_prior',ROOT/'cad/mounting_unit_interface_basis_v1/model.py')
p=importlib.util.module_from_spec(spec);spec.loader.exec_module(p)
n,np,cq,d=p.n,p.np,p.cq,p.d
CARRIER='arm063_recessed_clamp_connected_carrier'
CAP='arm063_recessed_front_cap'
OLD={p.p.NAME,*p.UNIT_NAMES}
AXES=[-25.5,25.5]
Y=-162.5


def context():
    rows,membership,source=p.context()
    for f in ['cad/mounting_unit_interface_basis_v1/reviews/check.json',
              'cad/mounting_unit_interface_basis_v1/reviews/interface_followup.json',
              'cad/central_reaction_frame_v1/reviews/assessment.json']:
        source[f]=n.m.drive.sha(ROOT/f)
    return rows,membership,source


def primitives():
    geom=n.m.drive.module('_arm063_eightbank',ROOT/'cad/eight_bank_layout_v1/model.py')
    return geom,geom.prior.prior.box


def clamp_half(upper):
    geom,box=primitives()
    lo,hi=(0.,25.) if upper else(-25.,0.)
    sh=box([-29,Y-4,lo],[29,Y+4,hi])
    iz0,iz1=(-1.,21.) if upper else(-21.,1.)
    sh=sh.cut(box([-22,Y-5,iz0],[22,Y+5,iz1]))
    for x in AXES:
        sh=sh.cut(geom.prior.prior.cylinder(1.7,52.,[x,Y,-26.]))
        if upper:
            # A nominal hex seat retains the nut at Z4..6.4. The circular
            # upper opening reserves axial access,not a purchased tool fit.
            h=cq.Workplane('XY').polygon(6,6.35).extrude(2.4).val().translate((x,Y,4.))
            sh=sh.cut(h).cut(geom.prior.prior.cylinder(3.175,20.,[x,Y,6.4]))
        else:
            sh=sh.cut(geom.prior.prior.cylinder(2.85,22.,[x,Y,-26.]))
    return sh.clean()


def build(before):
    lookup={z['name']:z for z in before};geom,box=primitives()
    old=lookup[p.p.NAME]['shape']
    # Replace the complete inner front root and lower clamp region,including
    # its frame/thumb bridges. All outer M3 pads and paired outer tie remain.
    removal=geom.prior.cyl_y(42.,48.5,-166.5)
    sh=old.cut(removal,tol=1e-6)
    # Forward outer rails would cross the new head-entry axes. Their whole
    # front lengths are replaced by two inner rails and a rear bottom bridge.
    for sign in [-1.,1.]:
        sh=sh.cut(box([min(sign*24,sign*27),-200.5,-27],
                      [max(sign*24,sign*27),-152.5,-24]),tol=1e-6)
    frame=box([-30,-155.5,-29],[30,-152.5,29]).cut(box([-26,-155.51,-25],[26,-152.49,25]))
    sh=sh.fuse(frame)
    for bank,stem in enumerate(geom.ORDER[1]):
        for t in [-13.,13.]:
            if stem=='middle_dip' and t==-13.:continue
            if stem=='thumb_ip':
                sh=sh.fuse(box([26,-159,t-4],[30,-118,t+4]))
                sh=sh.fuse(box([26,-121,t-4],[44,-118,t+4]))
            else:
                sh=sh.fuse(geom.radial_box([28,-159,t-4],[44,-152.5,t+4],geom.AZIMUTHS[bank]))
    sh=sh.fuse(box([-27,-208.5,-28],[27,-200.5,-24]))
    for sign in [-1.,1.]:
        sh=sh.fuse(box([min(sign*10,sign*13),-208.5,-28],
                      [max(sign*10,sign*13),-152.5,-24]))
    lower=clamp_half(False);upper=clamp_half(True)
    sh=sh.fuse(lower).clean()
    rows=[dict(name=CARRIER,shape=sh,body='forearm',color=[.3,.5,.75],role='whole_front_lower_clamp_and_rebuilt_connected_roots'),
          dict(name=CAP,shape=upper,body='forearm',color=[.3,.5,.75],role='new_recessed_front_cap')]
    transforms=[]
    for i,(oldx,newx) in enumerate(zip([-31.,31.],AXES)):
        M=np.eye(4);M[0,3]=newx-oldx
        for kind in ['M3x12_envelope','M3_nut_envelope']:
            oldname='arm054_'+kind+'_1_'+str(i);newname='arm063_'+kind+'_'+str(i)
            rows.append(dict(name=newname,shape=n.g.original.transform(lookup[oldname]['shape'],M),body='forearm',
                color=[.55,.58,.62],role='existing_fastener_shape_explicitly_repositioned'))
            transforms.append(dict(old_name=oldname,new_name=newname,transform=M))
    for z in rows:assert z['shape'].isValid() and len(z['shape'].Solids())==1,(z['name'],z['shape'].isValid(),len(z['shape'].Solids()))
    entries=[]
    for i,x in enumerate(AXES):
        for side,z0,r in [('head',-26.,2.85),('nut',6.4,3.175)]:
            entries.append(dict(name='arm063_'+side+'_entry_'+str(i),shape=geom.prior.prior.cylinder(r,20.,[x,Y,z0]),
                body='forearm',color=[.85,.48,.2],role='declared_entry_reference_not_installed_or_selected_tool'))
    design=dict(current862_replaced=False,proposal871_adopted=False,new_hardware_rows=6,unchanged_rows=865,proposal_rows=871,
        replaced_names=sorted(OLD),whole_front_unit_station_Y_mm=Y,old_axes_X_mm=[-31.,31.],new_axes_X_mm=AXES,
        explicit_unchanged_fastener_shape_transforms=transforms,front_cap_and_lower_half_replaced_together=True,
        clamp_outer_X_mm=[-29,29],clamp_outer_Z_mm=[-25,25],clamp_axial_thickness_mm=8.,beam_cavity_XZ_mm=[44,42],
        through_bore_diameter_mm=3.4,head_seat_Z_mm=-4.,nut_seat_Z_mm=4.,unchanged_M3x12_shaft_Z_mm=[-4,8],
        lower_counterbore_radius_mm=2.85,upper_hex_circumdiameter_mm=6.35,upper_circular_access_radius_mm=3.175,
        unchanged_entry_length_mm=20.,entry_is_not_selected_socket_bit_handle_or_real_fit=True,
        complete_inner_root_rebuild_radius_mm=42.,complete_inner_root_rebuild_Y_mm=[-166.5,-118.],
        front_normal_root_web_radial_mm=[28,44],front_root_web_Y_mm=[-159,-152.5],
        root_datum='Original front M3 support front faceY=-159. One explicit redesign,not a clearance/angle/radius sweep.',
        thumb_bridge_X_mm=[26,30],thumb_bridge_Y_mm=[-159,-118],thumb_top_bridge_X_mm=[26,44],
        closed_frame_outer_XZ_mm=[60,58],closed_frame_inner_XZ_mm=[52,50],frame_Y_mm=[-155.5,-152.5],
        new_inner_rails_X_mm=[[-13,-10],[10,13]],new_inner_rails_Y_mm=[-208.5,-152.5],new_inner_rails_Z_mm=[-28,-24],
        rear_bottom_bridge_X_mm=[-27,27],rear_bottom_bridge_Y_mm=[-208.5,-200.5],rear_bottom_bridge_Z_mm=[-28,-24],
        original_rear_hub_and_rear_clamp_unit_unchanged=True,original_middle_dip_outer_cross_tie_unchanged=True,
        hand_beam_motor_electrical_or_clock_command_changed=False,mechanical16placements_unchanged=True,
        nominal_load_path='Original16 module M3pairs -> retained outer spokes/paired tie -> rebuilt front roots/frame -> inner lower rails/rear bottom bridge -> new front lower clamp plus original rear clamp/outer rear rails/hub -> split caps/M3 -> unchanged beam. Actual roll output plate/stator fastening remains a separate unfinished interface.',
        same_frame_or_1solid_not_strength_preload_friction_torque_approval=True,
        actual_roll_source_contract='cad/mounting_unit_interface_basis_v1/reviews/interface_followup.json',
        real_roll_plate_beam_stator_candidate_built_this_stage=False)
    return rows,entries,design


def front_pad_inputs(before):
    geom,box=primitives();old=next(z['shape'] for z in before if z['name']==p.p.NAME);pads=[]
    for bank,stem in enumerate(geom.ORDER[1]):
        y=-121. if stem=='thumb_ip' else -162.
        for t in [-13.,13.]:
            region=geom.radial_box([66.5,y,t-4],[75.4,y+3,t+4],geom.AZIMUTHS[bank])
            pads.append(dict(id=stem+'_front_M3_'+str(int(t)),shape=old.intersect(region,tol=1e-6)))
    return pads
