"""Connected central frame around the actual beam, reserving current output space."""
import importlib.util
import sys
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('_arm061_prior',ROOT/'cad/rotation_reserved_support_v1/model.py')
p=importlib.util.module_from_spec(spec)
spec.loader.exec_module(p)
n,np,cq,d=p.n,p.np,p.a.cq,p.d
NAME='arm061_closed_front_frame_carrier'
OLD=p.CARRIER


def context():
    before,original,membership,prior,source,matrices=p.context()
    raw=n.p.read(ROOT/'cad/rotation_reserved_support_v1/reviews/check.json')
    saved=n.c.load(n.p.read(ROOT/raw['scene'])['parts'],source)
    rows=[z for z in before if z['name'] not in p.OLD]+saved
    assert len(rows)==871
    membership[NAME]='forearm'
    source.update({rel:n.m.drive.sha(ROOT/rel) for rel in [
        'cad/rotation_reserved_support_v1/reviews/check.json',
        'cad/rotation_reserved_support_v1/reviews/hub_followup.json',
        'cad/eight_bank_layout_v1/model.py','cad/arm_layout_proposal_v1/proposal.py',
        'cad/modular_roots_v9/build_candidate.py','cad/modular_roots_v9/design.json']})
    return rows,membership,raw,source


def build(rows):
    lookup={z['name']:z for z in rows}
    geom=n.m.drive.module('_arm061_eightbank',ROOT/'cad/eight_bank_layout_v1/model.py')
    box=geom.prior.prior.box
    radial=lambda lo,hi,angle:geom.radial_box(lo,hi,angle)
    old=lookup[OLD]['shape']
    ring=geom.prior.cyl_y(38.,3.,-162.).cut(geom.prior.cyl_y(34.,3.,-162.))
    sh=old.cut(ring,tol=1e-6)
    # The old thumb axial bridges and middle-DIP inner stump occupy the
    # protected output volume. Their loads are connected by new full webs.
    for t in [-13.,13.]: sh=sh.cut(box([34,-162,t-4],[38,-118,t+4]),tol=1e-6)
    sh=sh.cut(radial([34,-162,-17],[38,-159,-9],45.),tol=1e-6)
    frame_y0=-158.5+3.
    frame_y1=frame_y0+3.
    frame=box([-30,frame_y0,-29],[30,frame_y1,29]).cut(
        box([-26,frame_y0-.01,-25],[26,frame_y1+.01,25]))
    sh=sh.fuse(frame)
    # Full, declared webs connect original front spokes to the closed frame.
    # Omit the middle-DIP -13 branch already routed into its paired spoke.
    webs=[]
    for bank,stem in enumerate(geom.ORDER[1]):
        angle=geom.AZIMUTHS[bank]
        for t in [-13.,13.]:
            if stem=='middle_dip' and t==-13.: continue
            if stem=='thumb_ip':
                bridge=box([26,-162,t-4],[30,-118,t+4])
                top=box([26,-121,t-4],[38,-118,t+4])
                sh=sh.fuse(bridge).fuse(top)
                webs.append(dict(module=stem,tangent=t,radial_mm=[26,30],Y_mm=[-162,-118],original_M3_seat_plane_unchanged=True))
            else:
                sh=sh.fuse(radial([28,-162,t-4],[38,frame_y1,t+4],angle))
                webs.append(dict(module=stem,tangent=t,radial_mm=[28,38],Y_mm=[-162,frame_y1],original_M3_seat_plane_unchanged=True))
    for sign in [-1.,1.]:
        sh=sh.fuse(box([min(sign*24,sign*27),-232,-27],
            [max(sign*24,sign*27),frame_y1,-24]))
    # Restore the complete unchanged lower collars after the annular Boolean
    # replacement. No beam/cap/hardware material is cut or moved.
    for y in geom.STATIONS: sh=sh.fuse(geom.prior.prior.collar(y,False))
    sh=sh.clean()
    assert sh.isValid() and len(sh.Solids())==1,('newcarrier',sh.isValid(),len(sh.Solids()))
    row=dict(name=NAME,shape=sh,body='forearm',color=[.3,.5,.75],
        role='connected_rectangular_front_frame_axial_webs_and_retained_split_collars')
    design=dict(current862_replaced=False,proposal_adopted=False,proposal_rows=871,new_hardware_rows=1,unchanged_rows=870,
        replaced_name=OLD,motor_installation_clock_commands_or_electrical_positions_changed=False,
        accepted_hand_axes_lengths_changed=False,ARM060_wrist_entry_and_ARM059_mechanical_placements_unchanged=True,
        beam_source_outer_mm=[44,42],beam_source_wall_mm=5,
        original_lower_collars_Y_mm=geom.STATIONS,original_cap_front_Y_mm=-158.5,
        closed_frame_outer_XZ_mm=[60,58],closed_frame_inner_XZ_mm=[52,50],frame_wall_mm=4.,frame_axial_thickness_mm=3.,
        frame_Y_mm=[frame_y0,frame_y1],datum='One candidate:existing collar frontY-158.5 plus original3mm web thickness. Not minimum clearance or assembly allowance.',
        original_annulus_radii_mm=[34,38],original_front_hub_Y_mm=[-162,-159],
        thumb_axial_web_X_mm=[26,30],webs=webs,original_middle_dip_outer_cross_tie_preserved=True,
        lower_rails_XZ_mm=[[[24,-27],[27,-24]],[[-27,-27],[-24,-24]]],lower_rails_Y_mm=[-232,frame_y1],
        preserved_module_M3_axes_and_outer_pads=32,original_rear_hub_unchanged=True,
        nominal_load_path='16 actual adapter M3 pairs/outer pads -> front spokes and shifted thumb axial webs -> closed rectangular front frame -> two lower rails/original rear hub -> two retained lower collars/caps/M3 stack -> actual forearm beam. Beam/roll flange/motor output interfaces separately measured,not assumed torque-connected.',
        frame_and_old_ring_nominal_web_section_mm2=[12.,12.],equal_nominal_section_not_equal_strength=True,
        physical_strength_preload_friction_tool_or_assembly_qualified=False)
    return row,design


def seat_inputs(rows):
    geom=n.m.drive.module('_arm061_seat_basis',ROOT/'cad/eight_bank_layout_v1/model.py')
    old=next(z['shape'] for z in rows if z['name']==OLD)
    seats=[]
    for j,names in enumerate(geom.ORDER):
        for bank,stem in enumerate(names):
            y=geom.ROWS[j]+19. if stem=='thumb_ip' else geom.ROWS[j]-22.
            for t in [-13.,13.]:
                region=geom.radial_box([66.5,y,t-4],[75.4,y+3,t+4],geom.AZIMUTHS[bank])
                seats.append(dict(id=stem+'_M3_pad_'+str(int(t)),shape=old.intersect(region,tol=1e-6)))
    for i,y in enumerate(geom.STATIONS):
        seats.append(dict(id='lower_beam_collar_'+str(i),shape=old.intersect(geom.prior.prior.collar(y,False),tol=1e-6)))
    assert len(seats)==34
    return seats


def reaction_interfaces(rows):
    lookup={z['name']:z['shape'] for z in rows}
    pairs=[('beam__roll_flange','Forearm_box_beam','Roll_output_flange'),
           ('roll_flange__nominal_motor_case','Roll_output_flange','AK_forearm_roll')]
    result=[]
    for label,l,r in pairs:
        common=lookup[l].intersect(lookup[r],tol=1e-6)
        assert common.isValid()
        result.append(dict(id=label,left=l,right=r,distance_mm=lookup[l].distance(lookup[r]),
            common_volume_mm3=common.Volume() if common.Solids() else 0.,common_area_mm2=common.Area() if common.Faces() else 0.,
            common_solids=len(common.Solids()),common_faces=len(common.Faces()),
            body='forearm' if l=='Forearm_box_beam' else 'forearm_to_elbow',
            actual_output_hub_bolts_spline_or_torque_path_qualified=False))
    return result
