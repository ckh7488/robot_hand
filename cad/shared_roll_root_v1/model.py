"""One shared output/beam/rack root replaces both rear clamp functions explicitly."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('_arm065_prior',ROOT/'cad/roll_bolted_interface_v1/model.py')
p=importlib.util.module_from_spec(spec);spec.loader.exec_module(p)
n,np,cq,d=p.n,p.np,p.cq,p.d
NAME='arm065_shared_output_beam_rack_root';CAP='arm065_shared_rear_cap'
REAR=['arm054_beam_cap_0']+['arm054_'+k+'_0_'+str(i) for i in range(2) for k in ['M3x12_envelope','M3_nut_envelope']]
OLD={p.a.CARRIER,*REAR,'arm064_output_plate_lower_beam_clamp','arm064_upper_beam_clamp'}


def context():
    rows,membership,source=p.context();raw=n.p.read(ROOT/'cad/roll_bolted_interface_v1/reviews/check.json')
    saved=n.c.load(n.p.read(ROOT/raw['scene'])['parts'],source)
    rows=[z for z in rows if z['name'] not in p.OLD]+saved
    assert len(rows)==888 and len(OLD)==8
    for z in saved:membership[z['name']]=z['body']
    return rows,membership,source


def region():
    # Whole rear inner hub, original lower collar/ears, complete old rails,
    # and shared clamp bolt counterbores, not isolated overlap corners.
    sh=p.cy(42,-232.01,-226)
    sh=sh.fuse(p.box([-36,-208.5,-25],[36,-200.5,.01]))
    for s in [-1,1]:sh=sh.fuse(p.box([min(s*24,s*27),-232,-27],[max(s*24,s*27),-200.5,-24]))
    for y in [-219.,-204.5]:
        for x in [-25.5,25.5]:sh=sh.fuse(p.cz(2.85,-28,-4,x,y))
    return sh


def rear_pads(before):
    geom=n.m.drive.module('_arm065_eightbank',ROOT/'cad/eight_bank_layout_v1/model.py')
    old=next(z['shape'] for z in before if z['name']==p.a.CARRIER);out=[]
    for bank,stem in enumerate(geom.ORDER[0]):
        for t in [-13.,13.]:
            sh=geom.radial_box([66.5,-232,t-4],[75.4,-229,t+4],geom.AZIMUTHS[bank])
            out.append(dict(id=stem+'_rear_M3_'+str(int(t)),shape=old.intersect(sh,tol=1e-6)))
    return out


def build(before):
    lookup={z['name']:z for z in before};geom=n.m.drive.module('_arm065_primitives',ROOT/'cad/eight_bank_layout_v1/model.py')
    old=lookup[p.a.CARRIER]['shape'];removal=region();carrier=old.cut(removal,tol=1e-6)
    plate=lookup['arm064_output_plate_lower_beam_clamp']['shape'].intersect(p.box([-40,-232,-40],[40,-226,40]),tol=1e-6)
    lower=p.box([-29,-226,-27],[29,-200.5,-.1]).cut(p.box([-22,-226.01,-21],[22,-200.49,.2]))
    cap=p.box([-29,-226,.1],[29,-200.5,27]).cut(p.box([-22,-226.01,-.2],[22,-200.49,21]))
    carrier=carrier.fuse(plate).fuse(lower).fuse(p.cy(38,-230,-226).cut(p.cy(30,-230.01,-225.99)))
    for angle in geom.AZIMUTHS:
        for t in [-13.,13.]:
            carrier=carrier.fuse(geom.radial_box([34,-230,t-4],[44,-227,t+4],angle))
    for y in [-219.,-204.5]:
        for x in [-25.5,25.5]:
            carrier=carrier.cut(p.cz(1.7,-28,28,x,y)).cut(p.cz(2.85,-28,-4,x,y))
            cap=cap.cut(p.cz(1.7,-28,28,x,y)).cut(p.cz(3.175,6.4,28,x,y))
            cap=cap.cut(cq.Workplane('XY').polygon(6,6.35).extrude(2.4).val().translate((x,y,4)))
    rows=[dict(name=NAME,shape=carrier.clean(),body='forearm',color=[.28,.52,.75],role='single output plate+lower beam clamp+rear rack root;front clamp/root retained'),
        dict(name=CAP,shape=cap.clean(),body='forearm',color=[.28,.52,.75],role='one shared removable cap with two bolt stations replacing separate rear caps')]
    moves=[]
    for i,(oldx,newx) in enumerate(zip([-31.,31.],[-25.5,25.5])):
        M=np.eye(4);M[0,3]=newx-oldx
        for k in ['M3x12_envelope','M3_nut_envelope']:
            oldname='arm054_'+k+'_0_'+str(i);name='arm065_rear_'+k+'_'+str(i)
            rows.append(dict(name=name,shape=n.g.original.transform(lookup[oldname]['shape'],M),body='forearm',color=[.63,.65,.68],role='same rear fastener shape,explicit X translation;Y-204.5 unchanged'))
            moves.append(dict(old_name=oldname,new_name=name,transform=M))
    for z in rows:assert z['shape'].isValid() and len(z['shape'].Solids())==1,(z['name'],z['shape'].isValid(),len(z['shape'].Solids()))
    entry=[]
    for i,x in enumerate([-25.5,25.5]):
        for side,z0,z1,r in [('head',-26,-6,2.85),('nut',6.4,26.4,3.175)]:
            entry.append(dict(name='arm065_rear_'+side+'_entry_'+str(i),shape=p.cz(r,z0,z1,x,-204.5),body='forearm',color=[.9,.54,.17],role='20mm head/nut continuation,not selected tool'))
    raw=n.p.read(ROOT/'cad/roll_bolted_interface_v1/reviews/check.json')
    saved=n.c.load(n.p.read(ROOT/raw['entry_scene'])['parts'])
    inherited=[z for z in saved if z['name'].startswith('arm064_clamp_')]
    assert len(rows)==6 and len(entry)==4 and len(inherited)==4
    design=dict(current862_replaced=False,proposal886_adopted=False,proposal_rows=886,unchanged_rows=880,new_rows=6,
        replaced_names=sorted(OLD),explicit_same_shape_rear_fastener_moves=moves,
        merged_function='Two lower functions(carrier/output plate) become one root;two removable rear caps become one shared cap. Both M3 stations and all four rear old fasteners survive explicitly,not actuator deletion.',
        removed_old_rear_collar_station_Y_mm=-204.5,old_rear_fastener_X_mm=[-31,31],new_rear_fastener_X_mm=[-25.5,25.5],
        shared_clamp_Y_mm=[-226,-200.5],shared_outer_XZ_mm=[58,54],beam_cavity_XZ_mm=[44,42],bolt_stations_Y_mm=[-219,-204.5],
        complete_rear_inner_root_radius_mm=42,rear_inner_root_Y_mm=[-232.01,-226],
        new_rear_ring_radial_mm=[30,38],new_rear_ring_Y_mm=[-230,-226],
        rear_inner_spoke_radial_mm=[34,44],rear_inner_spoke_Y_mm=[-230,-227],
        original_outer_rear_spokes_and_16_M3pads_retained=True,completed_front_unit_not_redesigned=True,
        rear_root_datum='Actual output plate rear planeY=-230. Inner3mm spokes attach there and overlap original outer spokes overY[-230,-229];original rear hubY[-232,-229] withinR42 replaced as a unit.',
        output_interface_plate_pilot_6M3_geometry_from_ARM064_preserved=True,output_interface_or_case_axes_reclassified=False,
        stator_E_and_all_motor_axes_clock_commands_electrical_beam_hand_unchanged=True,
        nominal_path='16 motor M3pads->retained outer spokes->new inner rear hub/output ring->single output/beam root->actual output6M3;beam rectangular collar/cap with4M3 over two stations and unchanged front clamp share rack/beam reaction.',
        axial_retention='Y-226 beam rear-end stop is one-sided. Opposite pullout still depends on unverified clamp preload/friction. No positive two-sided axial capture or torque/strength qualification claimed.',
        cavity_split_gap_mm=.2,entry_reference_length_mm=20,entry_reference_not_selected_tools=True,
        old_154mm3_overlap_not_deleted_or_renamed_as_pass=True,geometry_physically_replaced_by_shared_root=True)
    return rows,entry,inherited,design,removal,plate
