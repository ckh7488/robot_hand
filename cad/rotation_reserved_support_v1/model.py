"""Outer paired-spoke load path and handward wrist entry, as a separate proposal."""
import importlib.util
import sys
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('_arm060_prior_post',ROOT/'cad/integrated_motor_end_v1/postprocess.py')
p=importlib.util.module_from_spec(spec)
spec.loader.exec_module(p)
a,n,np,d,cq=p.a,p.n,p.np,p.d,p.a.cq
CARRIER='arm060_outer_cross_tie_carrier'
GUIDE='arm060_handward_straight_wrist_entry'
OLD={a.CARRIER,'compact_saddle_nested_turns'}
OUTPUT_NAMES=a.a.CLOCK_NAMES|{'thumb_ip_stock_output_horn','thumb_ip_stock_central_retainer'}


def context():
    raw=n.p.read(ROOT/'cad/integrated_motor_end_v1/reviews/check.json')
    current,before,meta,membership,delta,reloc,clock,shift=p.metadata(raw)
    source=dict(raw['source_sha256'])
    moved=set(raw['design']['moved_names'])
    for z in meta:
        if 'step' not in z:
            rel='cad/modular_roots_v9/parts/'+z['name']+'.step'
            z.update(step=rel,sha256_step=source[rel])
    rows=[]
    matrices={}
    for z in n.c.load(meta,source):
        name=z['name']
        M=np.array(delta.get(name,np.eye(4)))
        if name in reloc: M=reloc[name]@M
        if name in a.a.CLOCK_NAMES: M=clock@M
        if name in moved: M=shift@M
        matrices[name]=M
        rows.append(dict(z,shape=n.g.original.transform(z['shape'],M)))
    old=next(z for z in before if z['name']=='arm054_eight_bank_carrier')
    original=n.c.load([old],source)[0]
    original['shape']=n.g.original.transform(original['shape'],matrices.get(old['name'],np.eye(4)))
    for rel in ['cad/integrated_motor_end_v1/reviews/check.json',
                'cad/integrated_motor_end_v1/reviews/mechanical_followup.json',
                'cad/motor_spool_coupling_v1/candidate2.py',
                'cad/motor_spool_coupling_v1/selected/reviews/check.json']:
        source[rel]=n.m.drive.sha(ROOT/rel)
    assert len(rows)==871 and len(current)==862
    membership[GUIDE]='hand'
    membership[CARRIER]='forearm'
    return rows,original,membership,raw,source,matrices


def build(original):
    R=d.around((0.,0.,0.),(0.,1.,0.),-45.)
    local=lambda lo,hi:n.g.original.transform(a.a.box(lo,hi),R)
    # Route the outer -13 pad into its paired +13 spoke in the original seat
    # plane. No inner axial web crosses the thumb-output rotational volume.
    carrier=original['shape'].cut(local([38.,-162.,-17.],[66.5,-159.,-9.]),tol=1e-6)
    carrier=carrier.fuse(local([66.5,-162.,-13.],[69.5,-159.,17.])).clean()
    assert carrier.isValid() and len(carrier.Solids())==1
    feed=n.m.drive.module('_arm060_feed',ROOT/'cad/wrist_external_feed_v1/feed_model.py')
    box,cyl=feed.box,feed.cyl
    guide=box([45,-9,-6],[62,-6,6]).fuse(box([54,-12,-6],[102,-9,4]))
    guide=guide.fuse(box([54,-9,-58],[62,-6,6])).fuse(box([45,-9,-58],[62,-6,-46]))
    guide=guide.fuse(box([54,-6,-58],[56,0,-46]))
    for z in [-3.,2.]: guide=guide.fuse(a.a.cy(2.5,-14.,-4.,100.,z))
    for z in [-3.,2.]: guide=guide.cut(a.a.cy(1.6,-14.01,-3.99,100.,z))
    for z in [-55.,-49.]: guide=guide.cut(cyl(59,z,-14,6,1.1))
    guide=guide.clean()
    assert guide.isValid() and len(guide.Solids())==1
    rows=[dict(name=CARRIER,shape=carrier,body='forearm',color=[.3,.5,.75],
        role='outer_tangent_cross_tie_into_paired_spoke_original_M3_plane'),
        dict(name=GUIDE,shape=guide,body='palm',color=[.93,.48,.1],
        role='handward_straight_pair_entry_on_existing_saddle_clamp',attachment_body='Palm_wrist_saddle')]
    design=dict(current862_replaced=False,proposal_rows=871,proposal_adopted=False,
        replaced_names=sorted(OLD),new_names=[z['name'] for z in rows],
        accepted_hand_axes_lengths_changed=False,motor_installation_command_clock_changed=False,
        electrical_or_M3_stack_moved=False,ARM059_integrated_base_and_16_placements_unchanged=True,
        carrier=dict(removed_free_span_local_mm=[[38,-162,-17],[66.5,-159,-9]],
            cross_tie_local_mm=[[66.5,-162,-13],[69.5,-159,17]],azimuth_deg=45.,
            M3_axis_world_mm=(R@np.array([71,-162,-13,1.]))[:3],
            load_path='Original -13 M3 outer pad -> outer tangent cross tie -> original paired +13 spoke -> original hub/collar/beam;no rear dogleg or added inner web.',
            thickness_mm=3.,strength_preload_friction_unverified=True),
        mechanical_entry=dict(forearm_ports_world_mm=[[93.2,-93,-2.5],[93.2,-93,17.5]],
            forearm_tangents=[[0,1,0],[0,1,0]],
            wrist_ports_local_mm=[[100,-14,-3],[100,-14,2]],wrist_tangents=[[0,1,0],[0,1,0]],
            wrist_front_ports_mm=[[100,-4,-3],[100,-4,2]],OD_bore_mm=[5.,3.2],straight_bore_mm=10.,
            removed_architecture='Rigid nested R40/R60 Z-to-Y turns at the moving wrist;not smaller bending radii.',
            long_materials_or_lengths_carried=False,
            load_path='Housing sliding bore -> integral short sleeves/bridge/spine -> original saddle clamp faces/two M2 -> Palm_wrist_saddle moving with H.',
            assembly_or_physical_bending_qualified=False))
    return rows,design


def rotation_bounds(output,centre):
    """Each actual solid stays inside its axial slab and AABB radial cylinder."""
    centre=np.array(centre)
    rows,proof=[],[]
    for z in output:
        b=np.array(n.c.m.pack._bbox(z['shape']))
        radial=np.maximum(abs(b[0,[0,2]]-centre[[0,2]]),abs(b[1,[0,2]]-centre[[0,2]]))
        radius=float(np.linalg.norm(radial))
        pad=1e-5
        sh=cq.Solid.makeCylinder(radius+pad,float(b[1,1]-b[0,1]+2*pad),
            cq.Vector(centre[0],b[0,1]-pad,centre[2]),cq.Vector(0,1,0))
        rows.append(dict(name='arm060_rotation_bound_'+z['name'],shape=sh,body='forearm',
            evidence_only=True,color=[.7,.3,.15]))
        proof.append(dict(actual_part=z['name'],actual_bbox_mm=b,axis_centre_mm=centre,
            axis=[0,1,0],radial_bound_mm=radius,axial_Y_mm=b[:,1],numerical_padding_mm=pad,
            proof='For every material point, Y and distance to the Y axis are invariant. The complete actual-solid AABB bounds both radial coordinates,including curved interiors. This conservative bound covers all angles;it is not the exact swept solid.'))
    assert len(rows)==9
    return rows,proof


def nut_approach(nut,axis):
    b=np.array(n.c.m.pack._bbox(nut['shape']))
    centre=np.array(axis)
    radial=np.maximum(abs(b[0,[0,2]]-centre[[0,2]]),abs(b[1,[0,2]]-centre[[0,2]]))
    radius=float(np.linalg.norm(radial))+1e-5
    lo=float(b[0,1]-20.)
    hi=float(b[1,1])
    sh=cq.Solid.makeCylinder(radius,hi-lo,cq.Vector(centre[0],lo,centre[2]),cq.Vector(0,1,0))
    return dict(name='arm060_M3_nut_axial_approach_probe',shape=sh,body='forearm',evidence_only=True,color=[.15,.7,.4]),dict(
        actual_nut=nut['name'],actual_bbox_mm=b,axis_world_mm=centre,
        direction_from_seat=[0,-1,0],probe_travel_mm=20.,outer_radius_mm=radius,axial_Y_mm=[lo,hi],
        entire_actual_nut_and_its_translations_in_bound=True,
        probe_travel_is_test_assumption_not_required_tool_length=True,
        tool_supplier_size_drive_or_handle_selected=False,
        purpose='Reserve an open straight approach behind the actual M3nut. Full cylinder also reserves its centre;not a verified wrench/driver or assembly path.')
