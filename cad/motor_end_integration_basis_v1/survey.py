"""Bound the next motor-end redesign from real occupied electrical/support material."""
import importlib.util
import sys
from pathlib import Path

sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('_arm058_prior',ROOT/'cad/forward_terminal_support_v1/postprocess.py')
p=importlib.util.module_from_spec(spec)
spec.loader.exec_module(p)
a,n,np,cq=p.a,p.n,p.np,p.a.cq
measure=n.m.drive.module('_arm058_measure',ROOT/'cad/long_housing_route_v1/review_core.py')


def context():
    source={}
    def read(rel):
        source[rel]=n.m.drive.sha(ROOT/rel)
        return n.p.read(ROOT/rel)
    raw=read('cad/forward_terminal_support_v1/reviews/check.json')
    current,_,proposed,membership,delta,_,_,reloc,clock=p.metadata(raw)
    guide=next(z for z in read('cad/wire_guides_v1/generated/scene.json')['groups'] if z['module']=='thumb_ip_actuator')
    wires=read('cad/remote_motor_lead_v2/generated/scene.json')['parts']
    evidence=read('cad/forward_terminal_support_v1/reviews/evidence.json')
    inherited=read(evidence['actual_common'][0]['scene'])['parts'][0]
    for rel in ['cad/wire_guides_v1/model.py','cad/eight_bank_layout_v1/model.py',
                'cad/forward_terminal_support_v1/postprocess.py',
                'cad/forward_terminal_support_v1/model.py']:
        source[rel]=n.m.drive.sha(ROOT/rel)
    lookup={z['name']:dict(z) for z in proposed}
    selected=['arm057_straight_jaw_0','thumb_ip_actuator_lead2',
              'thumb_ip_two_tail_spool','arm054_eight_bank_carrier']
    shapes={}
    for z in n.c.load([lookup[k] for k in selected]+[inherited],source):
        M=np.array(delta.get(z['name'],np.eye(4)))
        if z['name'] in reloc: M=reloc[z['name']]@M
        if z['name'] in a.CLOCK_NAMES: M=clock@M
        shapes[z['name']]=n.g.original.transform(z['shape'],M)
    T=np.array(delta['thumb_ip_actuator_adapter'])
    G=T@np.array(guide['frame_world_mm'])
    paths=[]
    for row in wires:
        if row.get('module')=='thumb_ip_actuator' and row['name'].endswith(('lead1','lead2','lead3')):
            paths.append(dict(name=row['name'],end_world_mm=(T@np.r_[row['world_end_mm'],1.])[:3],
                OD_mm=row['local']['outer_diameter_mm'],centreline_radius_mm=row['local']['centreline_radius_mm'],
                centreline_length_mm=row['local']['centreline_length_mm']))
    return raw,lookup,shapes,G,paths,source,inherited['name']


def metrics(sh):
    value=measure.metrics(sh)
    value['bbox_mm']=n.c.m.pack._bbox(sh) if sh.Vertices() else None
    return value


def calculate():
    raw,lookup,sh,G,paths,source,common_name=context()
    queries=[]
    evidence=[]
    def query(name,kind,left,right):
        print(name,flush=True)
        got=left.intersect(right,tol=1e-6) if kind=='common' else left.cut(right,tol=1e-6)
        assert got.isValid(),name
        value=metrics(got)
        queries.append(dict(id=name,operation=kind,volume_mm3=value['global_values'][0],
            solids=value['solids'],bbox_mm=value['bbox_mm']))
        return got
    def add(name,shape,color):
        if shape.Solids(): evidence.append(dict(name=name,shape=shape,body='forearm',color=color,evidence_only=True))
    jaw=sh['arm057_straight_jaw_0']
    wire_common=query('actual_lower_jaw__fixed_lead2','common',jaw,sh['thumb_ip_actuator_lead2'])
    add('arm058_jaw_lead2_common',wire_common,[.98,.04,.03])
    # This is the real closed groove below the lid, not three invented wire bores.
    groove=n.g.original.transform(a.box([-3.,-3.75,-69.75],[3.,3.75,-67.35]),G)
    blocked=query('actual_lower_jaw__protected_electrical_groove','common',jaw,groove)
    add('arm058_groove_blocked_material',blocked,[.95,.12,.05])
    saved=sh[common_name]
    local=n.g.original.transform(saved,np.linalg.inv(G))
    lo,hi=np.array(n.c.m.pack._bbox(local))
    bands=[('foot_bridge_band',lo[2]-1.,-77.),('post_band',-77.,-71.75),('groove_block_band',-71.75,hi[2]+1.)]
    for label,z0,z1 in bands:
        region=n.g.original.transform(a.box([lo[0]-1.,lo[1]-1.,z0],[hi[0]+1.,hi[1]+1.,z1]),G)
        part=query('saved_adjuster_base_common__'+label,'common',saved,region)
        add('arm058_adjuster_'+label,part,[.95,.15,.05])
    carrier=sh['arm054_eight_bank_carrier']
    spool_common=query('actual_clocked_spool__carrier','common',sh['thumb_ip_two_tail_spool'],carrier)
    add('arm058_spool_carrier_common',spool_common,[.98,.04,.03])
    bridges=cq.Compound.makeCompound([a.box([34.,-162.,z-4.],[38.,-118.,z+4.]) for z in [-13.,13.]])
    query('spool_common__outside_two_actual_bridges','cut',spool_common,bridges)
    # Remove only the bridge free span, retaining both hub and M3-spoke planes.
    gap=cq.Compound.makeCompound([a.box([34.,-159.,z-4.],[38.,-121.,z+4.]) for z in [-13.,13.]])
    disconnected=query('counterfactual_carrier_without_bridge_free_spans','cut',carrier,gap)
    islands=[]
    for i,solid in enumerate(sorted(disconnected.Solids(),key=lambda s:-s.Volume())):
        name='arm058_counterfactual_carrier_solid_'+str(i)
        add(name,solid,[.3,.5,.75] if i==0 else [.95,.52,.08])
        islands.append(dict(name=name,volume_mm3=metrics(solid)['global_values'][0],bbox_mm=n.c.m.pack._bbox(solid)))
    stack=[dict(axis_world_mm=[71.,-126.,z],axis_direction_world=[0.,1.,0.],
        terminal_pad_Y_mm=[-127.,-126.],guide_foot_Y_mm=[-126.,-124.],
        adapter_Y_mm=[-124.,-121.],carrier_spoke_Y_mm=[-121.,-118.]) for z in [-13.,13.]]
    names={z['name'] for z in p.metadata(raw)[2]}
    protected=sorted(name for name in names if name.startswith('thumb_ip_actuator_') and
        (any(x in name for x in ['lead','plug','wire_guide_lid','lid_screw','lid_nut','rack_screw','rack_nut'])))
    all_native={z['id']:z for z in queries}
    result=dict(completed=True,source_sha256=source,native_queries=queries,native_boolean_operations=len(queries),
        operation_failures=[],report_volume_mm3=.001,Boolean_tolerance_mm=1e-6,
        inherited_ARM057_groups_reexecuted=0,inherited_saved_adjuster_common_reused=True,
        current862_installation_replaced=False,new_installed_geometry_rows=0,new_layout_adopted=False,
        diagnostic_evidence_rows=len(evidence),electrical_frame_proposal=G,electrical_paths_preserved=paths,
        actual_electrical_groove_local_mm=dict(lo=[-3.,-3.75,-69.75],hi=[3.,3.75,-67.35],
            geometry='one rectangular open-base groove closed by the retained lid;not three bores'),
        protected_hardware_names=protected,M3_nominal_stack=stack,counterfactual_islands=islands,
        partition_is_coordinate_bands_not_separate_manufactured_parts=True,
        partition_sum_mm3=sum(z['volume_mm3'] for z in queries if z['id'].startswith('saved_adjuster_base_common__')),
        inherited_common_volume_mm3=metrics(saved)['global_values'][0],
        source_blocking_records_read_not_reexecuted=[z for z in raw['native']['changed26__unchanged846_neutral']['overlaps']
            if 'lead' in z['part'] or 'lid_' in z['part']],
        required_changes=dict(combined_support=['replace guide-base load web and R5 frame as one explicitly supported unit',
            'retain the actual electrical groove/lid/M2 axes/three lead paths/plug and shared M3 stack',
            'redesign the mechanical jaw/adjuster/R5 placement together because fixed lead and groove are occupied',
            'do not repair by carving wire channels into jaws or trimming lid nuts'],
            carrier=['preserve both M3 spoke seats and hub;reroute their supporting bridges around actual output space',
            'bridge deletion is a disconnected counterfactual,not a usable carrier design',
            'new bridge material requires full actual neighboring hardware/arm-motion/assembly checks']),
        assembly_proposal=dict(order=['on bench: support guide base/lid and R5/adjuster kit while M3 stack is loose',
            'retain three electrical leads in their existing groove and keep lid/M2 fastener positions',
            'seat combined unit on retained M3 pad/adapter/carrier stack with manual support',
            'fit motor/output before threading long mechanical cords'],
            manual_support_required=True,tool_access_or_service_paths_tested=False,
            open_groove_allows_lid_off_placement_not_proof_of_plug_through_closed_groove=True,
            actual_cable_connector_mating_or_strain_relief_qualified=False),
        nominal_reaction='mechanical jaw/M2 -> captured adjuster -> integrated R5/load web -> shared M3/guide-base-foot -> adapter -> both carrier M3 spokes -> bridges -> hub/collar -> beam/roll output; electrical groove remains distinct from structural web',
        acceptance=dict(physical_strength_preload_friction=False,whole_material_motion=False,whole_arm_motion=False,
            assembly_tool_access=False,manufacture_or_exterior=False,all_surface_trim_Boolean_equivalence=False),
        evidence_not_installed_or_authorized_cuts=True)
    assert len(queries)==8
    assert all_native['actual_lower_jaw__fixed_lead2']['volume_mm3']>.001
    assert all_native['actual_lower_jaw__protected_electrical_groove']['volume_mm3']>.001
    assert abs(result['partition_sum_mm3']-result['inherited_common_volume_mm3'])<1e-5
    return result,evidence


def main():
    out=HERE/'reviews/survey.json'
    assert not out.exists()
    result,rows=calculate()
    result['scene']=n.a.a.prior.export(rows,HERE/'generated/evidence')
    result['source_sha256'][Path(__file__).relative_to(ROOT).as_posix()]=n.m.drive.sha(Path(__file__))
    result['input_hash_guard']=all(n.m.drive.sha(ROOT/f)==h for f,h in result['source_sha256'].items())
    n.a.save(out,result)
    print('completed',len(rows),[(z['id'],z['volume_mm3'],z['solids']) for z in result['native_queries']],flush=True)


if __name__=='__main__': main()
