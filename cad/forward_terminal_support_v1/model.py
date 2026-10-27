"""New handward R5 support, actual relocated adjusters and explicitly clocked spool."""
import importlib.util
import sys
from pathlib import Path

sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('_arm057_prior',ROOT/'cad/terminal_unit_orientation_v1/model.py')
p=importlib.util.module_from_spec(spec)
spec.loader.exec_module(p)
n,np,cq,d=p.n,p.np,p.cq,p.d
FRAME='arm057_handward_R5_support'
GUIDES=['arm057_straight_jaw_0','arm057_straight_jaw_1']
CLOCK_NAMES={'thumb_ip_two_tail_spool',*[f'thumb_ip_spool_tap6_{i}' for i in range(1,5)],'thumb_ip_tail_clamp_1','thumb_ip_tail_clamp_2'}


def context():
    current,base55,fixed55,packet55,T,S0,S,prior55,prior54,source=p.context()
    raw56=n.p.read(ROOT/'cad/terminal_unit_orientation_v1/reviews/check.json')
    stored=n.c.load(n.p.read(ROOT/raw56['scene'])['parts'],source)
    baseline={key:list(rows) for key,rows in fixed55.items()}
    baseline['forearm']+=stored+[dict(z,shape=n.g.original.transform(z['shape'],S)) for z in packet55 if z['name']!=p.FRAME]
    removed=p.members()-{p.FRAME}|{p.NEW_FRAME}|CLOCK_NAMES
    fixed={key:[z for z in rows if z['name'] not in removed] for key,rows in baseline.items()}
    assert sum(map(len,baseline.values()))==872 and sum(map(len,fixed.values()))==846
    for rel in ['cad/terminal_unit_orientation_v1/reviews/check.json','cad/terminal_unit_orientation_v1/model.py']:
        source[rel]=n.m.drive.sha(ROOT/rel)
    return current,baseline,fixed,packet55,T,S0,S,prior54,source


def line_of(name):
    if name.startswith('motor_housing_clamp_'):
        return int(name.split('_')[3])
    if name.startswith('thumb_ip_barrel_'):
        return int(name.rsplit('_',1)[1])
    raise ValueError(name)


def setup(T,baseline):
    centre=(T@np.r_[p.terminal.M[:3,3],1.])[:3]
    carrier=next(z['shape'] for z in baseline['forearm'] if z['name']=='arm054_eight_bank_carrier')
    adapter=next(z['shape'] for z in baseline['forearm'] if z['name']=='thumb_ip_actuator_adapter')
    xmax=max(n.c.m.pack._bbox(z)[1][0] for z in [carrier,adapter])
    # Actual inward nut extent is6.1mm in this orientation; no added clearance target.
    x=xmax+6.1
    rotation=np.column_stack([np.array([0.,0.,1.]),np.array([0.,-1.,0.]),np.array([1.,0.,0.])])
    frames=[]
    for i,z in enumerate([centre[2]-10.,centre[2]+10.]):
        Q=np.eye(4)
        Q[:3,:3]=rotation
        Q[:3,3]=[x,-117.,z]
        frames.append(Q)
    return centre,x,frames,dict(actual_carrier_adapter_outer_X_mm=xmax,inward_M2_nut_projection_mm=6.1,
        barrel_axis_X_mm=x,projection_is_not_global_clearance_proof=True)


def channels(T,baseline):
    centre,x,frames,basis=setup(T,baseline)
    ports=[]
    for i,(axial,b) in enumerate(zip([4.,12.],[-180.,0.])):
        B=(T@np.r_[p.terminal.route.circle(p.terminal.M[:3,3],p.terminal.M[:3,:3],axial,10.,np.deg2rad(b)),1.])[:3]
        A=np.array([x-5.,B[1],B[2]])
        M=A+np.array([5./np.sqrt(2),5.-5./np.sqrt(2),0.])
        Z=A+np.array([5.,5.,0.])
        E=np.array([x,-146.,B[2]])
        pieces=[p.terminal.route.arc(A,M,Z,5.*np.pi/2,'new_handward_R5'),p.terminal.route.line(Z,E,'new_R5_straight')]
        ports.append(dict(line=i,Q=frames[i],B=B,A=A,end=E,pieces=pieces,groove_axial_mm=axial,
            tangent_angle_deg=b,tangent_length_mm=float(A[0]-B[0]),shoulder=np.array([x,-122.,B[2]]),mouth=np.array([x,-117.,B[2]])))
    return ports,basis


def box(lo,hi):
    return cq.Solid.makeBox(*np.array(hi)-lo,cq.Vector(*lo))


def cy(r,y0,y1,x,z):
    return cq.Solid.makeCylinder(r,y1-y0,cq.Vector(x,y0,z),cq.Vector(0,1,0))


def build(packet,T,S,baseline,source):
    ports,basis=channels(T,baseline)
    x=basis['barrel_axis_X_mm']
    z0,z1=[z['B'][2] for z in ports]
    parts=[]
    for y in [-287.,-261.]:
        parts.append(n.g.original.transform(p.terminal.cyl(3.,-82.,-81.,[-11.,y]),T))
        q=(T@np.array([-11.,y,-81.,1.]))[:3]
        parts.append(box([q[0],-127.,q[2]-3.],[x+4.,-126.,q[2]+3.]))
        parts.append(box([x+1.,-150.,q[2]-1.5],[x+4.,-126.,q[2]+1.5]))
    seat_z=[(T@np.array([-11.,y,-81.,1.]))[2] for y in [-287.,-261.]]
    rail_lo=min(z0-4.,min(seat_z)-1.5)
    rail_hi=max(z1+4.,max(seat_z)+1.5)
    for ya,yb in [(-150.,-147.),(-139.,-136.)]:
        parts.append(box([x-2.,ya,rail_lo],[x+2.,yb,rail_hi]))
    for port in ports:
        parts.append(cy(5.5,-139.,-136.,x,port['B'][2]))
        parts += [p.terminal.sweep_piece(seg,1.6) for seg in port['pieces']]
    frame=parts[0]
    for sh in parts[1:]:
        frame=frame.fuse(sh)
    for port in ports:
        for seg in port['pieces']:
            frame=frame.cut(p.terminal.sweep_piece(seg,.4))
        frame=frame.cut(cy(2.55,-139.01,-135.99,x,port['B'][2]))
    for y in [-287.,-261.]:
        frame=frame.cut(n.g.original.transform(p.terminal.cyl(1.7,-82.01,-80.99,[-11.,y]),T))
    frame=frame.clean()
    assert frame.isValid() and len(frame.Solids())==1,(frame.isValid(),len(frame.Solids()))
    new=[dict(name=FRAME,shape=frame,body='forearm',role='new_R5_guides_and_original_M3_seat_support',color=[.93,.48,.1])]
    moved=[]
    contracts=[]
    rel='cad/sheath_retention_v1/selected/generated/parts/scene.json'
    source[rel]=n.m.drive.sha(ROOT/rel)
    old_jaws=n.c.load([z for z in n.p.read(ROOT/rel)['parts'] if z['name'] in {'motor_housing_clamp_0_jaw_-1','motor_housing_clamp_1_jaw_-1'}],source)
    for port in ports:
        i,Q=port['line'],port['Q']
        oldQ=T@p.a.b.frame(i)
        delta=Q@np.linalg.inv(oldQ)
        names=[]
        for row in packet:
            name=row['name']
            if name in [p.FRAME,'arm055_split_return_0','arm055_split_return_1']:
                continue
            if line_of(name)!=i:
                continue
            moved.append(dict(row,shape=n.g.original.transform(row['shape'],delta)))
            names.append(name)
        jaw=next(z for z in old_jaws if z['name']=='motor_housing_clamp_'+str(i)+'_jaw_-1')['shape']
        jaw=n.g.original.transform(jaw,Q@np.linalg.inv(p.a.b.frame(i)))
        es=[cq.Edge.makeLine(cq.Vector(0,-12,0),cq.Vector(0,-24,0))]
        tube=p.a.b.sweep(es,2.5).cut(p.a.b.sweep(es,1.6)).clean()
        sh=jaw.fuse(n.g.original.transform(tube,Q)).clean()
        lost=jaw.cut(sh,tol=1e-6)
        assert sh.isValid() and len(sh.Solids())==1 and (not lost.Solids() or lost.Volume()<1e-8)
        new.append(dict(name=GUIDES[i],shape=sh,body='forearm',physical_line=i,
            role='preserved_original_lower_jaw_with_straight_handward_sleeve',color=[.93,.48,.1]))
        contracts.append(dict(line=i,Q=Q,transform_from_ARM055=delta,moved_names=names,
            shoulder_world_mm=port['shoulder'],mouth_world_mm=port['mouth'],free_port_world_mm=(Q@np.array([0.,-24.,0.,1.]))[:3],
            free_tangent_world=[0.,1.,0.],original_jaw_removed_mm3=lost.Volume() if lost.Solids() else 0.,
            straight_sleeve_lead_mm=12.,shoulder_to_free_port_mm=29.,guide_OD_bore_mm=[5.,3.2],
            old_R40_return_is_replaced_by_real_straight_guide_not_hidden_or_omitted=True))
    assert len(moved)==16 and len(new)==3
    clock=[]
    original={z['name']:z for rows in baseline.values() for z in rows}
    for name in sorted(CLOCK_NAMES):
        clock.append(dict(original[name],shape=n.g.original.transform(original[name]['shape'],S)))
    # The four physical PCD12 axes permute under the explicit -90deg clocking.
    holes=np.array([[6.,0.],[0.,6.],[-6.,0.],[0.,-6.]])
    rotated=holes@np.array([[0.,-1.],[1.,0.]])
    assert sorted(map(tuple,holes))==sorted(map(tuple,rotated))
    design=dict(parameter_basis=basis,ports=contracts,new_frame_uses_old_terminal_stalk=False,
        frame_replaces=p.NEW_FRAME,new_geometry_rows=3,relocated_fixed_rows=16,clocked_output_rows=7,
        clocked_names=sorted(CLOCK_NAMES),spool_clocking_canonical_X_deg=-90.,clock_transform_proposal=S,
        motor_installation_changed=False,motor_command_angle_deg=0.,four_PCD12_axes_permuted=True,
        PCD12_original_axes=holes,PCD12_clocked_axes=rotated,stock_horn_or_central_retainer_moved=False,
        M3_seat_stack_thickness_mm=1.,new_support_web_mm=3.,
        nominal_reaction='Straight sleeve/original jaw -> opposite jaw/M2 -> captured adjuster/nuts -> new R5 support rails/outboard webs -> original1mm M3 seats/bolts -> unchanged thumb adapter -> ARM054 carrier -> beam/roll flange. All mechanical strength/preload/friction and assembly remain unverified.',
        assembly_proposal='Mount new R5 support/adjuster/jaw kit at unchanged M3 seats first; clock spool by -90deg on the original stock horn with the four real PCD12 holes and existing screws; install motor/output,then thread the two cords. This is an untested ordering proposal,not a completed service path.')
    return new,moved,clock,design


def cords(T,S0,baseline,command=0.):
    route=p.terminal.route
    ports,_=channels(T,baseline)
    anchors=n.p.read(ROOT/'cad/motor_spool_coupling_v1/selected/reviews/check.json')['design']['anchors']
    rows,details=[],[]
    Qcommand=d.around(p.terminal.M[:3,3],(0,0,1),-command)
    for port,anchor,side,ma in zip(ports,anchors,[-1.,1.],[-300.,120.]):
        i=port['line']
        aa,bb=np.deg2rad([ma+command,port['tangent_angle_deg']])
        wrap=side*(aa-bb)
        assert 0<wrap<2*np.pi
        A=(T@np.r_[route.circle(p.terminal.M[:3,3],p.terminal.M[:3,:3],port['groove_axial_mm'],10.,aa),1.])[:3]
        mid=(T@np.r_[route.circle(p.terminal.M[:3,3],p.terminal.M[:3,:3],port['groove_axial_mm'],10.,(aa+bb)/2),1.])[:3]
        tail0=route.point(np.array(anchor['tail_exit_mm']))
        tail=(T@Qcommand@S0@np.r_[tail0,1.])[:3]
        pieces=[route.line(tail,A,'clocked_anchor_tail'),route.arc(A,mid,port['B'],10.*wrap,'clocked_R10_wrap'),
                route.line(port['B'],port['A'],'real_outward_tangent'),*port['pieces'],route.line(port['end'],port['mouth'],'real_adjuster_bore')]
        for j,seg in enumerate(pieces):
            rows.append(dict(name=f'arm057_cord_{i}_{j}',shape=p.terminal.sweep_piece(seg,.3),physical_line=i,
                segment=j,length_mm=seg['length_mm'],body='forearm',role=seg['label'],evidence_reference_not_hardware=True,
                color=[.97,.25,.1] if i==0 else [.1,.65,.95]))
        details.append(dict(line=i,command_deg=command,anchor_installation_phase_deg=ma,
            actual_anchor_angle_deg=ma+command,tangent_angle_deg=port['tangent_angle_deg'],wrap_angle_deg=float(np.rad2deg(wrap)),
            tangent_exit_world_mm=port['B'],R5_entry_world_mm=port['A'],R5_exit_world_mm=port['pieces'][0]['z'],
            shoulder_world_mm=port['shoulder'],mouth_world_mm=port['mouth'],tangent_length_mm=port['tangent_length_mm'],R5_radius_mm=5.,
            local_cord_length_mm=sum(z['length_mm'] for z in pieces),
            endpoint_gap_max_mm=max(np.linalg.norm(v['z']-w['a']) for v,w in zip(pieces,pieces[1:])),
            inherited_anchor_tail_kink_deg=26.565051177078,
            independent_command_geometry_not_same_whole_cord_motion=True))
    assert len(rows)==12 and all(z['shape'].isValid() and len(z['shape'].Solids())==1 for z in rows)
    return rows,details
