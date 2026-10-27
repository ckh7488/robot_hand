"""Separate whole support candidate retaining electrical material and both port datums."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('_arm074_prior',ROOT/'cad/captured_adjuster_kit_v1/model.py')
p=importlib.util.module_from_spec(spec);spec.loader.exec_module(p)
n,np,d=p.n,p.np,p.d
a=n.m.drive.module('_arm074_original_support',ROOT/'cad/integrated_motor_end_v1/model.py')
OLD=a.FRAME;NAME='arm074_branched_electrical_R5_support'


def electrical(source):
    rel='cad/forward_terminal_support_v1/reviews/check.json';raw=n.p.read(ROOT/rel);source[rel]=n.m.drive.sha(ROOT/rel)
    _,_,metas,_,delta,_,_,reloc,clock=a.p.metadata(raw)
    z=next(dict(z) for z in metas if z['name']=='thumb_ip_actuator_wire_guide_base')
    M=np.array(delta.get(z['name'],np.eye(4)))
    if z['name'] in reloc:M=reloc[z['name']]@M
    return n.g.original.transform(n.c.load([z],source)[0]['shape'],M),z,M


def context():
    rows,membership,source=p.context()
    for f in [Path(__file__),HERE/'reviews/survey.json',ROOT/'cad/integrated_motor_end_v1/model.py',ROOT/'cad/integrated_motor_end_v1/reviews/check.json']:
        source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    return rows,membership,source


def build(source):
    survey=n.p.read(HERE/'reviews/survey.json');raw=n.p.read(ROOT/'cad/forward_terminal_support_v1/reviews/check.json')
    old59=n.p.read(ROOT/'cad/integrated_motor_end_v1/reviews/check.json')
    shift=np.array(old59['design']['mechanical_translation_from_ARM057']);ports=a.channels(raw,shift)
    x=float(ports[0]['Q'][0,3]);inner=survey['protected_electrical_max_X_mm'];top=survey['actual_upper_jaw_max_Z_mm'];outer=x+4.
    base,meta,M=electrical(source)
    pieces=[base]
    # Replace the entire upper load route, beyond the full captured mechanism.
    pieces += [a.a.box([71.,-127.,-16.],[outer,-126.,-10.]),
        a.a.box([x+1.,-150.,-14.5],[outer,-126.,-11.5]),
        a.a.box([71.,-127.,10.],[inner,-126.,16.]),
        a.a.box([inner-3.,-127.,13.],[inner,-126.,top+3.]),
        a.a.box([inner-3.,-127.,top],[outer,-126.,top+3.]),
        a.a.box([x+1.,-150.,top],[outer,-126.,top+3.])]
    for y0,y1 in [(-150.,-147.),(-139.,-136.)]:pieces.append(a.a.box([x-2.,y0,-14.5],[x+2.,y1,top+3.]))
    for port in ports:
        pieces.append(a.a.cy(5.5,-139.,-136.,x,port['B'][2]))
        pieces.extend(a.a.p.terminal.sweep_piece(seg,1.6) for seg in port['pieces'])
    for z in [-13.,13.]:pieces.append(a.a.cy(3.,-127.,-126.,71.,z))
    sh=pieces[0]
    for q in pieces[1:]:sh=sh.fuse(q)
    for port in ports:
        for seg in port['pieces']:sh=sh.cut(a.a.p.terminal.sweep_piece(seg,.4))
        sh=sh.cut(a.a.cy(2.55,-139.01,-135.99,x,port['B'][2]))
    for z in [-13.,13.]:sh=sh.cut(a.a.cy(1.7,-127.01,-125.99,71.,z))
    sh=sh.clean();assert sh.isValid() and len(sh.Solids())==1,(sh.isValid(),len(sh.Solids()))
    design=dict(replaced_name=OLD,replacement_name=NAME,rows_replaced=1,other885_geometry_and_poses_unchanged=True,
        electrical_source=meta['step'],electrical_source_transform=M,port_contracts=old59['design']['ports'],
        M3_axes_world_mm=[[71.,-126.,z] for z in [-13.,13.]],M3_seat_Y_mm=[-127.,-126.],
        M3_stack_guidefoot_adapter_carrier_Y_mm=[[-126.,-124.],[-124.,-121.],[-121.,-118.]],
        upper_branch_inboard_outer_X_mm=inner,upper_branch_Z_mm=[top,top+3.],upper_branch_section_mm=3.,
        upper_boundary_from_actual_complete_jaw_not_design_clearance=True,lower_M3_route_preserved=True,
        old_upper_web_replaced_as_whole_not_small_corner_cut=True,old_ports_adjusters_jaws_motor_commands_clock_unchanged=True,
        current862_replaced=False,proposal886_adopted=False,nominal_reaction='Both original captured adjusters/jamnuts -> original R5 crossrail -> rear crossrail and branched upper/lower load paths -> retained two M3 seats/electrical foot -> original adapter/carrier -> beam/roll root. Strength,preload,friction and pullout are unqualified.',
        assembly_successes49_and92_not_inherited_to_changed_support=True)
    return dict(name=NAME,shape=sh,body='forearm',color=[.93,.48,.1],role='whole_branched_integrated_electrical_and_two_R5_support_candidate'),design


def queries(new,source):
    base,_,_=electrical(source);sh=new['shape'];survey=n.p.read(ROOT/'cad/motor_end_integration_basis_v1/reviews/survey.json');G=np.array(survey['electrical_frame_proposal'])
    groove=n.g.original.transform(a.a.box([-3.,-3.75,-69.75],[3.,3.75,-67.35]),G)
    items=[('original_electrical_material_outside_candidate','cut',base,sh),('protected_electrical_groove','common',sh,groove)]
    for z in [-13.,13.]:
        pad=a.a.cy(3.,-127.,-126.,71.,z).cut(a.a.cy(1.7,-127.01,-125.99,71.,z))
        items.append(('whole_original_M3_pad_'+str(z)+'_outside_candidate','cut',pad,sh))
    result=[]
    for label,op,l,r in items:
        q=l.cut(r,tol=1e-6) if op=='cut' else l.intersect(r,tol=1e-6)
        assert q.isValid();result.append(dict(id=label,operation=op,volume_mm3=q.Volume() if q.Solids() else 0.,solids=len(q.Solids())))
    return result
