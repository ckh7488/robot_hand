"""Unscaled 20-motor neutral packaging, on the approved hand exterior.

Importing this module does not build, export, launch processes, or alter inputs.
get_layout() returns the explicit mm/rigid-transform contract.
build_parts(layout=None) returns world-mm CadQuery shapes with name, body, kind,
color, source and mass assumptions. export(output_dir, ..., check=True) writes
the geometry screen and placements; the parent assembly can reuse build_parts.

The palm seats and rear rack are geometry, not approved attachment load paths.
All 20 motor-to-joint transmissions and output-spool couplers remain unfinished.
No vendor geometry is scaled. New modules do not reuse the old 32 mm root-axis
offset. Only the declared neutral pose is screened. No subprocess is used here.
"""
from pathlib import Path
import argparse
import hashlib
import json
import math

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
BASE=ROOT/'cad/modular_roots_v9'
FORM=ROOT/'cad/form_candidate_v1'
VENDOR_ASSETS=('M077_stator','M077_rotor','S101_relief','M2_body_TAP8',
               'M2_base_screws','M2_base_nuts')

def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def get_layout(path=None):
    data=json.loads(Path(path or HERE/'layout.json').read_text(encoding='utf-8'))
    if len(data['modules'])!=20 or len({m['joint'] for m in data['modules']})!=20:
        raise ValueError('Twenty independently assigned motor modules required')
    return data

def input_hashes():
    files=[Path(__file__).resolve(),HERE/'layout.json',HERE/'exterior.py',
           FORM/'design.json',FORM/'build.py',FORM/'lightweight_digits.py',
           FORM/'lightweight_thumb.py',FORM/'generated/scene.json',
           BASE/'mass_screen.json']
    files += [BASE/'parts'/(n+'.step') for n in VENDOR_ASSETS]
    scene=json.loads((FORM/'generated/scene.json').read_text())
    files += [BASE/'parts'/(p['asset']+'.step') for p in scene['arm_reference']]
    return {p.relative_to(ROOT).as_posix():digest(p) for p in files}

def _transform(shape,matrix):
    import cadquery as cq
    from OCP.gp import gp_Trsf
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    tr=gp_Trsf()
    tr.SetValues(*[float(matrix[i][j]) for i in range(3) for j in range(4)])
    return cq.Shape.cast(BRepBuilderAPI_Transform(shape.wrapped,tr,True).Shape())

def _box(cq,size,center):
    return cq.Workplane('XY').box(*size).translate(center).val()

def _cyl(cq,radius,length,start,direction=(0,0,1)):
    return cq.Solid.makeCylinder(radius,length,cq.Vector(*start),cq.Vector(*direction))

def _bbox(shape):
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    box=Bnd_Box();BRepBndLib.AddOptimal_s(shape.wrapped,box,False,False)
    b=box.Get();return [list(b[:3]),list(b[3:])]

def _properties(shape,mass):
    from OCP.GProp import GProp_GProps
    from OCP.BRepGProp import BRepGProp
    g=GProp_GProps();BRepGProp.VolumeProperties_s(shape.wrapped,g)
    v=g.Mass();c=g.CentreOfMass();I=g.MatrixOfInertia()
    return dict(volume_mm3=v,mass_kg=mass,com_world_mm=[c.X(),c.Y(),c.Z()],
        inertia_com_world_kg_m2=[[I.Value(i+1,j+1)*(mass/v)*1e-6 for j in range(3)] for i in range(3)],
        bbox_mm=_bbox(shape),valid=shape.isValid(),solid_count=len(shape.Solids()))

def build_parts(layout=None):
    """Build world-mm solids only; no export or input modification.

    A caller may use each part['shape'] directly in a new CadQuery assembly.
    Body 'palm' means one of six inner modules; 'forearm' includes four wrist-
    zone and ten distal modules on the rear rack, not a moving finger body.
    Vendor solids can be multi-solid assemblies; every new manufactured part
    is checked to be one valid solid. Mass is a source/volume assumption.
    """
    import cadquery as cq
    C=layout or get_layout();mc=C['module']
    assets={n:cq.importers.importStep(str(BASE/'parts'/(n+'.step'))).val() for n in VENDOR_ASSETS}
    rows=[]
    def add(name,shape,kind,body,module=None,source=None,transform=None,mass=None,color=None):
        if transform is not None:shape=_transform(shape,transform)
        if source is None and (not shape.isValid() or len(shape.Solids())!=1):
            raise ValueError('New component must be one valid solid: '+name)
        if mass is None:mass=shape.Volume()*(7.85e-6 if kind=='steel' else 1.25e-6)
        rows.append(dict(name=name,shape=shape,kind=kind,role='drive_'+kind,body=body,
            module=module,source=source,transform_world_mm=transform,
            color=color or {'motor':(0.17,0.20,0.24),'steel':(.52,.55,.58),
                'cradle':(.18,.46,.47),'support':(.22,.38,.56),'spool':(.85,.58,.21)}[kind],
            mass_kg=mass))
    plate=_box(cq,mc['adapter_size_mm'],mc['adapter_center_mm'])
    for x,y in mc['base_hole_xy_mm']:
        plate=plate.cut(_cyl(cq,mc['adapter_hole_radius_mm'],6,(x,y,-18)))
    # A ring here is a cable-drum envelope, deliberately without a claimed
    # horn/spline coupling: its nearest face is 0.5 mm beyond the rotor front.
    spool=_cyl(cq,8,6,(1,0,0),(1,0,0))
    for x in [.5,6.5]:spool=spool.fuse(_cyl(cq,9,1,(x,0,0),(1,0,0)))
    spool=spool.cut(_cyl(cq,2.2,10,(-1,0,0),(1,0,0))).clean()
    for m in C['modules']:
        prefix=m['name'];body=m['rigid_body'];M=m['transform_world_mm']
        for suffix,asset,kind,mass in [
            ('motor','M077_stator','motor',.018),
            ('rotor','M077_rotor','motor',0),
            ('cradle','S101_relief','cradle',None),
            ('body_screws','M2_body_TAP8','steel',None),
            ('base_screws','M2_base_screws','steel',None),
            ('base_nuts','M2_base_nuts','steel',None)]:
            add(prefix+'_'+suffix,assets[asset],kind,body,prefix,
                'cad/modular_roots_v9/parts/'+asset+'.step',M,mass)
        if m['zone']=='palm' and m.get('palm_mount_style')=='horizontal_four_posts':
            adapter=plate
            # The whole module lies in the palm plane; these four connected
            # pedestals reach the unchanged cavity floor without moving its
            # vendor mounting holes. They still have no chassis fasteners.
            for xx in [-27.4,-1.4]:
                for yy in [-24.5,9.5]:
                    adapter=adapter.fuse(_box(cq,(4,4,7.6),(xx,yy,-19.6)))
            adapter=adapter.clean()
        elif m['zone']=='palm':
            # Lower lip of the side-mounted plate continues into a real foot,
            # nominally on the existing floor, but without matching floor bolts.
            foot=_box(cq,(3.5,40,9.8),(-32.15,-7.5,-10.9))
            adapter=plate.fuse(foot).clean()
        else:
            # Actual flange into the rear rack. The motor/body screws retain
            # vendor locations; this flange gets two independent M3 envelopes.
            flange=_box(cq,(3,40,12),(-31.9,-7.5,-18.3))
            adapter=plate.fuse(flange)
            for yy in [-20.5,5.5]:
                adapter=adapter.cut(_cyl(cq,1.7,6,(-35,yy,-19.9),(1,0,0)))
            adapter=adapter.clean()
        add(prefix+'_adapter',adapter,'support',body,prefix,transform=M)
        add(prefix+'_spool',spool,'spool',body,prefix,transform=M)
    # Two connected rails/crossbars and four posts are actual new rack solids.
    # The posts meet the nominal beam lower plane; chassis clamps/bolts do not
    # exist yet, and that unfinished edge is not promoted to a complete chain.
    support=C['supports'];rail_z=support['forearm_rail_z_mm']
    y0,y1=support['forearm_rail_y_mm'];rack=None
    for x in support['forearm_rail_x_centers_mm']:
        s=_box(cq,(12,y1-y0,3),(x,(y0+y1)/2,sum(rail_z)/2))
        rack=s if rack is None else rack.fuse(s)
    x0=min(support['forearm_rail_x_centers_mm'])-6
    x1=max(support['forearm_rail_x_centers_mm'])+6
    for y in [-295,-253,-211,-169,-127,-85,-43,-3]:
        rack=rack.fuse(_box(cq,(x1-x0,4,3),((x0+x1)/2,y,sum(rail_z)/2)))
    for x,y in support['forearm_post_xy_mm']:
        rack=rack.fuse(_box(cq,(6,6,29),(x,y,-58.5)))
    for m in C['modules']:
        if m['zone']=='palm':continue
        cx,cy,cz=m['module_envelope_center_mm']
        for k,dy in enumerate([-13,13],1):
            x,y=cx-16,cy+dy
            rack=rack.cut(_cyl(cq,1.7,5,(x,y,-77)))
            screw=_cyl(cq,1.5,8,(x,y,-79)).fuse(_cyl(cq,2.85,2,(x,y,-81))).clean()
            nut=cq.Workplane('XY').polygon(6,6.35).extrude(2.4).translate((x,y,-73)).val()
            nut=nut.cut(_cyl(cq,1.5,4,(x,y,-74))).clean()
            add(m['name']+'_rack_screw_'+str(k),screw,'steel','forearm',m['name'])
            add(m['name']+'_rack_nut_'+str(k),nut,'steel','forearm',m['name'])
    add('Forearm_twenty_axis_module_rack',rack.clean(),'support','forearm')
    return rows

def _overlap_bbox(a,b,tol=1e-7):
    return all(min(a[1][i],b[1][i])-max(a[0][i],b[0][i])>tol for i in range(3))

def check_neutral(parts,layout=None,exterior_parts=None):
    """Exact OCC volume intersections after AABB culling, neutral only.

    Original hand/arm self-intersections are not rerun or silently accepted.
    Preserved vendor internals in one module are excluded by explicit policy;
    new parts have no intended-positive-overlap exemptions.
    """
    import cadquery as cq
    if exterior_parts is None:
        from exterior import rebuild_exterior
        exterior_parts,_=rebuild_exterior()
    refs=[dict(p,body='reference_hand',reference=True) for p in exterior_parts]
    arm=json.loads((FORM/'generated/scene.json').read_text())['arm_reference']
    for p in arm:
        s=cq.importers.importStep(str(BASE/'parts'/(p['asset']+'.step'))).val()
        M=[p['R'][i]+[p['pos_mm'][i]] for i in range(3)]+[[0,0,0,1]]
        refs.append(dict(name=p['name'],shape=_transform(s,M),body='reference_arm',reference=True))
    allparts=parts+refs
    boxes={p['name']:_bbox(p['shape']) for p in allparts}
    candidates=[];excluded=0;exact=0;failures=[];clashes=[]
    for i,p in enumerate(parts):
        for q in allparts[i+1:]:
            # Six source components preserve a previously used vendor module;
            # this excludes internal supplier contacts, not any new mount/part.
            if p.get('source') and q.get('source') and p.get('module')==q.get('module'):
                excluded+=1;continue
            if _overlap_bbox(boxes[p['name']],boxes[q['name']]):candidates.append((p,q))
    print('Exact neutral candidate pairs:',len(candidates),flush=True)
    for p,q in candidates:
        try:
            intersection=p['shape'].intersect(q['shape']);v=intersection.Volume();exact+=1
            if v>.01:clashes.append(dict(parts=[p['name'],q['name']],volume_mm3=v))
        except Exception as exc:failures.append(dict(parts=[p['name'],q['name']],error=str(exc)))
        if exact%50==0:print('Checked',exact,flush=True)
    new=[p for p in parts if p.get('source') is None]
    mb=[[min(boxes[p['name']][0][i] for p in parts) for i in range(3)],
        [max(boxes[p['name']][1][i] for p in parts) for i in range(3)]]
    rb=[[min(boxes[p['name']][0][i] for p in refs) for i in range(3)],
        [max(boxes[p['name']][1][i] for p in refs) for i in range(3)]]
    return dict(status='Neutral volume screen only; not assembly or fabrication approval',
        scope='New 20-module solids versus one another, approved hand exterior, and neutral V9 arm reference',
        part_count=len(parts),motor_count=sum(p['name'].endswith('_motor') for p in parts),
        exterior_count=len(exterior_parts),arm_reference_count=len(arm),
        exact_pairs=exact,excluded_preserved_vendor_internal_pairs=excluded,
        exclusion_policy='Both source vendor components inside the same motor module only; no new-part positive-overlap exemptions',
        unresolved_overlaps=sorted(clashes,key=lambda x:-x['volume_mm3']),boolean_failures=failures,
        new_single_solid_parts=len(new),all_new_parts_single_valid_solid=all(p['shape'].isValid() and len(p['shape'].Solids())==1 for p in new),
        packaging_bbox_mm=mb,reference_hand_arm_bbox_mm=rb,
        added_negative_z_protrusion_mm=max(0,rb[0][2]-mb[0][2]),
        new_parts_neutral_volume_clear=(not clashes and not failures),
        fabrication_approval=False,motor_to_joint_transmissions_complete=False,
        palm_to_chassis_fasteners_complete=False,rear_rack_to_arm_fasteners_complete=False,
        minimum_clearance_tolerance_flexure_access_and_motion_checked=False,
        limitations=(layout or get_layout())['limitations'])

def export(output_dir,layout=None,check=True,exterior_parts=None):
    """Write portable metadata plus new native parts; no duplicate vendor STEP.

    Return (parts, report). Parent may export one full assembly/render, avoiding
    a second heavy assembly build. Start/end hashes must remain identical.
    """
    import cadquery as cq
    start=input_hashes();C=layout or get_layout();out=Path(output_dir)
    out.mkdir(parents=True,exist_ok=True);(out/'parts').mkdir(exist_ok=True);(out/'meshes').mkdir(exist_ok=True)
    print('Building twenty unscaled motor modules',flush=True)
    parts=build_parts(C)
    report=check_neutral(parts,C,exterior_parts) if check else {'geometry_screen_run':False}
    rows=[];scene=[]
    for p in parts:
        row={k:v for k,v in p.items() if k!='shape'}
        row.update(_properties(p['shape'],p['mass_kg']))
        mesh=out/'meshes'/(p['name']+'.stl')
        cq.exporters.export(p['shape'],str(mesh),tolerance=.3,angularTolerance=.3)
        scene.append(dict(name=p['name'],mesh=mesh.relative_to(out).as_posix(),mesh_units='mm',
            color=list(p['color']),role=p['role'],body=p['body'],bbox_mm=row['bbox_mm'],
            pos_mm=[0,0,0],R=[[1,0,0],[0,1,0],[0,0,1]],sha256_stl=digest(mesh)))
        if p['source'] is None:
            native=out/'parts'/(p['name']+'.step')
            cq.exporters.export(p['shape'],str(native));row['saved_world_step']=native.relative_to(out).as_posix()
            row['saved_step_sha256']=digest(native)
        rows.append(row)
    finish=input_hashes()
    if finish!=start:raise RuntimeError('Input changed during packaging build; output is not complete')
    report.update(source_sha256=start,input_start_end_hash_guard_passed=True,
        motor_mass_kg=.018*20,total_packaging_mass_kg=sum(p['mass_kg'] for p in parts),
        mass_scope='Motor 18 g includes rotor; other parts fully solid polymer1.25/steel7.85 g/cm3 assumptions',
        module_counts=C['counts'])
    (out/'placements_mass_inertia.json').write_text(json.dumps({'units':'mm;kg;kg*m2','parts':rows},indent=2),encoding='utf-8')
    (out/'scene.json').write_text(json.dumps({'units':'mm','parts':scene,'source_sha256':start},indent=2),encoding='utf-8')
    (out/'geometry.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('source_sha256','limitations','unresolved_overlaps')},indent=2),flush=True)
    return parts,report

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',default=str(HERE/'generated/packaging'))
    args=parser.parse_args();export(args.output)
