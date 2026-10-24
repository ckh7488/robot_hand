"""Exact-solid CadQuery candidate; V9 sources are read-only. Run with --deps PATH if needed.

The output is a 20-axis construction study, NOT validated actuation or fabrication CAD.
Tendon polylines are colored route envelopes; they are deliberately excluded from mass.
"""
from pathlib import Path
from types import SimpleNamespace
import argparse, json, math, sys, hashlib, traceback, zipfile
SCRIPT_PATH=Path(__file__).resolve()
EARLY_SCRIPT_HASH=hashlib.sha256(SCRIPT_PATH.read_bytes()).hexdigest()
EARLY_DESIGN_HASH=hashlib.sha256((SCRIPT_PATH.parent/'design.json').read_bytes()).hexdigest()

ap=argparse.ArgumentParser(); ap.add_argument('--deps'); ap.add_argument('--output',help='Output root, default this candidate directory'); args=ap.parse_args()
if args.deps: sys.path.insert(0,args.deps)
import numpy as np
import cadquery as cq
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.gp import gp_Trsf
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from cadquery.occ_impl.exporters.assembly import exportGLTF
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

D=Path(__file__).resolve().parent; BASE=D.parent/'modular_roots_v9'
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
# Capture input bytes before decoding or importing geometry. A changed input makes
# the run fail, rather than stamping outputs with a newer file's unrelated hash.
INPUT_HASHES={SCRIPT_PATH:EARLY_SCRIPT_HASH,D/'design.json':EARLY_DESIGN_HASH}
for p in [BASE/'design.json',BASE/'scene.json',BASE/'mass_screen.json']:
    INPUT_HASHES[p]=digest(p)
C=json.loads((D/'design.json').read_text(encoding='utf8'))
ROOTOUT=Path(args.output).resolve() if args.output else D
OUT=ROOTOUT/'generated'; OUT.mkdir(parents=True,exist_ok=True); (OUT/'parts').mkdir(exist_ok=True)
(ROOTOUT/'images').mkdir(exist_ok=True); (ROOTOUT/'reviews').mkdir(exist_ok=True)
SCENE=json.loads((BASE/'scene.json').read_text())['Open']
MASS={x['name']:x['mass_kg'] for x in json.loads((BASE/'mass_screen.json').read_text())['items']}
ASSETS={}; PARTS=[]; BODIES={}; JOINTS=[]; ROUTES=[]; MOUNTS=[]; COLORS={'printed':(0.68,0.75,0.82),'motor':(0.16,0.20,0.25),'steel':(0.55,0.57,0.6),'pulley':(0.84,0.42,0.11),'route':(0.50,0.23,0.78),'mount':(0.14,0.50,0.49)}

def r(axis,a):
    a=math.radians(a); c,s=math.cos(a),math.sin(a)
    return np.array([[[1,0,0],[0,c,-s],[0,s,c]],[[c,0,s],[0,1,0],[-s,0,c]],[[c,-s,0],[s,c,0],[0,0,1]]][axis],float)
def T(p=(0,0,0),R=None):
    z=np.eye(4); z[:3,3]=p
    if R is not None:z[:3,:3]=R
    return z
def rp(v):return r(2,v[2])@r(1,v[1])@r(0,v[0])
def transform(shape,m):
    # gp_Trsf orthogonalizes imported floating-point rotation matrices; a generic
    # gp_GTrsf incorrectly treats round-off from the preserved CAD as a shear.
    tr=gp_Trsf(); tr.SetValues(*[float(v) for v in m[:3,:4].ravel()])
    return cq.Shape.cast(BRepBuilderAPI_Transform(shape.wrapped,tr,True).Shape())
def asset(n):
    if n not in ASSETS:
        path=BASE/'parts'/(n+'.step');INPUT_HASHES[path]=digest(path)
        ASSETS[n]=cq.importers.importStep(str(path)).val()
    return ASSETS[n]
def massprops(shape,mass):
    g=GProp_GProps(); BRepGProp.VolumeProperties_s(shape.wrapped,g)
    v=g.Mass(); c=g.CentreOfMass(); A=g.MatrixOfInertia()
    inertia=np.array([[A.Value(i+1,j+1) for j in range(3)] for i in range(3)])*(mass/v if v else 0)*1e-6
    return [c.X(),c.Y(),c.Z()],inertia.tolist(),v
def add(name,shape,body='palm',kind='printed',mass=None,origin=None,source=None,route=False):
    if origin is None:origin=np.eye(4)
    if source is None and not route and len(shape.Solids())!=1:
        raise ValueError('New manufactured component must be one connected solid: '+name)
    local=shape; world=transform(shape,origin)
    if mass is None:mass=0 if route else shape.Volume()*C['materials']['steel_density_kg_per_mm3' if kind=='steel' else 'printed_density_kg_per_mm3']
    com,I,vol=massprops(world,mass)
    p={'name':name,'shape':world,'body':body,'kind':kind,'mass_kg':mass,'com_world_mm':com,'inertia_com_world_kg_m2':I,'volume_mm3':vol,'source':source,'route':route,'transform_world':origin.tolist()}
    PARTS.append(p); return p
def body(name,parent,frame,axis,limits):
    BODIES[name]={'name':name,'parent_joint':parent,'frame_world':frame.tolist()}
    JOINTS.append({'name':name,'parent_joint':parent,'axis_local':axis,'frame_world':frame.tolist(),'range_deg':limits})
def cyl(radius,length,origin,axis=(1,0,0)):
    return cq.Solid.makeCylinder(radius,length,cq.Vector(*origin),cq.Vector(*axis))
def box(w,l,h,center):return cq.Workplane('XY').box(w,l,h).val().translate(cq.Vector(*center))
def exact_bounds(shape):
    bounds=Bnd_Box()
    # Explicit False avoids mesh-dependent overestimates after GLB tessellation.
    BRepBndLib.AddOptimal_s(shape.wrapped,bounds,False,False)
    return SimpleNamespace(**dict(zip(['xmin','ymin','zmin','xmax','ymax','zmax'],bounds.Get())))
def link_shape(L,w=22,d=18,proximal=False,tip=False,thumb=False,yaw=0):
    wall=C['links']['wall_mm']; bore=C['links']['joint_bore_radius_mm'];cx=C['links']['center_x_mm']
    y0=24 if proximal else 6; y1=L if tip else L-(28 if yaw else 8)
    outer=box(w,y1-y0,d,(cx,(y0+y1)/2,0))
    inner=box(w-2*wall,max(2,y1-y0-8),d-2*wall,(cx,(y0+y1)/2,0))
    solid=outer.cut(inner)
    if proximal:
        # Reuse the actual H101 bolt/nut face rather than colliding a new box
        # with the original motor at the base. The distal cradle is removed.
        source=asset('Thumb_metacarpal' if thumb else 'Flex_link_'+str(L))
        stub=source.intersect(box(150,5,100,(-14.5,19.5,0)))
        solid=solid.fuse(stub).fuse(box(w-2 if thumb else w,7,9,(cx,25,-10.5)))
    else:
        # Central eye nests between the preceding link's two outer fork ears.
        eye=cyl(7,12,(cx-6,0,0)).cut(cyl(bore,14,(cx-7,0,0)))
        solid=solid.fuse(eye).fuse(box(12,5,10,(cx,7.5,0)))
    if not tip:
        fork_frame=T((0,L,0),r(2,yaw))
        if yaw:
            # The entire fork, including its physical bores, is expressed in the
            # unchanged child-joint frame. A bottom adapter joins it to the tube.
            cross=box(w,4,d,(cx,-10,0))
            corners=[(cx-w/2,y1-2),(cx+w/2,y1-2)]
            R=r(2,yaw)
            distal=[(R@np.array([x,-10,0])+np.array([0,L,0]))[:2] for x in [cx+w/2,cx-w/2]]
            poly=corners+[tuple(p) for p in distal]
            floor=cq.Workplane('XY').polyline(poly).close().extrude(2).val().translate(cq.Vector(0,0,-d/2))
            solid=solid.fuse(floor).fuse(transform(cross,fork_frame))
        for side in [-1,1]:
            ear=box(4,16,d,(cx+side*9,-2,0)).cut(cyl(bore,6,(cx+side*9-3,0,0)))
            solid=solid.fuse(transform(ear,fork_frame))
    if proximal:
        # Preserve clearance around the actual H101 screws, including the small
        # added bridge. Only the new link is relieved; vendor fasteners stay exact.
        for it in SCENE:
            if it['name']==('thumb' if thumb else 'index')+'_mcp_output_screws':
                ref=next(f for f in C['hand']['fingers'] if f['name']==('thumb' if thumb else 'index'))
                relative=np.linalg.inv(ref['_frames'][1])@T(it['pos_mm'],np.array(it['R']))
                solid=solid.cut(transform(asset(it['asset']),relative))
    return solid.clean()
def ring(radius,width,center):
    p=np.array(center);return cyl(radius,width,p-np.array([width/2,0,0])).cut(cyl(C['links']['joint_bore_radius_mm'],width+2,p-np.array([width/2+1,0,0])))
def line_cylinder(a,b,rad):
    a=np.array(a);b=np.array(b);v=b-a;L=float(np.linalg.norm(v))
    return cq.Solid.makeCylinder(rad,L,cq.Vector(*a),cq.Vector(*(v/L))) if L>1e-8 else None

def make_frames():
    for f in C['hand']['fingers']:
        n=f['name']; F=T(f['root'],rp(f['base_rpy'])); H=F@T((0,0,C['hand']['root_axis_offset_z']))
        names=[n+'_spread',n+'_mcp',n+'_pip',n+'_dip'] if n!='thumb' else ['thumb_base_sweep','thumb_opposition','thumb_mcp','thumb_ip']
        Fs=[F,H,H@T((0,f['links'][0],0)),None]
        if n=='thumb':Fs[2]=Fs[2]@T(R=r(2,f['mcp_mount_yaw']))
        Fs[3]=Fs[2]@T((0,f['links'][1],0))
        for i,(nm,fr) in enumerate(zip(names,Fs)):
            key=['spread','mcp','pip','dip'][i] if n!='thumb' else nm
            body(nm,'palm' if i==0 else names[i-1],fr,[0,0,-1] if i==0 else [1,0,0],C['joint_ranges_deg'][key])
        f['_names']=names; f['_frames']=Fs

def imported_body(it):
    n=it['name']
    for f in C['hand']['fingers']:
        if not n.startswith(f['name']+'_'):continue
        names=f['_names']
        if '_root_' in n:
            return 'palm' if n.endswith(('root_motor','root_frame')) or '_root_base_' in n else names[0]
        if '_mcp_' in n:
            return names[0] if n.endswith(('_motor','_fixed_frame')) or '_mcp_base_' in n else names[1]
        return names[0]
    return 'palm'

def import_preserved():
    static_names={'Palm_back_chassis','Palm_front_cover','Palm_cover_screws','Palm_cover_nuts','Thumb_internal_cradle','Palm_thumb_mount_screws','Palm_thumb_mount_nuts','Palm_saddle_mount_screws','Palm_saddle_mount_nuts','Palm_wrist_saddle'}
    for it in SCENE:
        n=it['name'];keep=n in static_names or ('root_' in n and n[0].isupper())
        for f in C['hand']['fingers']:
            if n.startswith(f['name']+'_') and ('_root_' in n or n.endswith('_spread_carrier') or ('_mcp_' in n and not n.endswith('_link'))):keep=True
        if not keep:continue
        kind='motor' if it['role']=='vendor_motor' else 'steel' if it['role']=='fastener' else 'printed'
        add(n,asset(it['asset']),imported_body(it),kind,MASS.get(n,0),T(it['pos_mm'],np.array(it['R'])),str((BASE/'parts'/(it['asset']+'.step')).relative_to(D.parent.parent)))

def remote_motor(nm,bodyname,frame):
    add(nm+'_motor',asset('M077_stator'),bodyname,'motor',0.018,frame,'V9 vendor XL330-M077 STEP')
    add(nm+'_rotor_visual',asset('M077_rotor'),bodyname,'motor',0,frame,'visual rotor; mass already included in motor')
    sp=C['remote_motors']; sc=[sp['spool_center_x_mm'],0,0]
    spool=ring(sp['spool_radius_mm'],sp['spool_width_mm'],sc)
    for x in [sc[0]-sp['spool_width_mm']/2,sc[0]+sp['spool_width_mm']/2]:spool=spool.fuse(ring(sp['spool_radius_mm']+1,1,[x,0,0]))
    add(nm+'_dual_spool',spool,bodyname,'pulley',origin=frame)
    # Palm motors use the actual S101 body-side interface, then a properly
    # registered adapter. The prior horizontal plate did not attach to a motor.
    if bodyname=='palm':
        mc=C['palm_motor_mounts']
        for suffix,key,kind,mass_name in [('cradle','cradle_asset','mount','index_mcp_fixed_frame'),('body_screws','body_screw_asset','steel','index_mcp_base_body_tap'),('base_screws','base_screw_asset','steel','index_mcp_base_screws'),('base_nuts','base_nut_asset','steel','index_mcp_base_nuts')]:
            asset_name=mc[key]
            add(nm+'_'+suffix,asset(asset_name),bodyname,kind,MASS[mass_name],frame,'cad/modular_roots_v9/parts/'+asset_name+'.step')
        plate=box(*mc['adapter_dimensions_mm'],mc['adapter_center_local_mm'])
        for x,y in mc['base_hole_xy_local_mm']:
            plate=plate.cut(cyl(mc['adapter_hole_radius_mm'],mc['adapter_dimensions_mm'][2]+2,(x,y,mc['adapter_bottom_z_mm']-1),(0,0,1)))
        MOUNTS.append({'name':nm,'frame_world':frame.tolist()})
    else:
        # The moving thumb mounts are outside this palm-cradle checkpoint.
        plate=box(32,40,3,(-14.4,-7.5,-13))
        for x in [-27.4,-1.4]:
            for y in [-22.5,7.5]:plate=plate.cut(cyl(1.65,5,(x,y,-16),(0,0,1)))
    relief_fragment_volume=0
    if nm=='middle_remote_pip':
        # Local relief around the preserved thumb root housing; no source edit.
        obstacle=next(p['shape'] for p in PARTS if p['name']=='thumb_root_motor')
        plate=plate.cut(transform(obstacle,np.linalg.inv(frame)))
        solids=plate.Solids();largest=max(solids,key=lambda s:s.Volume())
        relief_fragment_volume=sum(s.Volume() for s in solids)-largest.Volume()
        plate=largest
    part=add(nm+'_mount_plate',plate,bodyname,'mount',origin=frame)
    if relief_fragment_volume>0:part['discarded_relief_fragment_volume_mm3']=relief_fragment_volume
    return (frame@np.r_[sc,1])[:3]

def new_fingers():
    for f in C['hand']['fingers']:
        n=f['name'];Fs=f['_frames']; names=f['_names'];w=C['links']['width_mm'];d=C['links']['depth_mm'];off=C['links']['pulley_side_offset_mm']
        for j,L in enumerate(f['links']):
            nm=names[j+1];fr=Fs[j+1]
            add(n+'_light_link_'+str(j+1),link_shape(L,w,d,proximal=j==0,tip=j==2,thumb=n=='thumb',yaw=f.get('mcp_mount_yaw',0) if j==0 else 0),nm,'printed',origin=fr)
            if j>0:
                add(n+'_shaft_'+str(j),cyl(C['links']['pin_radius_mm'],w+12,(C['links']['center_x_mm']-w/2-6,0,0)),nm,'steel',origin=fr)
                add(n+'_pulley_'+str(j),ring(8,3,[C['links']['center_x_mm']+(off if j==1 else -off),0,0]),nm,'pulley',origin=fr)
        if n!='thumb':
            starts=[]
            for j,label in enumerate(['pip','dip']):
                pos=[f['root'][0]+C['remote_motors']['palm_motor_origin_x_offset_mm'],C['remote_motors']['palm_rows_y_mm'][j],C['remote_motors']['palm_motor_z_mm']]
                pos=C['remote_motors'].get('palm_placement_overrides_mm',{}).get(n+'_'+label,pos)
                rot=C['remote_motors'].get('palm_rotation_overrides_deg',{}).get(n+'_'+label,[0,0,0])
                M=T(pos,rp(rot))
                starts.append(remote_motor(n+'_remote_'+label,'palm',M))
            coeffs=[{n+'_mcp':1,n+'_pip':1},{n+'_mcp':1,n+'_pip':1,n+'_dip':1}]
        else:
            starts=[]
            for j,label in enumerate(['mcp','ip']):
                M=Fs[1]@T(C['remote_motors']['thumb_local_origins_mm'][j],rp(C['remote_motors']['thumb_local_rotations_deg'][j]))
                starts.append(remote_motor('thumb_remote_'+label,names[1],M))
            coeffs=[{'thumb_mcp':1},{'thumb_mcp':1,'thumb_ip':1}]
            # Visible crossbar supporting both moving motor plates.
            cross=box(102,12,3,(-14.5,43,-16)).fuse(box(12,8,5.5,(-14.5,34,-11.75)))
            cross=cross.fuse(box(12,12,3,(-14.5,35,-16)))
            add('thumb_moving_motor_crossbar',cross,names[1],'mount',origin=Fs[1])
        for j,start in enumerate(starts):
            joint=names[j+2]; side=C['links']['center_x_mm']+(off if j==0 else -off)
            target=(Fs[j+2]@np.array([side,0,0,1]))[:3]
            upstream=[]
            if n!='thumb':upstream.append((Fs[0]@np.array([side,0,0,1]))[:3])
            upstream.append((Fs[1]@np.array([side,0,0,1]))[:3])
            if j==1:upstream.append((Fs[2]@np.array([side,0,0,1]))[:3])
            # Tangent offsets are placeholders; physical wrap geometry is a listed blocker.
            pts=[start]+upstream+[target]
            if n=='thumb':pts=[start]+(([ (Fs[2]@np.array([side,0,0,1]))[:3] ]) if j==1 else [])+[target]
            for sign in [-1,1]:
                shifted=[(np.array(p)+np.array([0,0,sign*8])).tolist() for p in pts]
                solids=[line_cylinder(a,b,C['remote_motors']['cable_diameter_mm']/2) for a,b in zip(shifted[:-1],shifted[1:])]
                solids=[x for x in solids if x is not None]
                add(joint+('_flexor_route' if sign<0 else '_extensor_route'),cq.Compound.makeCompound(solids),'palm','route',0,route=True)
                ROUTES.append({'joint':joint,'side':'flexor' if sign<0 else 'extensor','points_world_mm':shifted,'length_mm':sum(float(np.linalg.norm(np.array(b)-a)) for a,b in zip(shifted[:-1],shifted[1:])),'status':C['transmission']['routes_status']})
            BODIES[joint]['transmission']={'spool_radius_mm':8,'joint_radius_mm':8,'motor_angle_coefficients':coeffs[j],'spread_coefficient_assumption':0,'status':C['transmission']['mapping_status']}

def aggregate():
    for b in ['palm']+list(BODIES):
        ps=[p for p in PARTS if p['body']==b and p['mass_kg']>0]; total=sum(p['mass_kg'] for p in ps)
        com=sum((np.array(p['com_world_mm'])*p['mass_kg'] for p in ps),np.zeros(3))/total if total else np.zeros(3)
        I=np.zeros((3,3))
        for p in ps:
            q=(np.array(p['com_world_mm'])-com)/1000;I+=np.array(p['inertia_com_world_kg_m2'])+p['mass_kg']*(q@q*np.eye(3)-np.outer(q,q))
        frame=np.eye(4) if b=='palm' else np.array(BODIES[b]['frame_world']);rot=frame[:3,:3]; local=rot.T@(com-frame[:3,3])
        BODIES.setdefault(b,{'name':b,'parent_joint':None,'frame_world':frame.tolist()})
        BODIES[b].update({'mass_kg':total,'com_local_mm':local.tolist(),'inertia_com_local_kg_m2':(rot.T@I@rot).tolist(),'component_names':[p['name'] for p in ps]})
    for j in JOINTS:
        frame=np.array(j['frame_world']);parent=j['parent_joint'];pf=np.eye(4) if parent=='palm' else np.array(BODIES[parent]['frame_world']);j['frame_parent']= (np.linalg.inv(pf)@frame).tolist()
    contract={'revision':C['revision'],'scope':C['scope'],'geometry_status':C['status'],'units':{'length':'mm','angle':'degree','mass':'kg','inertia':'kg*m^2'},'joints':JOINTS,'bodies':list(BODIES.values()),'transmission':C['transmission'],'actuator_count':20,'robot_delay_requirement_ms':{'target':50,'upper_limit':100},'measurement_scope':'motor command to physical joint; no camera/perception/glove path','mass_limitations':['Printed density is an assumption, not a weighed prototype','Cables, connectors, harness and tension adjusters not included','Visual vendor rotor mass included in 18g motor envelope','Tendon geometry and constant Jacobian not validated']}
    (ROOTOUT/'simulation_contract.json').write_text(json.dumps(contract,indent=2),encoding='utf8')
    rows=[{k:v for k,v in p.items() if k!='shape'} for p in PARTS]
    (OUT/'placements_mass_inertia.json').write_text(json.dumps({'parts':rows,'hand_mass_kg':sum(p['mass_kg'] for p in PARTS),'bodies':list(BODIES.values())},indent=2),encoding='utf8')

def geometry_check():
    checks=[{'name':p['name'],'valid':p['shape'].isValid(),'solid_count':len(p['shape'].Solids()),'volume_mm3':p['volume_mm3']} for p in PARTS]
    # Exact overlap screening between new component envelopes and retained hardware.
    candidates=[p for p in PARTS if not p['route']]; clashes=[];intended=[];tested=0
    # OCC's tight bounds on vendor BReps are expensive; calculate once per part.
    bounds={p['name']:exact_bounds(p['shape']) for p in candidates}
    for i,a in enumerate(candidates):
        for b in candidates[i+1:]:
            if a['source'] and b['source'] and not ('remote' in a['name'] or 'remote' in b['name']):continue
            # Only an explicit same-motor visual rotor coupling is exempt.
            # New same-body mounts/motors are included in packaging checks.
            root_a=a['name'].split('_rotor_visual')[0];root_b=b['name'].split('_rotor_visual')[0]
            if a['name'].endswith('_rotor_visual') and b['name'] in [root_a+'_motor',root_a+'_dual_spool']:continue
            if b['name'].endswith('_rotor_visual') and a['name'] in [root_b+'_motor',root_b+'_dual_spool']:continue
            aa=bounds[a['name']]; bb=bounds[b['name']]
            if min(aa.xmax,bb.xmax)<=max(aa.xmin,bb.xmin) or min(aa.ymax,bb.ymax)<=max(aa.ymin,bb.ymin) or min(aa.zmax,bb.zmax)<=max(aa.zmin,bb.zmin):continue
            tested+=1
            if tested%50==0:print('Exact candidate overlap checks:',tested,flush=True)
            try:
                vol=a['shape'].intersect(b['shape']).Volume()
                if vol>0.01:
                    expected=next((m for m in MOUNTS if {a['name'],b['name']}=={m['name']+'_body_screws',m['name']+'_motor'}),None)
                    # Only the named self-tapping screw shafts in their own
                    # vendor body are exempt. Excess intrusion still fails.
                    if expected and abs(vol-8.906415172927)<0.05:
                        intended.append({'a':a['name'],'b':b['name'],'overlap_mm3':vol,'type':'self_tapping_thread_envelope','rationale':'Four retained M2 TAP8 shaft envelopes intersect their own factory pilot holes; nominal shaft radius 0.95 mm, factory pilot surface radius 0.8 mm. Helical threads are not modeled.','expected_overlap_mm3':8.906415172927,'tolerance_mm3':0.05})
                    else:clashes.append({'a':a['name'],'b':b['name'],'overlap_mm3':vol,'note':'Unintended/unresolved solid intersection; not an approved contact'})
            except Exception as e:clashes.append({'a':a['name'],'b':b['name'],'error':str(e)})
    report={'status':'CONSTRUCTION CANDIDATE - NOT FABRICATION PASS','valid_brep_count':sum(x['valid'] for x in checks),'total_shapes':len(checks),'component_checks':checks,'open_pose_exact_new_cross_body_overlap_pairs':clashes,'intended_thread_envelope_intersections':intended,'overlap_scope':'All new-component pairs, including same rigid body and remote cradle hardware; legacy-only pairs excluded. Only own motor/visual rotor and listed own pilot-hole/thread-envelope pairs exempt. Field name retained for review compatibility.','exact_candidate_pairs_tested':tested,'limitations':['Only neutral pose; no continuous swept collision proof','Eight palm motor-to-cradle and cradle-to-adapter fasteners modeled; adapter-to-chassis supports absent','Cable route solids are envelopes only, excluded from clash and mass checks','Thumb moving motor fasteners, tendon attachments and tensioners not detailed','Thread geometry, torque, fit, stiffness and practical tool access remain unvalidated'],'actuator_count':sum(p['name'].endswith('_motor') for p in PARTS),'independent_joint_count':len(JOINTS)}
    report['physical_assembly_bounds_mm']={'min':[min(getattr(b,k) for b in bounds.values()) for k in ['xmin','ymin','zmin']],'max':[max(getattr(b,k) for b in bounds.values()) for k in ['xmax','ymax','zmax']],'scope':'Exact OCC bounding boxes of all physical components; cable-route envelopes excluded'}
    manufactured=[p for p in PARTS if p['source'] is None and not p['route']]
    report['manufactured_connected_solid_check']={'required_count_per_component':1,'component_count':len(manufactured),'passed':all(len(p['shape'].Solids())==1 for p in manufactured),'enforcement':'Generator rejects a disconnected new manufactured component before mass/export; no exceptions in this candidate'}
    report['distal_hinge_axis_checks']=hinge_axis_check()
    report['palm_motor_mounting']=mounting_check()
    (ROOTOUT/'reviews'/'geometry.json').write_text(json.dumps(report,indent=2),encoding='utf8');return report

def hinge_axis_check():
    """Measure OCC cylindrical surfaces, separately from declared joint frames.

    This is a generator self-check; the independent review reopens saved STEP.
    Coaxiality does not prove bearing fit, shaft retention or swept clearance.
    """
    lookup={p['name']:p for p in PARTS};rows=[]
    for f in C['hand']['fingers']:
        for j in [1,2]:
            frame=f['_frames'][j+1];axis=frame[:3,0];anchor=frame[:3,3]
            item={'joint':f['_names'][j+1],'surface_checks':[]}
            for label,name,radius in [('parent_bore',f['name']+'_light_link_'+str(j),C['links']['joint_bore_radius_mm']),('child_bore',f['name']+'_light_link_'+str(j+1),C['links']['joint_bore_radius_mm']),('pin',f['name']+'_shaft_'+str(j),C['links']['pin_radius_mm'])]:
                choices=[]
                for face in lookup[name]['shape'].Faces():
                    surface=BRepAdaptor_Surface(face.wrapped)
                    if surface.GetType()!=GeomAbs_Cylinder:continue
                    cylinder=surface.Cylinder()
                    if abs(cylinder.Radius()-radius)>1e-5:continue
                    direction=cylinder.Axis().Direction();location=cylinder.Axis().Location()
                    a=np.array([direction.X(),direction.Y(),direction.Z()]);p=np.array([location.X(),location.Y(),location.Z()])
                    angle=math.degrees(math.acos(float(np.clip(abs(a@axis),-1,1))))
                    distance=float(np.linalg.norm(np.cross(anchor-p,a)))
                    choices.append({'axis_error_deg':angle,'line_distance_mm':distance,'axis_world':a.tolist(),'point_world_mm':p.tolist()})
                best=min(choices,key=lambda c:c['line_distance_mm']+c['axis_error_deg']) if choices else None
                item['surface_checks'].append({'role':label,'component':name,'nominal_radius_mm':radius,'best_surface':best,'passed':bool(best and best['axis_error_deg']<1e-4 and best['line_distance_mm']<1e-4)})
            item['passed']=all(c['passed'] for c in item['surface_checks']);rows.append(item)
    return {'scope':'In-memory OCC cylinder self-check; saved STEP independently remeasured by review tool','passed':all(x['passed'] for x in rows),'joints':rows}

def mounting_check():
    """Check actual cylinder axes and explicit, incomplete attachment graph.

    Minimum distance is recorded separately from face contact/load capacity.
    No adapter-to-chassis edge is invented from proximity or a shared body ID.
    """
    mc=C['palm_motor_mounts'];lookup={p['name']:p for p in PARTS};rows=[]
    def driver_screen(n,F):
        paths=[]
        def screen(label,local_start,local_axis,targets):
            bit=transform(cyl(1.5,30,local_start,local_axis),F);ab=exact_bounds(bit);hits=[]
            for target in targets:
                shape=lookup[n+target]['shape'];bb=exact_bounds(shape)
                if min(ab.xmax,bb.xmax)<=max(ab.xmin,bb.xmin) or min(ab.ymax,bb.ymax)<=max(ab.ymin,bb.ymin) or min(ab.zmax,bb.zmax)<=max(ab.zmin,bb.zmin):continue
                volume=bit.intersect(shape).Volume()
                if volume>.01:hits.append({'component':n+target,'overlap_mm3':volume})
            return {'label':label,'origin_world_mm':(F@np.r_[local_start,1])[:3].tolist(),'axis_world':(F[:3,:3]@local_axis).tolist(),'driver_radius_mm':1.5,'straight_length_mm':30,'checked_components':[n+t for t in targets],'blockers':hits,'clear_in_declared_subset':not hits}
        for x,y in mc['base_hole_xy_local_mm']:
            p=[x,y,-11.2+1e-4]
            paths.append(screen('base screw before motor installation',p,[0,0,1],['_cradle','_mount_plate','_base_nuts']))
            paths.append(screen('same base screw with motor present',p,[0,0,1],['_motor']))
        for side,x in zip([-1,1],mc['body_head_seat_x_local_mm']):
            for y,z in mc['body_axis_yz_local_mm']:
                paths.append(screen('body screw after base assembly, before spool/chassis installation',[x+side*(1.2+1e-4),y,z],[side,0,0],['_motor','_cradle','_mount_plate','_base_screws','_base_nuts']))
        return {'scope':'30 mm long, diameter 3 mm straight driver envelope inside this module only; handle, screw insertion/removal sweep and neighboring assembly are NOT validated','required_order':'Fasten cradle to adapter before installing motor; fit side body screws before spool and final chassis assembly','preassembly_subset_clear':all(p['clear_in_declared_subset'] for p in paths if p['label']!='same base screw with motor present'),'paths':paths}
    def axis_surface(name,radius,point,direction):
        choices=[]
        for face in lookup[name]['shape'].Faces():
            s=BRepAdaptor_Surface(face.wrapped)
            if s.GetType()!=GeomAbs_Cylinder:continue
            c=s.Cylinder()
            if abs(c.Radius()-radius)>1e-5:continue
            a=c.Axis().Direction();p=c.Axis().Location();a=np.array([a.X(),a.Y(),a.Z()]);p=np.array([p.X(),p.Y(),p.Z()])
            error=math.degrees(math.acos(float(np.clip(abs(a@direction),-1,1))))
            distance=float(np.linalg.norm(np.cross(point-p,a)))
            b=exact_bounds(face)
            choices.append({'axis_error_deg':error,'line_distance_mm':distance,'radius_mm':c.Radius(),'point_world_mm':p.tolist(),'axis_world':a.tolist(),'surface_bounds_mm':[getattr(b,k) for k in ['xmin','ymin','zmin','xmax','ymax','zmax']]})
        best=min(choices,key=lambda c:c['line_distance_mm']+c['axis_error_deg']) if choices else None
        return {'component':name,'required_radius_mm':radius,'best_surface':best,'passed':bool(best and best['line_distance_mm']<1e-4 and best['axis_error_deg']<1e-4)}
    for m in MOUNTS:
        n=m['name'];F=np.array(m['frame_world']);point=lambda p:(F@np.r_[p,1])[:3];body_interfaces=[];base_interfaces=[]
        for y,z in mc['body_axis_yz_local_mm']:
            p=point([0,y,z]);direction=F[:3,0]
            surfaces=[axis_surface(n+s,rad,p,direction) for s,rad in [('_motor',.8),('_cradle',1.025),('_body_screws',.95)]]
            body_interfaces.append({'axis_point_world_mm':p.tolist(),'axis_world':direction.tolist(),'surface_checks':surfaces,'passed':all(x['passed'] for x in surfaces),'head_seats_local_x_mm':mc['body_head_seat_x_local_mm'],'screws_on_axis':2,'nominal_shaft_length_each_mm':8,'nominal_cradle_grip_each_mm':1.3,'nominal_motor_penetration_each_mm':6.7,'engagement_status':'Axial envelopes only; factory pilot thread formation and pullout strength not validated'})
        for x,y in mc['base_hole_xy_local_mm']:
            p=point([x,y,-13]);direction=F[:3,2]
            surfaces=[axis_surface(n+s,rad,p,direction) for s,rad in [('_cradle',1.025),('_mount_plate',mc['adapter_hole_radius_mm']),('_base_screws',1),('_base_nuts',1)]]
            base_interfaces.append({'axis_point_world_mm':p.tolist(),'axis_world':direction.tolist(),'surface_checks':surfaces,'passed':all(x['passed'] for x in surfaces),'cradle_grip_mm':1.2,'adapter_grip_mm':mc['adapter_dimensions_mm'][2],'total_grip_mm':4,'screw_underhead_to_tip_mm':6,'nut_thickness_mm':1.6,'nominal_tip_beyond_nut_mm':.4,'engagement_status':'Nominal envelope dimensions; threads, torque, tolerance and fastener specification not validated'})
        pairs=[(n+'_motor',n+'_cradle'),(n+'_body_screws',n+'_cradle'),(n+'_cradle',n+'_mount_plate'),(n+'_base_screws',n+'_cradle'),(n+'_base_nuts',n+'_mount_plate')]
        contacts=[{'a':a,'b':b,'minimum_surface_distance_mm':lookup[a]['shape'].distance(lookup[b]['shape']),'type':'nominal seating contact; minimum distance alone does not establish bearing area or stiffness'} for a,b in pairs]
        rows.append({'motor':n+'_motor','frame_world':m['frame_world'],'nodes':[n+s for s in ['_motor','_body_screws','_cradle','_base_screws','_base_nuts','_mount_plate']],'edges':[{'from':n+'_motor','to':n+'_cradle','fasteners':[n+'_body_screws'],'type':'factory pilot-hole side screws'},{'from':n+'_cradle','to':n+'_mount_plate','fasteners':[n+'_base_screws',n+'_base_nuts'],'type':'through screws and underside nuts'}],'body_interfaces':body_interfaces,'base_interfaces':base_interfaces,'nominal_seating_distances':contacts,'driver_screen':driver_screen(n,F),'stage_axes_passed':all(x['passed'] for x in body_interfaces+base_interfaces),'complete_chain_to_chassis':False,'chassis_anchor_edges':[],'unresolved_edge':n+'_mount_plate -> actual chassis rail/anchor','assembly_access_status':'Module preassembly has bounded driver tests below; no assembled-hand driver, full screw insertion, nut-loading or connector-removal sweep has passed'})
    report={'revision':C['revision'],'scope':'Eight palm-fixed motor-to-S101-cradle-to-adapter interfaces; actual OCC cylinder self-check. Independent review reopens saved STEP.','motor_count':len(rows),'stage_axes_passed':all(x['stage_axes_passed'] for x in rows),'complete_chassis_load_path_count':0,'all_chassis_load_paths_complete':False,'manufacturing_ready':False,'modules':rows,'limitations':[mc['hardware_limitations'],mc['support_status'],'Vendor frame material uses the retained V9 mass estimate','No support stiffness, tendon reaction load, screw tightening or continuous motion test','Moving thumb mount fasteners unchanged and incomplete']}
    (ROOTOUT/'reviews'/'mounting.json').write_text(json.dumps(report,indent=2),encoding='utf8')
    if not report['stage_axes_passed']:raise ValueError('Palm mounting cylinder axes failed; see reviews/mounting.json')
    return {'module_count':len(rows),'motor_to_adapter_axes_passed':report['stage_axes_passed'],'complete_chassis_load_path_count':0,'details':'reviews/mounting.json'}

def export():
    assembly=cq.Assembly(name='Remote_drive_v1_full_hand')
    for p in PARTS:assembly.add(p['shape'],name=p['name'],color=cq.Color(*COLORS.get(p['kind'],COLORS['printed'])))
    assembly.save(str(ROOTOUT/'assembly.step'))
    with zipfile.ZipFile(ROOTOUT/'assembly.step.zip','w',compression=zipfile.ZIP_DEFLATED,compresslevel=9) as z:
        z.write(ROOTOUT/'assembly.step',arcname='assembly.step')
    exportGLTF(assembly,str(ROOTOUT/'assembly.glb'),binary=True,tolerance=0.65,angularTolerance=0.4)
    # New manufactured geometry exported individually; imported assets remain original references.
    for p in PARTS:
        if p['source'] is None and not p['route']:cq.exporters.export(p['shape'],str(OUT/'parts'/(p['name']+'.step')))
    return assembly

def render():
    mesh=[]
    for p in PARTS:
        verts,faces=p['shape'].tessellate(0.65,0.4)
        vs=np.array([[v.x,v.y,v.z] for v in verts]);tris=vs[np.array(faces)] if faces else np.empty((0,3,3))
        mesh.append((p,tris))
    fig=plt.figure(figsize=(16,8),facecolor='#f3f5f8')
    for ix,(title,elev,azim,cutaway) in enumerate([('Original lengths and roots / 20 independent axes',22,-68,False),('Palm cutaway / 8 relocated motors + moving thumb pair',65,-90,True)]):
        ax=fig.add_subplot(1,2,ix+1,projection='3d');ax.set_facecolor('#f3f5f8')
        for p,tris in mesh:
            if cutaway and p['name'] in ['Palm_front_cover','Palm_back_chassis']:continue
            color=COLORS.get(p['kind'],COLORS['printed']);alpha=0.12 if p['name']=='Palm_front_cover' else 1
            if p['name']=='Palm_back_chassis':alpha=0.38
            pc=Poly3DCollection(tris,facecolor=color,edgecolor='none',alpha=alpha,rasterized=True);ax.add_collection3d(pc)
        ax.set(xlim=(-125,190),ylim=(-40,350),zlim=(-90,75));ax.set_box_aspect((315,390,165));ax.view_init(elev,azim);ax.set_axis_off();ax.set_title(title,fontsize=11,pad=6)
    fig.suptitle('REMOTE DRIVE V1 | Full five-finger construction candidate',fontsize=17,fontweight='bold',x=.5,y=.96)
    fig.text(.04,.04,'232 × 195 mm palm and V9 roots preserved. Purple = route envelopes; orange = pulleys/spools.\nTendon wraps, tensioners, distal feedback and swept clearances remain unvalidated. NOT A FABRICATION RELEASE.',fontsize=10,color='#444')
    fig.savefig(ROOTOUT/'images'/'assembly.png',dpi=160,bbox_inches='tight');plt.close(fig)

if __name__=='__main__':
    try:
        print('Building exact-solid candidate',flush=True);make_frames();import_preserved();new_fingers();aggregate();print('Mass contract ready',flush=True)
        report=geometry_check();print('Geometry screened',flush=True);export();print('Compressed STEP and GLB exported',flush=True);render()
        sources=[BASE/'design.json',BASE/'scene.json',BASE/'mass_screen.json']+[BASE/'parts'/(n+'.step') for n in sorted(ASSETS)]
        changed=[str(p) for p,h in INPUT_HASHES.items() if digest(p)!=h]
        if changed:raise RuntimeError('Input changed during CAD generation: '+', '.join(changed))
        source_hashes={str(p.relative_to(D.parent.parent)).replace('\\','/'):INPUT_HASHES[p] for p in sources}
        (OUT/'source_hashes.json').write_text(json.dumps(source_hashes,indent=2),encoding='utf8')
        (OUT/'build_summary.json').write_text(json.dumps({'status':'generated','revision':C['revision'],'cad_kernel':'CadQuery '+cq.__version__,'actuators':report['actuator_count'],'joints':len(JOINTS),'mass_kg':sum(p['mass_kg'] for p in PARTS),'shape_count':len(PARTS),'overlap_pairs':len(report['open_pose_exact_new_cross_body_overlap_pairs']),'design_sha256':INPUT_HASHES[D/'design.json'],'generator_sha256':INPUT_HASHES[Path(__file__).resolve()],'input_hash_guard':{'captured_before_read':True,'end_check_passed':True,'checked_files':len(INPUT_HASHES)}},indent=2),encoding='utf8')
        print('Build complete',flush=True)
    except Exception:
        traceback.print_exc();raise
