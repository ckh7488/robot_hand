"""Connected passive two-axis roots for the accepted FORM002 axes.

No work is performed at import. build_root_modules(design) returns world-mm
parts, old names to remove, and a JSON-safe geometric interface contract.
apply_palm_mounts(parts, design) modifies only a candidate copy of the palm.

This is a manufacturing CONCEPT: custom-machined polymer/aluminium carriers,
reamed bronze bushings, nominal steel pins/retainers and M3 hardware. Nominal
fits, threads, spring-clip stress, strength, fatigue and cable drive are not
approved. All original axes, digit lengths and thumb21mm lift remain intact.
"""
from pathlib import Path
import argparse,hashlib,json,math

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
BLUE=(.22,.38,.56);TEAL=(.18,.48,.46);BRONZE=(.62,.45,.20);STEEL=(.55,.58,.62)
RHO={'polymer_assumption':1.25e-6,'bronze_assumption':8.8e-6,'steel_assumption':7.85e-6}

def _configs(design):
    return [*design['fingers'],dict(design['thumb'],name='thumb')]

def _cyl(cq,r,h,p,axis=(0,0,1)):
    return cq.Solid.makeCylinder(r,h,cq.Vector(*p),cq.Vector(*axis))

def _ring(cq,r,ri,h,p,axis=(0,0,1)):
    return _cyl(cq,r,h,p,axis).cut(_cyl(cq,ri,h+2,tuple(p[i]-axis[i] for i in range(3)),axis))

def _box(cq,sz,p):
    return cq.Workplane('XY').box(*sz).translate(p).val()

def _world(shape,f):
    return shape.rotate((0,0,0),(0,0,1),f.get('base_yaw_deg',0)).translate(f['root'])

def _point(p,f):
    a=math.radians(f.get('base_yaw_deg',0));c,s=math.cos(a),math.sin(a)
    return [f['root'][0]+c*p[0]-s*p[1],f['root'][1]+s*p[0]+c*p[1],f['root'][2]+p[2]]

def _axis(f):
    a=math.radians(f.get('base_yaw_deg',0));return [math.cos(a),math.sin(a),0]

def _mount_legs(f):
    """Keep upper housing anchors; move both thumb feet equally inside the palm.

    Equal displacement preserves the original 20 mm fastener separation. All
    lower hardware and geometric interface metadata use these same coordinates.
    """
    delta_world=(-6.,12.) if f['name']=='thumb' else (0.,0.)
    a=math.radians(f.get('base_yaw_deg',0));c,s=math.cos(a),math.sin(a)
    dx=c*delta_world[0]+s*delta_world[1]
    dy=-s*delta_world[0]+c*delta_world[1]
    return [dict(index=i,upper_xy_mm=[x,-8.],lower_xy_mm=[x+dx,-8.+dy],
                 lower_shift_world_mm=list(delta_world),lower_vertical_height_mm=10.)
            for i,x in enumerate([-10.,10.],1)]

def _rect_wire(cq,x,y,z):
    return cq.Wire.makePolygon([cq.Vector(x+dx,y+dy,z)
        for dx,dy in [(-5,-5),(5,-5),(5,5),(-5,5)]],close=True)

def _mount_leg(cq,leg,floor):
    x,y=leg['lower_xy_mm'];ux,uy=leg['upper_xy_mm']
    if (x,y)==(ux,uy):return _box(cq,(10,10,-22-floor),(x,y,(floor-22)/2))
    # The straight lower section carries the finite bolt/nut cavity. A ruled
    # loft joins the unchanged upper anchor; small union overlaps avoid a
    # coincident-only connection. No joint axis or exterior skin is relocated.
    top=floor+leg['lower_vertical_height_mm']
    lower=_box(cq,(10,10,top-floor),(x,y,(floor+top)/2))
    slant=cq.Solid.makeLoft([_rect_wire(cq,x,y,top-.25),
                           _rect_wire(cq,ux,uy,-21.5)],ruled=True)
    return lower.fuse(slant).clean()

def _clip_x(cq,x,ri,ro,width,z):
    s=_ring(cq,ro,ri,width,(x,0,z),(1,0,0))
    return s.cut(_box(cq,(width+2,ro+1,2.6),(x+width/2,(ro+1)/2,z))).clean()

def _clip_z(cq,z):
    s=_ring(cq,8,5.25,.8,(0,0,z))
    return s.cut(_box(cq,(10,5.5,2),(5,0,z+.4))).clean()

def _hull2(points):
    points=sorted(set(points))
    def cross(o,a,b):return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    lower=[];upper=[]
    for p in points:
        while len(lower)>1 and cross(lower[-2],lower[-1],p)<=0:lower.pop()
        lower.append(p)
    for p in reversed(points):
        while len(upper)>1 and cross(upper[-2],upper[-1],p)<=0:upper.pop()
        upper.append(p)
    return lower[:-1]+upper[:-1]

def _nut_sweep(cq,f,x,y=-8.):
    # Exact convex translation sweep of the nut's outer hex prism. Filling its
    # central hole is conservative for housing clearance, which is the scope.
    pts=[(x+3.175*math.cos(a*math.pi/3),y+dy+3.175*math.sin(a*math.pi/3))
         for a in range(6) for dy in [-9,0]]
    s=cq.Workplane('XY').polyline(_hull2(pts)).close().extrude(2.4).translate((0,0,-37.1-f['root'][2])).val()
    return _world(s,f)

def build_root_modules(design):
    import cadquery as cq
    parts=[];removed=[];probes=[];interfaces=[];connections=[];seats=[];mounts=[];insertions=[]
    def add(name,s,body,material,role,color):
        s=s.clean()
        if not s.isValid() or len(s.Solids())!=1:raise ValueError('Root part not one valid solid: '+name)
        parts.append(dict(name=name,shape=s,color=color,role=role,body=body,
            density_kg_per_mm3=RHO[material],material=material,
            volume_mm3=s.Volume(),valid=True,solid_count=1))
    for f in _configs(design):
        n=f['name'];thumb=n=='thumb';h=f.get('flexion_offset',[0,0,0])[2]
        if f.get('flexion_offset',[0,0,0])[:2]!=[0,0]:raise ValueError('Only preserved axial thumb lift supported')
        body=n+'_base_sweep' if thumb else n+'_spread'
        child=n+'_opposition' if thumb else n+'_mcp'
        col=TEAL if thumb else BLUE;rootz=f['root'][2]
        floor=-40.5-rootz
        housing=_ring(cq,15.5,8.05,6,(0,0,-22))
        legs=_mount_legs(f)
        for leg in legs:housing=housing.fuse(_mount_leg(cq,leg,floor))
        # The legs continue past the lower flange/washer/clip stack. Relieve
        # only their inner upper corners; the rear feet and outer webs remain
        # integral with the housing. This is actual geometry, not a clash waiver.
        housing=housing.cut(_cyl(cq,9.65,3.5,(0,0,-25.5)))
        # The slanted webs deliberately overlap the annulus for a solid union.
        # Restore its complete bearing bore after that union, before hardware.
        housing=housing.cut(_cyl(cq,8.05,6,(0,0,-22)))
        # Final axial holes and nut pockets are cut after all legs are fused.
        for leg in legs:
            x,y=leg['lower_xy_mm']
            # The thumb bolt ends at world Z=-32.5. Its blind clearance ends
            # at -31.5, below the slanted web, retaining 1 mm axial tip space.
            # A long vertical cutter would unnecessarily notch the angled web.
            drill_length=10 if thumb else 40
            housing=housing.cut(_cyl(cq,1.7,drill_length,(x,y,floor-1)))
            cavity=cq.Workplane('XY').polygon(6,6.58).extrude(2.6).translate((x,y,-37.1-rootz)).val()
            slot=_box(cq,(6.6,7,2.6),(x,y-3,-35.8-rootz))
            housing=housing.cut(cavity.fuse(slot))
        # The one-piece turntable, shaft and both fork cheeks replace the old
        # unconnected ring/forks. The platform is below the rotating eye.
        carrier=_cyl(cq,14,3,(0,0,-15.5))
        carrier=carrier.fuse(_cyl(cq,6,9.5,(0,0,-25)))
        groove=_ring(cq,6.5,5.2,1,(0,0,-24))
        carrier=carrier.cut(groove)
        for x in [-13,9]:
            cheek=_cyl(cq,7.5,4,(x,0,h),(1,0,0))
            web=_box(cq,(4,8,h+9.5),(x+2,-4,(h-17.5)/2))
            carrier=carrier.fuse(cheek).fuse(web)
        # This last cut removes the exact blocked-web defect found at all10
        # old fork sites; no later union may fill the bore back in.
        carrier=carrier.cut(_cyl(cq,2.05,34,(-17,0,h),(1,0,0))).clean()
        hn=n+'_root_fixed_housing';cn=n+'_root_spread_carrier'
        add(hn,_world(housing,f),'palm','polymer_assumption','root_fixed_housing',col)
        add(cn,_world(carrier,f),body,'polymer_assumption','integral_root_carrier',col)
        connections.append(dict(id=n+'_integral_ring_forks_shaft',part_names=[cn],
            method='One fused BRep solid, with final pin bore cut after union'))
        connections.append(dict(id=n+'_integral_housing_two_mount_legs',part_names=[hn],
            method='One fused BRep solid'))
        # Two flanged half-bushes can be inserted from opposite sides, unlike
        # an impossible one-piece bearing with flanges on both ends.
        for label,z,flange_z in [('lower',-22,-22.5),('upper',-19,-16)]:
            b=_ring(cq,8.04,6.025,3,(0,0,z)).fuse(_ring(cq,9.5,6.025,.5,(0,0,flange_z)))
            name=n+'_spread_'+label+'_bushing'
            add(name,_world(b,f),'palm','bronze_assumption','split_spread_bushing',BRONZE)
            interfaces.append(dict(id=name+'_journal',part=name,axis_origin_mm=_point((0,0,0),f),
                axis_direction=[0,0,1],radius_mm=6.025,span_mm=[z,z+3]))
        add(n+'_spread_lower_washer',_world(_ring(cq,9.5,6.1,.5,(0,0,-23)),f),body,'steel_assumption','spread_thrust_washer',STEEL)
        add(n+'_spread_retaining_clip',_world(_clip_z(cq,-23.9),f),body,'steel_assumption','nominal_spring_clip',STEEL)
        # Headed flex pin, groove and distinct snap-retainer give an axial
        # load path at both ends. Clip stress and shaft fits remain unvalidated.
        pin=_cyl(cq,2,29.4,(-14,0,h),(1,0,0)).fuse(_cyl(cq,3.5,2,(-16,0,h),(1,0,0)))
        pin=pin.cut(_ring(cq,2.4,1.6,.8,(14.1,0,h),(1,0,0)))
        pn=n+'_root_flex_pin'
        add(pn,_world(pin,f),body,'steel_assumption','headed_grooved_flex_pin',STEEL)
        for label,x in [('left',-14),('right',13)]:
            add(n+'_flex_'+label+'_pin_washer',_world(_ring(cq,3.7,2.05,1,(x,0,h),(1,0,0)),f),body,'steel_assumption','pin_washer',STEEL)
        add(n+'_flex_retaining_clip',_world(_clip_x(cq,14.15,1.65,3.5,.7,h),f),body,'steel_assumption','nominal_spring_clip',STEEL)
        for label,x,flange_x in [('left',-8,-8.6),('right',0,8)]:
            b=_ring(cq,3.38,2.015,8,(x,0,h),(1,0,0)).fuse(_ring(cq,4.5,2.015,.6,(flange_x,0,h),(1,0,0)))
            bn=n+'_root_proximal_'+label+'_bushing'
            add(bn,_world(b,f),child,'bronze_assumption','flanged_proximal_bushing',BRONZE)
            interfaces.append(dict(id=bn+'_journal',part=bn,axis_origin_mm=_point((0,0,h),f),axis_direction=_axis(f),radius_mm=2.015,span_mm=[x,x+8]))
        for label,x in [('left',-9),('right',8.65)]:
            add(n+'_flex_'+label+'_thrust_washer',_world(_ring(cq,4.8,2.05,.35,(x,0,h),(1,0,0)),f),body,'steel_assumption','flex_thrust_washer',STEEL)
        for label,x in [('left',-13),('right',9)]:
            probes.append(dict(id=n+'_'+label+'_finite_pin_clearance',origin_mm=_point((x,0,h),f),
                direction=_axis(f),radius_mm=2,length_mm=4,against_parts=[cn],scope='Actual fork wall, including final-cut web'))
            interfaces.append(dict(id=n+'_'+label+'_fork_bore',part=cn,
                axis_origin_mm=_point((0,0,h),f),axis_direction=_axis(f),radius_mm=2.05,span_mm=[x,x+4]))
        interfaces.append(dict(id=n+'_actual_flex_pin',part=pn,axis_origin_mm=_point((0,0,h),f),
            axis_direction=_axis(f),radius_mm=2,span_mm=[-14,14.1]))
        if thumb:
            probes.append(dict(id='thumb_source_eye_final_pin_clearance',origin_mm=_point((-8,0,h),f),
                direction=_axis(f),radius_mm=2,length_mm=16,against_parts=['thumb_segment1_thenar_shell'],
                scope='Inherited thenar web final bore, cut in candidate copy after all source unions'))
            interfaces.append(dict(id='thumb_source_eye_final_bore',part='thumb_segment1_thenar_shell',
                axis_origin_mm=_point((0,0,h),f),axis_direction=_axis(f),radius_mm=3.4,span_mm=[-8,8]))
        probes.append(dict(id=n+'_spread_journal_clearance',origin_mm=_point((0,0,-22),f),direction=[0,0,1],
            radius_mm=6,length_mm=6,against_parts=[n+'_spread_lower_bushing',n+'_spread_upper_bushing'],scope='Spread shaft insertion envelope'))
        seats.extend([
            dict(id=n+'_upper_spread_thrust',parts=[cn,n+'_spread_upper_bushing'],plane_origin_mm=_point((0,0,-15.5),f),normal=[0,0,1],nominal_contact_area_mm2=math.pi*(9.5**2-6.025**2)),
            dict(id=n+'_upper_bush_housing',parts=[hn,n+'_spread_upper_bushing'],plane_origin_mm=_point((0,0,-16),f),normal=[0,0,1],nominal_contact_area_mm2=math.pi*(9.5**2-8.05**2))])
        for leg in legs:
            k=leg['index'];x,y=leg['lower_xy_mm']
            # Captive nut is side-insertable through an explicit rear slot.
            nut=cq.Workplane('XY').polygon(6,6.35).extrude(2.4).translate((x,y,-37.1-rootz)).val()
            nut=nut.cut(_cyl(cq,1.5,4,(x,y,-38-rootz)))
            bolt=_cyl(cq,1.5,12,(x,y,-44.5-rootz)).fuse(_cyl(cq,2.85,3,(x,y,-47.5-rootz)))
            socket=cq.Workplane('XY').polygon(6,2.9).extrude(1.8).translate((x,y,-47.5-rootz)).val()
            bolt=bolt.cut(socket)
            prefix=n+'_root_mount_'+str(k)
            add(prefix+'_bolt',_world(bolt,f),'palm','steel_assumption','nominal_M3x12_socket_bolt',STEEL)
            add(prefix+'_washer',_world(_ring(cq,3.5,1.65,.5,(x,y,-44.5-rootz)),f),'palm','steel_assumption','mount_washer',STEEL)
            add(prefix+'_nut',_world(nut,f),'palm','steel_assumption','nominal_captive_M3_nut',STEEL)
            pt=_point((x,y,0),f)
            mounts.append(dict(id=prefix,axis_point_mm=[pt[0],pt[1],-44],axis_direction=[0,0,1],
                housing=hn,chassis='Palm_tapered_chassis',bolt=prefix+'_bolt',nut=prefix+'_nut',washer=prefix+'_washer',
                shaft_radius_mm=1.5,clearance_radius_mm=1.7,nut_bore_radius_mm=1.5,
                housing_bore_span_z_mm=[-40.5,-37.1],
                floor_span_z_mm=[-44,-40.5],shaft_span_z_mm=[-44.5,-32.5],nut_span_z_mm=[-37.1,-34.7],
                nut_engagement_mm=2.4,tip_beyond_nut_mm=2.2,leg_geometry=leg,
                blind_bolt_clearance_top_z_mm=-31.5 if thumb else None,
                blind_bolt_tip_axial_space_mm=1.0 if thumb else None,
                thread_geometry='Nominal cylinders only; helical thread and purchase specification unverified'))
            a=math.radians(f.get('base_yaw_deg',0))
            insertions.append(dict(id=prefix+'_rear_nut_insertion',moving_part=prefix+'_nut',
                translations_world_mm=[[-math.sin(a)*dy,math.cos(a)*dy,0] for dy in [i*.5-9 for i in range(19)]],
                against_parts=[hn],sample_spacing_mm=.5,sample_count=19,
                scope='Rear insertion before module is mounted in palm; full system access excluded',
                continuous_generator_sweep=dict(finger=n,leg_local_x_mm=x,leg_local_y_mm=y,translation_local_y_mm=[-9,0])))
            seats.append(dict(id=prefix+'_leg_floor_seat',parts=[hn,'Palm_tapered_chassis'],
                plane_origin_mm=[pt[0],pt[1],-40.5],normal=[0,0,1],
                patch_rectangle_mm=[10,10],patch_yaw_deg=f.get('base_yaw_deg',0),nominal_contact_area_mm2=100-math.pi*1.7**2))
            probes.append(dict(id=prefix+'_driver_access',origin_mm=[pt[0],pt[1],-67.5],direction=[0,0,1],
                radius_mm=1.25,length_mm=20,against_parts=['Palm_tapered_chassis',hn],scope='Axial tool approach only; hand/wrist/system access not approved'))
        removed.extend([n+'_spread_housing',n+'_spread_ring',n+'_root_fork_-13',n+'_root_fork_13',n+'_joint_pin_0',n+'_segment1_proximal_bushing'])
    report=dict(schema_version='gesture_hand_v3.root_module.v1',modules=5,new_part_count=len(parts),
        removed_names=removed,clearance_probes=probes,cylinder_interfaces=interfaces,
        solid_connections=connections,seating_interfaces=seats,mount_interfaces=mounts,assembly_translation_probes=insertions,
        axes_preserved=True,spread_axis_sign=-1,thumb_root_lift_mm=design['thumb'].get('flexion_offset',[0,0,0])[2],
        palm_modification_api='apply_palm_mounts(parts,design)',
        thumb_support_relocation=dict(lower_foot_shift_world_mm=[-6,12],both_feet_shifted=True,
            unchanged_upper_anchors_local_mm=[[-10,-8],[10,-8]],lower_vertical_height_mm=10,
            nominal_foot_patch_mm=[10,10],fastener_separation_mm=20,
            method='Integral ruled loft between relocated vertical feet and original upper housing anchors',
            axes_and_exterior_unchanged=True,strength_approved=False),
        assembly_sequence=['Preassemble each root outside the palm; insert two spread half-bushings from opposite housing faces.',
          'Position the closed lower washer from the open front before inserting the carrier shaft; temporary support is required.',
          'Lower the integral carrier shaft through bushings and the pre-positioned washer; install the external groove clip from the open front with its mouth initially facing the shaft.',
          'Install two proximal flanged bushes in the existing segment1 eye; place thrust washers in fork.',
          'Insert headed flex pin through final bores, fit opposite washer and groove clip.',
          'Slide two M3 nuts through rear leg slots before palm installation; place feet on drilled palm floor and fasten from below.'],
        nominal_fits=dict(flex_shaft_diameter_mm=4,flex_fork_bore_mm=4.1,flex_bush_ID_mm=4.03,
            proximal_bush_OD_mm=6.76,preserved_eye_bore_mm=6.8,spread_shaft_mm=12,spread_bush_ID_mm=12.05),
        manufacturing_release=False,complete_drive=False,
        limitations=['Custom bushing and retainer geometries are concept envelopes, not selected catalog parts.',
          'Nominal clearances do not specify manufacturing tolerances, process capability or bearing service life.',
          'Pin anti-rotation, spring-clip stress, bolt preload/thread tolerances and access with the full wrist are unvalidated.',
          'Clip orientation and elastic installation, temporary washer support and nominal retainer endplay remain assembly engineering tasks.',
           'Palm wall relief and ten new floor holes weaken the original palm; no strength or fatigue approval.',
           'Relocated thumb feet and slanted webs require strength, print/machining-process and tolerance review.',
          'Distal hinges, appearance plates, caps, whole-hand collisions and motor/tendon transmission remain separate work.'])
    return parts,removed,report

def cut_palm_root_interfaces(palm,design):
    """Single candidate-only pocket/hole definition for build and local audit."""
    import cadquery as cq
    cuts=[]
    for f in _configs(design):
        lower_world=-30.3 if f['name']=='thumb' else f['root'][2]-23
        upper_world=f['root'][2]+9
        palm=palm.cut(_world(_cyl(cq,16.2,upper_world-lower_world,
            (0,0,lower_world-f['root'][2])),f))
        centers=[]
        for leg in _mount_legs(f):
            x,y=leg['lower_xy_mm'];q=_point((x,y,0),f);centers.append(q[:2])
            palm=palm.cut(_cyl(cq,1.7,6,(q[0],q[1],-45)))
        cuts.append(dict(finger=f['name'],root_relief_radius_mm=16.2,
            root_relief_world_z_mm=[lower_world,upper_world],floor_hole_centers_world_xy_mm=centers,
            floor_hole_radius_mm=1.7,preserved_floor_world_z_mm=[-44,-40.5]))
    palm=palm.clean()
    if not palm.isValid() or len(palm.Solids())!=1:raise ValueError('Root-drilled palm must remain one valid solid')
    return palm,dict(cuts=cuts,exterior_outer_outline_repositioned=False,
        strength_approved=False,scope='Root clearance pockets and floor holes only; adjacent open-wall edges remain a structural review item')

def apply_palm_mounts(parts,design):
    """Repair candidate copies of palm interfaces and the thumb root bore.

    The source assets are never modified. Explicitly recorded changes include
    the adjacent web/post/cover and final thumb bores, not just palm floor holes.
    """
    import cadquery as cq
    out=[];cuts=[];modified=[];before_palm=None;after_palm=None
    for p in parts:
        name=p['name'];s=p['shape'];before=s.Volume();change=None
        if name=='Palm_tapered_chassis':
            before_palm=before
            s,root_cuts=cut_palm_root_interfaces(s,design)
            for f in _configs(design):
                # The larger shallow relief clears proximal shells above the
                # preserved floor and housing feet; it is not a strength proof.
                if f['name']!='thumb':s=s.cut(_cyl(cq,32,9,(f['root'][0],f['root'][1],-4)))
                cuts.append(dict(finger=f['name'],root_relief_radius_mm=16.2,
                    relief_world_z_mm=next(c['root_relief_world_z_mm'] for c in root_cuts['cuts'] if c['finger']==f['name']),floor_hole_radius_mm=1.7,
                    proximal_shell_upper_relief=None if f['name']=='thumb' else dict(radius_mm=32,world_z_mm=[-4,5])))
            change='Root pockets, shallow proximal-shell clearance and ten floor bores'
        elif name=='Palm_side_web_-71':
            s=s.cut(_box(cq,(20,30,80),(-71,103.5,-8)))
            change='Shortened front end from Y99.5 to Y88.5 to clear little-root hardware; original remaining web retained'
        elif name in ['Cover_post_-18_104','Cover_screw_-18_104']:
            s=s.translate((0,-10,0))
            change='Retained identity relocated with mating cover passage from XY(-18,104) to(-18,94); screw remains a provisional head'
        elif name=='Palm_shaped_front_cover':
            s=s.cut(_cyl(cq,1.8,5,(-18,94,10)))
            change='New passage at(-18,94); old(-18,104) hole lies entirely in the existing R19 root opening'
        elif name=='thumb_segment1_thenar_shell':
            f=dict(design['thumb'],name='thumb');h=f.get('flexion_offset',[0,0,0])[2]
            # Final cuts occur after every inherited eye/web/shell union.
            s=s.cut(_world(_cyl(cq,3.4,16.02,(-8.01,0,h),(1,0,0)),f))
            for x in [-13,8.2]:
                cutter=_cyl(cq,3.4,4.8,(x,0,0),(1,0,0))
                cutter=cutter.rotate((0,0,0),(0,1,0),f.get('second_roll_deg',-65)).rotate((0,0,0),(0,0,1),f.get('second_yaw_deg',25)).translate((0,f['links'][0],h))
                s=s.cut(_world(cutter,f))
            change='Final proximal R3.4 bore and two inherited distal R3.4 sleeve bores recut after all source unions'
        if change is None:out.append(p);continue
        s=s.clean()
        if not s.isValid() or len(s.Solids())!=1:raise ValueError('Modified source part must remain one valid solid: '+name)
        if name=='Palm_tapered_chassis':after_palm=s.Volume()
        out.append(dict(p,shape=s,volume_mm3=s.Volume(),source='gesture_hand_v3_interface_repair'))
        modified.append(dict(name=name,change=change,original_volume_mm3=before,new_volume_mm3=s.Volume(),one_valid_solid=True))
    if not any(p['name']=='Palm_tapered_chassis' for p in out):raise ValueError('Palm chassis missing')
    return out,dict(status='Candidate-only explicit source interface repairs',cuts=cuts,modified_parts=modified,
        modified_names=[r['name'] for r in modified],original_volume_mm3=before_palm,new_volume_mm3=after_palm,
        one_valid_solid=True,strength_approved=False,
        limitations=['Shallow wall relief and shortened web require structural review.',
          'Moved cover post has a matching passage but inherited screw geometry is still a head envelope, not a complete threaded fastener.',
          'These are local interface changes; all-motion finger-to-finger clearance and fabrication remain unapproved.'])

def validate_local(design,output,palm_step=None,design_source=None):
    """Bounded root-only smoke test and STEP export; full-hand review separate."""
    import cadquery as cq
    import importlib.util
    source=Path(__file__).resolve()
    inputs=[source]
    if design_source is not None:inputs.append(Path(design_source).resolve())
    if palm_step is not None:
        inputs.append(Path(palm_step).resolve())
        inputs.append(HERE/'geometry_audit.py')
        verifier_path=ROOT/'cad/articulation_drive_v2/verify.py'
        expected='79a0b66b8c5ede499e870e954b32ac0d2e679c4f96a9e3e0bf4d0fdca44680f3'
        if hashlib.sha256(verifier_path.read_bytes()).hexdigest()!=expected:
            raise ValueError('Frozen finite-interface verifier changed')
        inputs.append(verifier_path)
    hashes=lambda:{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    before=hashes();start=before[str(source)]
    parts,removed,report=build_root_modules(design);by={p['name']:p for p in parts}
    checks=[]
    for p in report['clearance_probes']:
        if any(n not in by for n in p['against_parts']):continue
        probe=_cyl(cq,p['radius_mm'],p['length_mm'],p['origin_mm'],p['direction'])
        for name in p['against_parts']:
            checks.append(dict(id=p['id'],part=name,intersection_mm3=probe.intersect(by[name]['shape']).Volume()))
    insertion_checks=[]
    configs={f['name']:f for f in _configs(design)}
    for row in report['assembly_translation_probes']:
        definition=row['continuous_generator_sweep'];f=configs[definition['finger']]
        sweep=_nut_sweep(cq,f,definition['leg_local_x_mm'],definition.get('leg_local_y_mm',-8.))
        for name in row['against_parts']:
            insertion_checks.append(dict(id=row['id'],against_part=name,
                continuous_outer_hex_sweep_material_mm3=sweep.intersect(by[name]['shape']).Volume(),
                scope='Straight rear translation before palm mounting; hex outer prism includes nut bore conservatively'))
    clashes=[];tested=0
    for i,a in enumerate(parts):
        aa=a['shape'].BoundingBox()
        for b in parts[i+1:]:
            bb=b['shape'].BoundingBox()
            if not all(min(getattr(aa,k+'max'),getattr(bb,k+'max'))-max(getattr(aa,k+'min'),getattr(bb,k+'min'))>0 for k in ['x','y','z']):continue
            v=a['shape'].intersect(b['shape']).Volume();tested+=1
            if v>.02:clashes.append(dict(parts=[a['name'],b['name']],volume_mm3=v))
    out=Path(output);out.mkdir(parents=True,exist_ok=True);(out/'parts').mkdir(exist_ok=True)
    for p in parts:cq.exporters.export(p['shape'],str(out/'parts'/(p['name']+'.step')))
    floor_review=None
    if palm_step is not None:
        # Read the actual accepted-form reconstruction. Apply exactly the root
        # pockets/floor bores used by gesture_hand_v3.build.mount_roots; unrelated
        # v2 web/cover changes in apply_palm_mounts are intentionally not invoked.
        palm=cq.importers.importStep(str(palm_step)).val()
        if not palm.isValid() or len(palm.Solids())!=1:raise ValueError('Palm input must be one valid solid')
        original_volume=palm.Volume()
        palm,palm_cut_report=cut_palm_root_interfaces(palm,design)
        if not palm.isValid() or len(palm.Solids())!=1:raise ValueError('Root-drilled palm became invalid/disconnected')
        cq.exporters.export(palm,str(out/'parts/Palm_tapered_chassis_root_mounts.step'))
        spec=importlib.util.spec_from_file_location('_gesture_root_interface_verifier',verifier_path)
        verifier=importlib.util.module_from_spec(spec);spec.loader.exec_module(verifier)
        from geometry_audit import planar_faces,palm_thickness_screen
        # Local dependency injection only: the v2 source file remains frozen.
        verifier.planar_faces=planar_faces
        shapes={name:p['shape'] for name,p in by.items()};shapes['Palm_tapered_chassis']=palm
        seats=[verifier.seating_check(row,shapes) for row in report['seating_interfaces'] if row['id'].endswith('_leg_floor_seat')]
        finite_mounts=[verifier.mounting_check(row,shapes) for row in report['mount_interfaces']]
        palm_clashes=[];palm_pairs=0;pb=verifier.bounds(palm)
        for part in parts:
            bounds=verifier.bounds(part['shape'])
            if not all(min(pb[1,i],bounds[1,i])-max(pb[0,i],bounds[0,i])>0 for i in range(3)):continue
            common=part['shape'].intersect(palm,tol=1e-6);volume=float(common.Volume());palm_pairs+=1
            if volume>.02:palm_clashes.append(dict(parts=[part['name'],'Palm_tapered_chassis'],volume_mm3=volume))
        washers=[by['thumb_root_mount_'+str(i)+'_washer']['shape'] for i in (1,2)]
        thickness=palm_thickness_screen(palm,report['mount_interfaces'],design['thumb'])
        floor_review=dict(input_step=str(Path(palm_step).resolve()),input_sha256=before[str(Path(palm_step).resolve())],
            source_volume_mm3=original_volume,root_drilled_volume_mm3=palm.Volume(),one_valid_solid=True,
            palm_cut_report=palm_cut_report,thickness_sections=thickness,
            seating_checks=seats,finite_mount_checks=finite_mounts,root_palm_exact_pairs=palm_pairs,
            root_palm_unresolved_overlaps=palm_clashes,thumb_washer_gap_mm=float(washers[0].distance(washers[1])),
            passed=not palm_clashes and all(r['passed'] for r in seats+finite_mounts) and thickness['floor_3_5mm_preserved_in_sampled_sections'],
            scope='Actual saved palm STEP with candidate root pockets and floor holes; root interfaces only, no remaining hand parts or motion')
    result=dict(source_sha256=start,root_report=report,part_count=len(parts),all_single_valid=True,
        finite_probe_checks=checks,nut_insertion_sweep_checks=insertion_checks,neutral_exact_pairs=tested,neutral_unresolved_overlaps=clashes,
        palm_floor_review=floor_review,
        scope='New roots and optional actual palm interfaces only; source digit/whole-hand motion excluded')
    after=hashes();result['input_hash_guard']=dict(start=before,end=after,passed=before==after)
    result['source_end_hash_matches']=after[str(source)]==start
    result['local_geometric_checks_passed']=not clashes and all(r['intersection_mm3']<=1e-4 for r in checks) and all(
        r['continuous_outer_hex_sweep_material_mm3']<=1e-4 for r in insertion_checks) and (floor_review is None or floor_review['passed'])
    if not result['source_end_hash_matches']:raise RuntimeError('Root source changed during run')
    if before!=after:raise RuntimeError('Root validation input changed during run')
    (out/'root_module_check.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='root_report'},indent=2),flush=True)
    return result

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--design',default=str(ROOT/'cad/accepted_form_v1/design.json'))
    ap.add_argument('--output',default=str(HERE/'generated/root_module'))
    ap.add_argument('--palm-step',help='Actual accepted-form palm STEP; validates drilled floor seats and finite fastener chain')
    args=ap.parse_args();result=validate_local(json.loads(Path(args.design).read_text()),args.output,args.palm_step,args.design)
    raise SystemExit(0 if result['local_geometric_checks_passed'] else 2)
