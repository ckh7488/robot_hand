"""Small native-solid counterexamples; never loads a hand assembly or runs CAD build."""
from pathlib import Path
import argparse, copy, json, math
import cadquery as cq
import audit_fabrication as audit


def box(dx,dy,dz,x=0,y=0,z=0):
    return cq.Solid.makeBox(dx,dy,dz,cq.Vector(x,y,z))


def cylinder(radius,lo,hi):
    return cq.Solid.makeCylinder(radius,hi-lo,cq.Vector(0,0,lo),cq.Vector(0,0,1))


def cover_fixture(float_washer=False):
    top=1.75 if float_washer else 1.5
    palm=box(12,12,5,-6,-6,-5).cut(cylinder(1.8,-6,1)).cut(cylinder(3.3,-5.1,-2))
    cover=box(12,12,1,-6,-6,0).cut(cylinder(1.8,-1,2))
    washer=cylinder(4.2,top-.5,top).cut(cylinder(1.7,0,3))
    nut=cylinder(3.175,-3,-2).cut(cylinder(1.5,-4,-1))
    screw=cylinder(1.5,-4,top).fuse(cylinder(3.3,top,top+1.65))
    parts=dict(Palm_tapered_chassis=palm,Palm_shaped_front_cover=cover,
               washer=washer,nut=nut,screw=screw)
    d=dict(id='cover',centre_xy_mm=[0,0],screw='screw',nut='nut',washer='washer',
           shaft_radius_mm=1.5,shaft_span_z_mm=[-4,top],nut_span_z_mm=[-3,-2],
           washer_top_z_mm=top,nut_upper_seat_z_mm=-2)
    return parts,d


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if args.output.exists():raise FileExistsError('Preserve prior test evidence')
    v=audit.load_v2(audit.ROOT);v.planar_faces=audit.geometry_audit.planar_faces
    paths=[Path(__file__).resolve(),Path(audit.__file__).resolve(),Path(audit.geometry_audit.__file__).resolve(),audit.ROOT/'cad/articulation_drive_v2/verify.py']
    before={str(path):v.digest(path) for path in paths}
    tests=[]
    def check(name,observed,expected,detail=None):
        row=dict(name=name,observed=bool(observed),expected=bool(expected),passed=bool(observed)==bool(expected))
        if detail is not None:row['detail']=detail
        tests.append(row)
    seat=dict(parts=['a','b'],plane_origin_mm=[0,0,0],normal=[0,0,1],patch_radius_mm=1.)
    lower=box(4,4,1,-2,-2,-1);upper=box(4,4,1,-2,-2,0)
    good=audit.positive_seat(v,dict(a=lower,b=upper),seat)
    check('localized_contact_passes',good['passed'],True,good)
    remote=audit.positive_seat(v,dict(a=box(4,4,1,8,-2,-1),b=box(16,4,1,-2,-2,0)),seat)
    check('remote_coplanar_contact_does_not_seat_missing_post',remote['passed'],False,remote)
    check('remote_counterexample_has_other_contact',remote.get('unlocalized_contact_area_mm2',0)>1,True)
    missing_patch=dict(seat);missing_patch.pop('patch_radius_mm')
    row=audit.positive_seat(v,dict(a=lower,b=upper),missing_patch)
    check('undeclared_seating_region_rejected',row['passed'],False,row)
    theta=30;normal=[0,-math.sin(math.radians(theta)),math.cos(math.radians(theta))]
    rotated=dict(a=lower.rotate((0,0,0),(1,0,0),theta),b=upper.rotate((0,0,0),(1,0,0),theta))
    row=audit.positive_seat(v,rotated,dict(seat,normal=normal,patch_centre_xy_mm=[0,0]))
    check('inclined_world_Z_footprint_passes',row['passed'] and abs(row['contact_area_mm2']-math.pi/math.cos(math.radians(theta)))<1e-5,True,row)
    row=audit.positive_seat(v,rotated,dict(seat,normal=normal))
    check('rotated_normal_disk_passes',row['passed'] and abs(row['contact_area_mm2']-math.pi)<1e-5,True,row)

    key=dict(part='a',mating_part='b',plane_origin_mm=[0,0,0],plane_normal=[0,1,0],
             axial_origin_mm=[0,0,0],axial_direction=[1,0,0],span_mm=[0,10],mating_flat_offset_mm=.03)
    own=box(10,1,2,0,-1,-1);mate=box(10,1,2,0,.03,-1)
    row=audit.flat_key_check(v,dict(a=own,b=mate),key)
    check('shared_flat_key_full_span_passes',row['passed'],True,row)
    row=audit.flat_key_check(v,dict(a=own,b=box(10,1,2,0,.03,3)),key)
    check('transverse_disjoint_key_with_same_axial_extent_fails',row['passed'],False,row)
    split=cq.Compound.makeCompound([box(4,1,2,0,-1,-1),box(4,1,2,6,-1,-1)])
    row=audit.flat_key_check(v,dict(a=split,b=mate),key)
    check('two_key_fragments_with_axial_gap_fail',row['passed'],False,row)

    underside=dict(id='cover_washer',parts=['washer','Palm_shaped_front_cover'],
                   plane_origin_mm=[0,0,1],normal=[0,0,1],patch_radius_mm=4.2,patch_centre_xy_mm=[0,0])
    parts,d=cover_fixture();seating=audit.positive_seat(v,parts,underside)
    row=audit.cover_check(v,parts,d,[seating])
    check('complete_nominal_cover_stack_passes',row['passed'],True,row)
    parts_float,d_float=cover_fixture(True);floating=audit.positive_seat(v,parts_float,underside)
    row=audit.cover_check(v,parts_float,d_float,[floating])
    check('floating_washer_fails_even_when_head_and_nut_are_seated',row['passed'],False,row)
    check('floating_fixture_other_seats_really_pass',all(x['passed'] for x in row['seats']),True)
    row=audit.cover_check(v,parts,d,[])
    check('missing_cover_underside_declaration_fails',row['passed'],False,row)

    for name,valid,count,expected in [('valid_single',True,1,True),('invalid',False,1,False),('disconnected',True,2,False)]:
        gate=audit.solid_gate([dict(name=name,valid=valid,solid_count=count)])
        check('saved_part_gate_'+name,gate['passed'],expected,gate)
    check('empty_part_inventory_fails',audit.solid_gate([])['passed'],False)

    fastener=dict(id='clamp1',skin='Palm_shaped_front_cover',structure='Palm_tapered_chassis',
                  bolt='screw',nut='nut',washer='washer',axis_origin_mm=[0,0,0],axis_direction=[0,0,1],
                  shaft_radius_mm=1.5,shaft_span_local_z_mm=d['shaft_span_z_mm'],nut_span_local_z_mm=d['nut_span_z_mm'])
    definitions=[dict(id='clamp1_boss_skin',parts=['Palm_tapered_chassis','Palm_shaped_front_cover'],plane_origin_mm=[0,0,0],normal=[0,0,1],patch_radius_mm=4.8),
                 dict(underside,id='clamp1_washer_skin'),
                 dict(id='clamp1_head_washer',parts=['screw','washer'],plane_origin_mm=[0,0,1.5],normal=[0,0,1],patch_radius_mm=3.3),
                 dict(id='clamp1_nut_boss',parts=['nut','Palm_tapered_chassis'],plane_origin_mm=[0,0,-2],normal=[0,0,1],patch_radius_mm=3.175)]
    seats=[audit.positive_seat(v,parts,definition) for definition in definitions]
    row=audit.skin_fastener_check(v,parts,fastener,seats)
    check('declared_skin_clamp_requires_complete_saved_contact_chain',row['passed'],True,row)
    other=copy.deepcopy(seats)
    for row in other:row['definition']['id']=row['definition']['id'].replace('clamp1','clamp2')
    row=audit.skin_fastener_check(v,parts,fastener,other)
    check('another_clamp_seats_cannot_satisfy_missing_clamp',row['passed'],False,row)
    row=audit.skin_fastener_check(v,parts,fastener,[r for r in seats if r['definition']['id']!='clamp1_washer_skin'])
    check('skin_clamp_missing_washer_contact_fails',row['passed'],False,row)

    # A second actual clamp direction exercises the recessed axial fingertip
    # head rather than assuming the fascia's Z washer stack covers that branch.
    structure=box(12,12,8,-6,-6,-8).cut(cylinder(1.2,-9,1)).cut(cylinder(2.5,-5.2,-3.4))
    cap_plain=box(12,12,3.2,-6,-6,0).cut(cylinder(1.2,-1,4))
    cap=cap_plain.cut(cylinder(2.05,1.8,3.3))
    bolt=cylinder(1,-6,1.8).fuse(cylinder(1.9,1.8,3.0))
    nut=cylinder(2.31,-5,-3.4).cut(cylinder(1,-6,-3))
    to_y=lambda shape:shape.rotate((0,0,0),(1,0,0),-90)
    tip_parts={name:to_y(shape) for name,shape in dict(core=structure,cap=cap,bolt=bolt,nut=nut).items()}
    tip=dict(id='tip1',skin='cap',structure='core',bolt='bolt',nut='nut',local_axis='Y',
             axis_origin_mm=[0,0,0],axis_direction=[0,1,0],shaft_radius_mm=1,
             shaft_span_local_y_mm=[-6,1.8],nut_span_local_y_mm=[-5,-3.4],
             head_counterbore_radius_mm=2.05,head_span_local_y_mm=[1.8,3.0])
    tip_definitions=[dict(id='tip1_'+name,parts=names,plane_origin_mm=[0,y,0],normal=[0,1,0],patch_radius_mm=radius)
                     for name,names,y,radius in [('boss_cap',['core','cap'],0,3.5),('head_cap',['bolt','cap'],1.8,1.9),('nut_boss',['nut','core'],-3.4,2.4)]]
    tip_seats=[audit.positive_seat(v,tip_parts,d) for d in tip_definitions]
    row=audit.skin_fastener_check(v,tip_parts,tip,tip_seats)
    check('recessed_axial_Y_tip_clamp_passes',row['passed'],True,row)
    bad_tip=dict(tip_parts,cap=to_y(cap_plain))
    bad_tip_seats=[audit.positive_seat(v,bad_tip,d) for d in tip_definitions]
    row=audit.skin_fastener_check(v,bad_tip,tip,bad_tip_seats)
    check('missing_tip_head_counterbore_fails',row['passed'],False,row)

    after={str(path):v.digest(path) for path in paths}
    report=dict(schema_version='gesture_hand_v3.fabrication_counterexamples.v1',tests=tests,
                passed=all(t['passed'] for t in tests) and before==after,test_count=len(tests),
                source_sha256=before,inputs_unchanged=before==after,
                scope='Small synthetic native solids only; no hand CAD load, fabrication release, force or lifetime assessment.')
    audit.write_json(args.output,report)
    print(json.dumps(dict(passed=report['passed'],test_count=len(tests),failures=[t['name'] for t in tests if not t['passed']])),flush=True)
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
