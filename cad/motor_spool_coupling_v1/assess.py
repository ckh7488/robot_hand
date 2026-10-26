"""Separate new coupling contacts from the unchanged reference motor interior."""
import argparse,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import model as g
c,b,m,p,cq,np,HERE,ROOT=g.c,g.b,g.m,g.p,g.cq,g.np,g.HERE,g.ROOT


def assessment():
    raw=p.read(HERE/'reviews/check.json');assert len(raw['rotations'])==37 and p.read(HERE/'reviews/initial_export.json')['source_hashes_verified']
    scene=p.read(HERE/'generated/scene.json');new=c.load(scene['parts'])
    source=c.load(scene['replaced_parts'])
    original_rotor=next(z for z in source if z['name']==g.NAME+'_rotor')['shape']
    motorrow=next(z for z in scene['retained_parts'] if z['name']==g.NAME+'_motor')
    motor=c.load([motorrow])[0]['shape']
    stock=[z for z in new if z['role'].startswith('unmodified_stock')]
    stockcompound=cq.Compound.makeCompound([z['shape'] for z in stock])
    signature=g.module('_drive_stock_signature',ROOT/'cad/palm_motor_layout_v1/review_core.py').signature
    # Compare the original compound to the same three solids after splitting.
    aa,zz=signature(original_rotor,m.pack._bbox),signature(stockcompound,m.pack._bbox)
    unchanged=aa['solids']==zz['solids']==3 and np.allclose(aa['global_values'],zz['global_values'],rtol=0,atol=1e-5) and np.allclose(aa['vertices'],zz['vertices'],rtol=0,atol=1e-5)
    baseline=original_rotor.intersect(motor,tol=1e-6)
    region=g.pose(cq.Solid.makeCylinder(1.30001,7.90002,cq.Vector(-9.60001,0,0),cq.Vector(1,0,0)))
    outside=baseline.cut(region,tol=1e-6).Volume()
    expected_pair=('thumb_ip_stock_central_retainer',g.NAME+'_motor')
    records=[]
    for z in raw['rotations']:
        got=z['packaging'];old=[q for q in got['overlaps'] if (q['lead'],q['part'])==expected_pair]
        extra=[q for q in got['overlaps'] if (q['lead'],q['part'])!=expected_pair]
        same=len(old)==1 and abs(old[0]['material_intersection_mm3']-baseline.Volume())<1e-5
        records.append(dict(angle_deg=z['angle_deg'],unchanged_reference_contact_matches=same,
            new_external_overlaps=extra,wires_clear=g.clear(z['wires']),neutral_arm_clear=g.clear(z['arm']),
            operation_failures=got['operation_failures']))
    gate=unchanged and outside<.001 and all(z['unchanged_reference_contact_matches'] and not z['new_external_overlaps'] and not z['operation_failures'] and z['wires_clear'] and z['neutral_arm_clear'] for z in records)
    # This first candidate has proven rotation obstructions and its later
    # assembly study was stopped. Never require nonexistent completion keys
    # from that preserved partial report or turn it into an assembly pass.
    gate = bool(gate and raw.get('completed',False))
    return dict(completed=True,new_coupling_nominal_space_clear=bool(gate),raw_all_parts_zero_overlap=False,
        raw_report_unchanged='reviews/check.json',original_assembly_study_completed=bool(raw['completed']),stock_three_solids_geometry_unchanged=bool(unchanged),
        baseline_stock_motor_common_mm3=baseline.Volume(),baseline_common_outside_axial_reference_shaft_mm3=outside,
        reference_region=dict(axis='canonical +X',x_mm=[-9.6,-1.7],radius_mm=1.3),rotation_classification=records,
        thread_contact_proofs=raw['thread_contacts'],shared_seat_area_mm2=raw['shared_seat_area_mm2'],
        limitation='The unmodified source central screw overlaps its own source motor in the internal axial shaft/thread envelope.Original compound has exactly the same contact;it is preserved,not removed or approved as an exact motor thread design.Raw collision report remains false.This does not certify vendor internals,retainer identity/fit or motor load.New spool screws use bounded nominal tap contacts,not physical thread/strength qualification.',
        physical_approved=False,whole_hand_complete=False,tendon_to_finger_connected=False)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,default=HERE/'reviews/assessment.json');args=ap.parse_args()
    assert not args.output.exists()
    got=assessment();p.write(args.output,got);print('coupling space',got['new_coupling_nominal_space_clear'],flush=True)


if __name__=='__main__':main()
