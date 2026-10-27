"""Correct tap/pin classification without altering the completed first native run."""
import importlib.util
import sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm062_interpret',Path(__file__).with_name('model.py'))
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
n,np,d,ROOT,HERE=a.n,a.np,a.d,a.ROOT,a.HERE


def calculate(raw):
    old=raw['vendor_interface'];centre=np.array(old['source_motor_axis_xy_mm'])
    front=next(z for z in old['output_front_planes'] if z['area_mm2']>300)
    zfront=front['z']
    M=np.array(old['reference_transform']);M[1,3]=-230.+zfront
    def holes(radius):
        selected=[q for q in old['cylinders'] if abs(q['radius']-radius)<1e-6 and
            abs(q['axis_radius_from_motor_centre_mm']-10.)<1e-5 and q['bbox'][0][2]<zfront+1e-5]
        result=[]
        for q in selected:
            xy=q['centre'][:2]
            result.append(dict(source_face=q['index'],source_axis_xy_mm=xy,source_radius_mm=q['radius'],
                radial_mm=float(np.linalg.norm(np.array(xy)-centre)),
                source_trim_Z_mm=[q['bbox'][0][2],q['bbox'][1][2]],
                world_front_axis_point_mm=(M@np.r_[xy,zfront,1.])[:3]))
        return result
    taps,pins=holes(1.25),holes(1.5)
    assert len(taps)==6 and len(pins)==2
    probes=[]
    for i,h in enumerate(taps):
        pt=np.array(h['world_front_axis_point_mm'])
        probes.append(dict(name='arm062_output_tap_axis_continuation_'+str(i),
            shape=a.cq.Solid.makeCylinder(h['source_radius_mm'],4.,a.cq.Vector(*pt),a.cq.Vector(0,1,0)),
            body='forearm',color=[.85,.48,.2],role='source_tap_axis_reference_not_selected_clearance_bore_or_installed_bolt'))
    root_records=raw['native']['new_axial_entry4__all_existing871']['overlaps']
    correction=dict(completed=True,initial_field_output_M3_cylinder_axes_is_misclassified=True,
        initial_field_actually_describes_two_dowel_holes=True,initial_code_raw_native_and_reference_exports_preserved=True,
        corrected_output_M3_taps=taps,corrected_output_dowels=pins,
        source_M3_tap_representation_diameter_mm=2.5,source_tap_trim_length_including_numerical_bbox_pad_mm=[q['source_trim_Z_mm'][1]-q['source_trim_Z_mm'][0] for q in taps],
        drawing_thread_M3_depth_mm=6.,source_cylinder_trim_not_usable_thread_length=True,
        exact_source_front_plane=front,exact_plane_reference_transform=M,
        initial_bbox_datum_offset_from_exact_plane_mm=float(old['source_output_front_Z_mm']-zfront),
        exact_output_front_world_Y_mm=-230.,existing_flange_rear_world_Y_mm=-230.,
        output_front_and_flange_plane_coincidence_is_registration_assumption=True,
        old_nominal_case_four_mm_gap_is_not_a_four_mm_output_coupling_length=True,
        native_interface_contact_or_torque_transfer_not_approved=True,
        drawing_case_front_relative_to_output_mm=1.5,body31_in_drawing_measured_from_output_front=True,
        actual_stator_mounting_and_clock_not_qualified=True,
        new_axis_probes_diameter_mm=2.5,new_axis_probes_length_mm=4.,
        probe_basis='Exact STEP tap axes extended across existing4mm plate thickness. Minimum axis reservation only,not a chosen through-hole clearance,bolt length/thread engagement or finished coupling.',
        first_entry_records_classification=dict(connected_carrier_reference_space_intersections=[q for q in root_records if q['part']==a.p.NAME],
            coaxial_protruding_bolt_in_solid_nut_reference=[q for q in root_records if q['part'].startswith('arm054_M3x12')],
            selected_tool_unsupported_so_not_confirmed_tool_collisions=True,
            no_native_record_deleted_or_excluded=True))
    return correction,probes


def main():
    out=HERE/'reviews/interface_followup.json';assert not out.exists()
    raw=n.p.read(HERE/'reviews/check.json');assert raw['completed'] and raw['input_hash_guard']
    correction,probes=calculate(raw)
    # Only the two existing arm parts are needed for these new axis relations.
    rows=[]
    for name in ['Roll_output_flange','Forearm_box_beam']:
        rel='cad/modular_roots_v9/parts/'+name+'.step'
        rows+=n.c.load([dict(name=name,step=rel,sha256_step=n.m.drive.sha(ROOT/rel))])
    native=n.c.material(probes,rows)
    correction.update(native=native,new_native_groups=1,scene=n.a.a.prior.export(probes,HERE/'generated/output_axis_reference'),
        old_entry_groups_repeated=0,old_ARM061_interface_queries_repeated=0,
        source_sha256={f.relative_to(ROOT).as_posix():n.m.drive.sha(f) for f in [Path(__file__),HERE/'model.py',HERE/'reviews/check.json']})
    correction['input_hash_guard']=all(n.m.drive.sha(ROOT/f)==h for f,h in correction['source_sha256'].items())
    n.a.save(out,correction)
    print('corrected taps6 pins2 new axis group',native['pairs_in_scope'],native['exact_pairs'],len(native['overlaps']),flush=True)


if __name__=='__main__':main()
