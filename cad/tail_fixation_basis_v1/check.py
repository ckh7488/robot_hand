"""Keep both complete cord/housing materials while approaching four fasteners."""
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import basis as b
from OCP.OSD import OSD_ThreadPool


def main():
    out=b.HERE/'reviews/check.json';assert not out.exists();start=time.monotonic()
    case,source,rows=b.context();lookup={z['name']:z for z in rows}
    source['cad/tail_fixation_basis_v1/check.py']=b.m.drive.sha(Path(__file__))
    report=dict(completed=False,source_sha256=source,contacts=[],paths=[],current_all_names=list(lookup),
        hardware_rows=862,whole_housing_pieces=26,whole_cord_pieces=52,omitted_hardware=[],new_hardware=0,
        start_condition='Separate conditional axial recovery of each actual fastener from one own nominal length outward; all other hardware, both complete housings and cords remain nominal. Not warehouse insertion, helical tightening or a combined factory sequence.',
        tightening_torque_friction_compression_or_retention_solved=False,whole_hand_motion_approved=False,
        inherited_CORD043_initial_core_passed=False,inherited_same_cord_seam_hold_preserved=True,
        reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads())
    def save():b.a.save(out,report)
    probes=[];save()
    for spec in b.specifications(case):
        got,probe=b.contact(spec,lookup);report['contacts'].append(got);probes.append(probe);save()
        print('contact',got['id'],[z['distance_mm'] for z in got['surfaces']],flush=True)
    report['contact_material_scene']=b.export(probes,'contact_material');save()
    for plan in b.plans(case):
        record=dict(**plan,obstacle_names=[name for name in lookup if name!=plan['moving']],samples=[])
        report['paths'].append(record);ob=[lookup[name] for name in record['obstacle_names']];save()
        for j,d in enumerate(plan['distances_mm']):
            moving=b.moved(lookup[plan['moving']],plan,d);native=b.c.material([moving],ob)
            item=dict(outward_distance_mm=d,native=native,clear=b.m.drive.clear(native))
            record['samples'].append(item);save()
            if j in [0,len(plan['distances_mm'])-1] or not item['clear']:
                item['scene']=b.export([moving],plan['id']+'_'+str(j));save()
            print(plan['id'],d,item['clear'],len(native['overlaps']),flush=True)
            if not item['clear']:break
        record['full_declared_path_clear']=len(record['samples'])==3 and all(z['clear'] for z in record['samples']);save()
    tests=[z['native'] for path in report['paths'] for z in path['samples']]+[z['native'] for z in report['contacts']]
    report.update(completed=True,pairs_in_scope=sum(z['pairs_in_scope'] for z in tests),
        AABB_native_pairs=sum(z['exact_pairs'] for z in tests),overlap_records=sum(len(z['overlaps']) for z in tests),
        operation_failures=sum(len(z['operation_failures']) for z in tests),
        bounded_fastener_approaches_clear=all(z['full_declared_path_clear'] for z in report['paths']),
        no_slip_is_only_a_declared_boundary_condition=True,actual_tail_fixation_verified=False,
        motor_bore_radial_play_mm=.1,distal_nominal_floor_to_plate_gap_mm=.6,undeformed_cord_diameter_mm=.6,
        native_old_threading_or_same_cord_seam_replays=0,elapsed_s=time.monotonic()-start,
        input_hash_guard=all(b.m.drive.sha(b.ROOT/f)==h for f,h in source.items()))
    save()


if __name__=='__main__':main()
