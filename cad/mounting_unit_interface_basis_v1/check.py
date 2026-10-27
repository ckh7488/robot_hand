"""New entry-space relations and exact vendor surface/datum evidence only."""
import importlib.util
import sys
import time
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm062_model',Path(__file__).with_name('model.py'))
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
n,np,d,ROOT,HERE=a.n,a.np,a.d,a.ROOT,a.HERE
from OCP.OSD import OSD_ThreadPool


def main():
    out=HERE/'reviews/check.json';assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    started=time.monotonic();rows,membership,source=a.context()
    unit,entry,contract=a.mounting_unit(rows)
    vendor,interface=a.vendor_interface()
    for rel in [a.SOURCE,'cad/modular_roots_v9/sources/AK60.pdf','cad/modular_roots_v9/sources/vendor_sewing.json',
                'cad/modular_roots_v9/build_candidate.py','cad/central_reaction_frame_v1/reviews/followup.json']:
        source[rel]=n.m.drive.sha(ROOT/rel)
    for f in [Path(__file__),HERE/'model.py',HERE/'sources/receipts.json']:
        source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    # Export references separately so none can be confused with new hardware.
    clean=lambda rs:[{k:v for k,v in z.items() if k not in ['step','mesh','sha256_step','sha256_stl']} for z in rs]
    raw=dict(completed=False,source_sha256=source,unit_contract=contract,vendor_interface=interface,
        mounting_reference_scene=n.a.a.prior.export(clean(unit),HERE/'generated/mounting_reference'),
        entry_reference_scene=n.a.a.prior.export(entry,HERE/'generated/entry_reference'),
        vendor_reference_scene=n.a.a.prior.export([vendor],HERE/'generated/vendor_reference'),native={},
        current862_replaced=False,proposal871_adopted=False,new_installed_hardware=0,
        old_finished_native_groups_reexecuted=0,old_preservation_queries_reexecuted=0,
        vendor_open_surface_native_volume_queries=0,OCC_pool_threads=1,reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6)
    def save():n.a.save(out,raw)
    def run(label,left,right):
        t=n.c.material(left,right);raw['native'][label]=t;save();print(label,t['pairs_in_scope'],t['exact_pairs'],len(t['overlaps']),flush=True)
    save()
    run('new_axial_entry4__all_existing871',entry,rows)
    protection=n.c.load(n.p.read(ROOT/'cad/rotation_reserved_support_v1/generated/protection/scene.json')['parts'],source)
    run('new_axial_entry4__existing_rotation_bounds9',entry,protection[:9])
    total=d.k.flat(list(raw['native'].values()))
    raw.update(completed=True,native_groups=len(raw['native']),pairs_in_scope=total['pairs_in_scope'],AABB_native_pairs=total['exact_pairs'],
        overlap_records=len(total['overlaps']),operation_failures=len(total['operation_failures']),
        raw_reference_entry_clear=not total['overlaps'] and not total['operation_failures'],whole_design_raw_clear=False,
        input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()),elapsed_s=time.monotonic()-started)
    save();print('complete',raw['native_groups'],raw['overlap_records'],raw['operation_failures'],'holes',len(interface['output_M3_cylinder_axes']),flush=True)


if __name__=='__main__':main()
