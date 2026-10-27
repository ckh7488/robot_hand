"""Exact-SHA bounded replay of the new contact and fastener evidence."""
import sys,argparse,subprocess,ast
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import basis as b
import support as s
from OCP.OSD import OSD_ThreadPool


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*args):return subprocess.check_output(['git',*args],cwd=b.ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1') and not args.output.exists() and not args.output.resolve().is_relative_to(b.ROOT)
    manifest=b.p.read(b.ROOT/'checkpoints/FIX044.manifest.json');frozen={}
    for key in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(b.p.read(b.ROOT/'AUDIT_EVIDENCE_INDEX.json')[key])
    def guard():return all(b.m.drive.sha(b.ROOT/f)==z['sha256'] for f,z in manifest['files'].items()) and all(b.m.drive.sha(b.ROOT/f)==h for f,h in frozen.items())
    assert guard();raw=b.p.read(b.HERE/'reviews/check.json');case,source,rows=b.context();lookup={z['name']:z for z in rows}
    recovery=b.p.read(b.HERE/'reviews/metadata_recovery.json')
    def fn(path):return {z.name:ast.dump(z,include_attributes=False) for z in ast.parse(path.read_text(encoding='utf-8-sig')).body if isinstance(z,ast.FunctionDef) and z.name!='export'}
    original=b.HERE/'history/basis_initial.py.txt'
    assert fn(original)==fn(b.HERE/'basis.py') and b.m.drive.sha(original)==raw['source_sha256']['cad/tail_fixation_basis_v1/basis.py']
    assert all(b.m.drive.sha(b.ROOT/f)==h for f,h in recovery['geometry_sha256_preserved'].items())
    result=dict(completed=False,target_commit=args.commit,differences=[],native_samples=[],contacts=[],geometry_comparisons=[])
    def save():b.a.save(args.output,result)
    def compare(label,got,want):
        aa={(z['lead'],z['part']):z['material_intersection_mm3'] for z in got['overlaps']};bb={(z['lead'],z['part']):z['material_intersection_mm3'] for z in want['overlaps']}
        err=max([abs(aa[k]-bb[k]) for k in aa.keys()&bb.keys()]+[0.])
        ok=aa.keys()==bb.keys() and err<1e-5 and got['pairs_in_scope']==want['pairs_in_scope'] and got['operation_failures']==want['operation_failures']
        result['native_samples'].append(dict(id=label,native=got,matched=bool(ok),maximum_basic_volume_difference=err))
        if not ok:result['differences'].append(label)
        print(label,ok,flush=True);save()
    metric=b.e.measure
    supplement=b.p.read(b.HERE/'reviews/support.json')
    probe_meta=b.p.read(b.ROOT/raw['contact_material_scene'])['parts']+b.p.read(b.ROOT/supplement['scene'])['parts'];saved_probes=b.c.load(probe_meta)
    for spec,want,saved in zip(b.specifications(case)+s.specifications(case),raw['contacts']+supplement['contacts'],saved_probes):
        got,probe=b.contact(spec,lookup);compare('contact_'+got['id'],got['native'],want['native'])
        distances=[abs(x['distance_mm']-y['distance_mm']) for x,y in zip(got['surfaces'],want['surfaces'])]
        # Closest-point coordinates can be nonunique along a tangent contact.
        states=all(x['normal_samples']==y['normal_samples'] for x,y in zip(got['surfaces'],want['surfaces']))
        # Rebuild the saved boundary points and compare classification, not the arbitrary DSS witness.
        ok=max(distances)<1e-5 and states and abs(got['tail_axis_dot_contact_normal']-want['tail_axis_dot_contact_normal'])<1e-10
        result['contacts'].append(dict(id=got['id'],native_distances=got['surfaces'],matched=bool(ok),maximum_distance_difference=max(distances)))
        if not ok:result['differences'].append('contact_geometry_'+got['id'])
        q=dict(name=probe['name'],**metric.match(metric.metrics(probe['shape']),metric.metrics(saved['shape'])))
        result['geometry_comparisons'].append(q)
        if not q['matched']:result['differences'].append('probe_metric_'+probe['name'])
        save()
    for plan,want in zip(b.plans(case),raw['paths']):
        assert plan['moving']==want['moving'];ob=[lookup[name] for name in want['obstacle_names']]
        for index in sorted({0,len(want['samples'])-1}):
            z=want['samples'][index];saved=b.c.load(b.p.read(b.ROOT/z['scene'])['parts'])[0]
            compare(plan['id']+'_'+str(index),b.c.material([saved],ob),z['native'])
            if index==0:
                fresh=b.moved(lookup[plan['moving']],plan,z['outward_distance_mm'])
                q=dict(name=plan['moving'],**metric.match(metric.metrics(fresh['shape']),metric.metrics(saved['shape'])))
                result['geometry_comparisons'].append(q)
                if not q['matched']:result['differences'].append('fastener_metric_'+plan['moving'])
            save()
    result.update(completed=True,passed=not result['differences'] and guard() and not git('status','--porcelain=v1'),
        input_hash_guard=guard(),manifest_files=len(manifest['files']),frozen_files=len(frozen),
        checkout_clean=not git('status','--porcelain=v1'),OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),
        independent_audit=False,inherited_CORD043_initial_core_passed=False,actual_fixation_or_hand_motion_approved=False,
        original_non_export_function_AST_preserved=True,metadata_recovery_geometry_hashes=len(recovery['geometry_sha256_preserved']))
    save()


if __name__=='__main__':main()
