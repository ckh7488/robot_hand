"""Exact-commit replay of the exterior baseline, with no motion re-solve."""
import sys,argparse,subprocess
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import inspect_render as b
n,r,np,ROOT,HERE=b.n,b.r,b.np,b.ROOT,b.HERE
from OCP.OSD import OSD_ThreadPool


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*values):return subprocess.check_output(['git',*values],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1') and not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
    manifest=n.p.read(ROOT/'checkpoints/PACK048.manifest.json');frozen={}
    for key in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(n.p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[key])
    def guard():return all(r.sha(ROOT/f)==z['sha256'] for f,z in manifest['files'].items()) and all(r.sha(ROOT/f)==h for f,h in frozen.items())
    assert guard();raw=n.p.read(HERE/'reviews/baseline.json');state,base,materials=b.metadata();rows=[]
    result=dict(completed=False,target_commit=args.commit,differences=[],mesh_boxes=[],native_STEP_boxes=[],groups=[])
    def save():n.a.save(args.output,result)
    for got,want in zip(base+materials,raw['mesh_parts']):
        assert got['name']==want['name'] and got['mesh']==want['mesh'];points=r.read_stl_points(ROOT/got['mesh']);box=np.array([points.min(axis=0),points.max(axis=0)])
        err=float(np.max(abs(box-want['mesh_bbox_mm'])));ok=err<1e-7
        rows.append(dict(want,mesh_bbox_mm=box));result['mesh_boxes'].append(dict(name=got['name'],bbox_mm=box,maximum_difference_mm=err,matched=ok))
        if not ok:result['differences'].append('mesh_'+got['name'])
    assert len(rows)==len(raw['mesh_parts'])==980
    selections={'hand397':rows[:397],'packaging385':rows[397:782],'forearm_packaging346':[z for z in rows[397:782] if z['body']!='palm'],
        'electrical64':rows[782:846],'arm10':rows[846:856],'PAIR6':rows[856:862],'hardware862':rows[:862],
        'materials118':rows[862:],'whole980':rows,'upstream_free_housing16':[z for z in rows[862:] if z['kind']=='housing' and 3<=z['path_segment']<3+n.NUP]}
    for key,selected in selections.items():
        got=b.summary(selected);want=raw['groups'][key];err=float(np.max(abs(got['bbox_mm']-want['bbox_mm'])));ok=err<1e-7 and got['parts']==want['parts']
        result['groups'].append(dict(id=key,bbox_mm=got['bbox_mm'],maximum_difference_mm=err,matched=ok))
        if not ok:result['differences'].append('group_'+key)
    for z in raw['native_STEP_boxes']:
        box=n.c.m.pack._bbox(n.cq.importers.importStep(str(ROOT/z['step'])).val());err=float(np.max(abs(np.array(box)-z['bbox_mm'])));ok=err<1e-7
        result['native_STEP_boxes'].append(dict(name=z['name'],bbox_mm=box,maximum_difference_mm=err,matched=ok))
        if not ok:result['differences'].append('STEP_box_'+z['name'])
        save()
    result.update(completed=True,passed=not result['differences'] and guard() and not git('status','--porcelain=v1'),
        manifest_files=len(manifest['files']),frozen_files=len(frozen),input_hash_guard=guard(),checkout_clean=not git('status','--porcelain=v1'),
        OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),native_Boolean_groups=0,motion_or_seam_checks_reexecuted=0,
        rendering_reexecuted=False,geometry_optimized=False,independent_audit=False,packaging_or_physical_drive_approved=False,
        inherited_MOTION047_initial_core_passed=False)
    save()


if __name__=='__main__':main()
