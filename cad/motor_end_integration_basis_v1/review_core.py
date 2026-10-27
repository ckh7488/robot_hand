"""Fresh exact-commit replay of12 bounded diagnostics and11 evidence shapes."""
import argparse
import importlib.util
import subprocess
import sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm058_review_survey',Path(__file__).with_name('survey.py'))
s=importlib.util.module_from_spec(spec)
spec.loader.exec_module(s)
n,np,ROOT,HERE=s.n,s.np,s.ROOT,s.HERE
f=n.m.drive.module('_arm058_review_follow',HERE/'carrier_followup.py')
from OCP.OSD import OSD_ThreadPool


def match(left,right):
    a,b=s.metrics(left),s.metrics(right)
    count=(a['valid'],a['solids'])==(b['valid'],b['solids']) and a['valid']
    # Prior single-solid matcher compares the same adaptive face/edge/vertex metrics.
    # Counts are checked above; a diagnostic common can contain two separate solids.
    aa,bb=dict(a,solids=1),dict(b,solids=1)
    got=s.measure.match(aa,bb)
    return dict(got,matched=bool(got['matched'] and count),actual_solid_counts=[a['solids'],b['solids']])


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--commit',required=True)
    ap.add_argument('--output',required=True,type=Path)
    args=ap.parse_args()
    def git(*xs):
        return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1')
    assert not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
    manifest=n.p.read(ROOT/'checkpoints/ARM058.manifest.json')
    frozen={}
    for k in ['geometry_sha256','code_sha256','report_sha256']:
        frozen.update(n.p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[k])
    def guard():
        return all(n.m.drive.sha(ROOT/k)==v['sha256'] for k,v in manifest['files'].items()) and all(n.m.drive.sha(ROOT/k)==v for k,v in frozen.items())
    assert guard() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    result=dict(completed=False,target_commit=args.commit,differences=[],native_samples=[],geometry_comparisons=[],
        manifest_files=len(manifest['files']),frozen_files=len(frozen),OCC_pool_threads=1,
        current862_installation_replaced=False,same_agent_not_independent_audit=True,
        old_ARM051_through_ARM057_native_reexecuted=False,whole_drive_or_exterior_approved=False)
    def save(): n.a.save(args.output,result)
    for label,model in [('survey',s),('carrier_followup',f)]:
        got,rows=model.calculate()
        want=n.p.read(HERE/('reviews/'+label+'.json'))
        for x,y in zip(got['native_queries'],want['native_queries']):
            err=abs(x['volume_mm3']-y['volume_mm3'])
            bbox=(x['bbox_mm'] is None and y['bbox_mm'] is None) or (x['bbox_mm'] is not None and y['bbox_mm'] is not None and np.max(abs(np.array(x['bbox_mm'])-y['bbox_mm']))<1e-5)
            ok=x['id']==y['id'] and x['operation']==y['operation'] and x['solids']==y['solids'] and err<1e-5 and bbox
            result['native_samples'].append(dict(id=x['id'],native=x,matched=bool(ok),volume_difference_mm3=err))
            if not ok: result['differences'].append(x['id'])
        saved=n.c.load(n.p.read(ROOT/want['scene'])['parts'])
        assert len(saved)==len(rows)
        for x,y in zip(rows,saved):
            assert x['name']==y['name']
            m=dict(name=x['name'],**match(x['shape'],y['shape']))
            result['geometry_comparisons'].append(m)
            if not m['matched']: result['differences'].append('geometry_'+x['name'])
        if label=='survey':
            ok=np.max(abs(np.array(got['electrical_frame_proposal'])-want['electrical_frame_proposal']))<1e-10
            ok=ok and got['protected_hardware_names']==want['protected_hardware_names'] and got['M3_nominal_stack']==want['M3_nominal_stack']
            for x,y in zip(got['electrical_paths_preserved'],want['electrical_paths_preserved']):
                ok=ok and x['name']==y['name'] and all(abs(x[k]-y[k])<1e-10 for k in ['OD_mm','centreline_radius_mm','centreline_length_mm']) and np.max(abs(np.array(x['end_world_mm'])-y['end_world_mm']))<1e-10
            result['electrical_contract_matched']=bool(ok)
        else:
            x,y=got['identified_spoke'],want['identified_spoke']
            ok=all(x[k]==y[k] for k in x if k!='M3_axis_world_mm') and np.max(abs(np.array(x['M3_axis_world_mm'])-y['M3_axis_world_mm']))<1e-10
            result['neighbor_spoke_identification_matched']=bool(ok)
        if not ok: result['differences'].append(label+'_contract')
        save()
    result.update(completed=True,native_queries=len(result['native_samples']),native_matches=sum(z['matched'] for z in result['native_samples']),
        evidence_shapes=len(result['geometry_comparisons']),geometry_matches=sum(z['matched'] for z in result['geometry_comparisons']),
        input_hash_guard=guard(),checkout_clean=not git('status','--porcelain=v1'),
        replay_scope='12new focused Boolean diagnostics,11 source/STEP evidence metrics at adaptive1e-10/absolute1e-5,electrical frame/threelead contracts/protected names/M3 source intervals and neighboring middle_dip spoke/M3 identity. No ARM05736/39/seam/angles/6DSS/2pads reruns,no whole862 motion,source electrical hardware regeneration,tool/service/force/preload/manufacture/exterior or all trim/surface Boolean equivalence approval.')
    result['initial_core_passed']=not result['differences'] and result['input_hash_guard'] and result['checkout_clean']
    save()
    print('core',result['initial_core_passed'],'native',result['native_matches'],'geometry',result['geometry_matches'],flush=True)


if __name__=='__main__': main()
