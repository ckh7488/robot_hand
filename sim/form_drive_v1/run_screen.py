"""Reproducible conceptual 20-axis screen, retaining missing events and convergence failures."""
from pathlib import Path
import argparse
import copy
import hashlib
import json
import sys
import time as clock
import numpy as np
from model_adapter import load_legacy, build_system, scenario, run_simulation

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'cad/form_drive_v1'))
from build_contract import build

SCENARIOS=['small5','small10','large45','fast90_150ms','upstream_hold','rapid_reversal','independent_distal']

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,default=Path(__file__).parent/'results/drive001')
    p.add_argument('--traces',type=Path,required=True);p.add_argument('--quick',action='store_true')
    args=p.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=True);args.traces.mkdir(parents=True,exist_ok=True)
    ref=Path(__file__).parent/'reference_06b9ab4';legacy=load_legacy(ref)
    inputs=list(build()['source_sha256'])+['cad/form_drive_v1/build_contract.py','sim/form_drive_v1/model_adapter.py','sim/form_drive_v1/run_screen.py']
    inputs += ['sim/form_drive_v1/reference_06b9ab4/'+k for k in legacy['source_sha256']]
    digest=lambda:{p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in inputs}
    manifest=digest();(out/'input_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    cfg=copy.deepcopy(legacy['model'].DEFAULTS)
    (out/'parameters.json').write_text(json.dumps(dict(parameters=cfg,status='Uncalibrated CP006 motor/controller/cable assumptions copied explicitly for comparison; not a tuned or validated real servo.'),indent=2)+'\n')
    jobs=[(1.,s,.000125) for s in (['small5'] if args.quick else SCENARIOS)]
    if not args.quick:
        jobs += [(c,s,.000125) for c in (0.3,0.) for s in ('small5','fast90_150ms')]
        jobs += [(c,s,.0000625) for c in (1.,0.3,0.) for s in ('small5','fast90_150ms')]
    records=[];traces={}
    for coupling,name,dt in jobs:
        label=f'coupling{coupling:g}_{name}_{round(dt*1e6,2):g}us'
        contract=out/f'contract_coupling{coupling:g}.json'
        contract.write_text(json.dumps(build(coupling),indent=2)+'\n')
        start=clock.monotonic()
        try:
            params={**cfg,'dt_s':dt};system=build_system(contract,ref,parameters=params,allow_conceptual=True,repo_root=ROOT)
            t,q,moves=scenario(legacy,name,dt)
            trace=run_simulation(system,t,q);m=legacy['evaluate'].evaluate(trace,moves,system['active'])
        except Exception as exc:
            record=dict(label=label,scenario=name,coupling=coupling,dt_s=dt,elapsed_s=clock.monotonic()-start,
                finite=False,status='execution_failed',error=type(exc).__name__+': '+str(exc))
            (out/(label+'.json')).write_text(json.dumps(record,indent=2)+'\n')
            records.append(record);print(json.dumps(record),flush=True);continue
        finite=all(np.all(np.isfinite(trace[k])) for k in ['q','v','motor_speed','peak_tension'])
        record=dict(label=label,scenario=name,coupling=coupling,dt_s=dt,elapsed_s=clock.monotonic()-start,finite=bool(finite),
                    contract_sha256=system['contract_sha256'],provenance=system['provenance'],metrics=m)
        (out/(label+'.json')).write_text(json.dumps(record,indent=2)+'\n')
        np.savez_compressed(args.traces/(label+'.npz'),time=t,q=trace['q'],v=trace['v'],command=q)
        records.append(record);traces[(coupling,name,dt)]=(t,trace['q'])
        print(json.dumps({k:record[k] for k in ['label','elapsed_s','finite']}|{k:m[k] for k in ['valid_event_max_ms','missing_events','raw_max_error_deg','unsettled_quality_holds']}),flush=True)
    convergence=[]
    for c in (1.,0.3,0.):
        for name in ('small5','fast90_150ms'):
            a=traces.get((c,name,.000125));b=traces.get((c,name,.0000625))
            if a is None or b is None:
                if not args.quick:convergence.append(dict(coupling=c,scenario=name,passed=False,status='missing_or_failed_run'))
                continue
            bq=np.column_stack([np.interp(a[0],b[0],b[1][:,j]) for j in range(20)])
            d=float(np.max(np.abs(np.rad2deg(a[1]-bq))))
            ra=next(r for r in records if r['coupling']==c and r['scenario']==name and r['dt_s']==.000125)
            rb=next(r for r in records if r['coupling']==c and r['scenario']==name and r['dt_s']==.0000625)
            ma,mb=ra['metrics'],rb['metrics']
            ea,eb=ma['events'],mb['events']
            keys=lambda rows:[(r['move'],r['joint'],r['fraction']) for r in rows]
            identity=keys(ea)==keys(eb)
            same=identity and all(x['classification']==y['classification'] for x,y in zip(ea,eb))
            ha,hb=ma['hold_quality'],mb['hold_quality']
            holds_identical=[(r['move'],r['joint'],r['status']) for r in ha]==[(r['move'],r['joint'],r['status']) for r in hb]
            delays=[abs(x['lag_ms']-y['lag_ms']) for x,y in zip(ea,eb) if x['lag_ms'] is not None and y['lag_ms'] is not None]
            lag_delta=max(delays,default=None)
            angle_limit=.02 if name=='small5' else 1.
            passed=bool(d<=angle_limit and same and holds_identical and lag_delta is not None and lag_delta<=2.)
            convergence.append(dict(coupling=c,scenario=name,max_angle_difference_deg=d,max_event_lag_difference_ms=lag_delta,
                angle_limit_deg=angle_limit,event_identity_identical=identity,event_classifications_identical=same,
                hold_quality_classifications_identical=holds_identical,passed=passed))
    unchanged=digest()==manifest
    summary=dict(status='conceptual_screen_only' if unchanged else 'invalid_inputs_changed_during_run',inputs_unchanged=unchanged,hardware_100ms_proven=False,
        joint_count=20,motor_paths_assumed=20,passed_convergence=sum(x['passed'] for x in convergence),convergence=convergence,
        convergence_criteria='Provisional numerical check: max same-time q difference <=0.02 deg for small5, <=1 deg for fast90; every event identity/class and hold status unchanged; corresponding event lag difference <=2 ms. Not physical acceptance.',
        cases=[dict(label=r['label'],scenario=r['scenario'],coupling=r['coupling'],dt_s=r['dt_s'],finite=r['finite'],
            **({k:r['metrics'][k] for k in ['event_count','valid_event_max_ms','missing_events','anticipatory_events','raw_max_error_deg','unsettled_quality_holds','simulated_100ms_event_budget_met','max_tendon_tension_N']} if 'metrics' in r else dict(status=r['status'],error=r['error']))) for r in records],
        limitations=build()['limitations'],runtime=dict(python=sys.version,numpy=np.__version__,mujoco=legacy['model'].mujoco.__version__),
        runner_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary),flush=True)
    if not unchanged or not all(r['finite'] for r in records):raise SystemExit(2)

if __name__=='__main__':main()
