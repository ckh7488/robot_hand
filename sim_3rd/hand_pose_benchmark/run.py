"""Resumable full-coverage inference. One sqlite file per model/phase."""
import argparse
import hashlib
import io
import json
import pathlib
import sqlite3
import time
import traceback
import zlib
from datetime import datetime,timezone

import numpy as np

from data import Samples
from metrics import match_and_measure,geometric_flags,inter_mesh_crossings

DEFAULT_OUT=pathlib.Path(r'E:\hand_recog\vision-results\20261026')


def jsonable(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    if isinstance(value,dict):return {k:jsonable(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [jsonable(v) for v in value]
    return value


def write_progress(path,value):
    """A Windows reader briefly holding the progress file must not stop inference."""
    temp=path.with_suffix('.tmp');temp.write_text(json.dumps(value),encoding='utf8')
    for attempt in range(10):
        try:temp.replace(path);return
        except PermissionError:time.sleep(.05)
    print('Progress snapshot busy; inference results remain committed.',flush=True)


def main():
    p=argparse.ArgumentParser();p.add_argument('--model',choices=['mediapipe','wilor','hamer'],required=True)
    p.add_argument('--phase',choices=['eval','train'],default='eval');p.add_argument('--device',type=int,default=0)
    p.add_argument('--limit',type=int,default=0);p.add_argument('--output',type=pathlib.Path,default=DEFAULT_OUT)
    p.add_argument('--worker',type=int,default=0);p.add_argument('--workers',type=int,default=1)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    if not 0<=args.worker<args.workers:raise ValueError('worker must be in [0, workers)')
    suffix=f'_worker{args.worker}' if args.workers>1 else ''
    db=sqlite3.connect(args.output/f'{args.model}_{args.phase}.sqlite',timeout=60)
    db.execute('PRAGMA journal_mode=WAL')
    db.execute('CREATE TABLE IF NOT EXISTS results (id TEXT PRIMARY KEY, dataset TEXT, split TEXT, status TEXT, elapsed_ms REAL, metrics TEXT, payload BLOB, completed_at TEXT)')
    done={r[0] for r in db.execute('SELECT id FROM results')}
    samples=Samples(args.phase)
    (args.output/f'inventory_{args.phase}.json').write_text(json.dumps(samples.inventory,indent=2),encoding='utf8')
    from adapters import FACTORIES
    model=FACTORIES[args.model](args.device)
    if model.faces is not None and not (args.output/f'{args.model}_faces.npy').exists():
        np.save(args.output/f'{args.model}_faces.npy',model.faces)
    meta=dict(model=args.model,phase=args.phase,total_inputs=len(samples),device=args.device,
              detector='MediaPipe palm detector' if args.model=='mediapipe' else 'WiLoR YOLO detector.pt, conf=0.3',
              precision='float32',mode='full_image_no_gt_input',checkpoint_keys=getattr(model,'checkpoint_keys',None),
              metric_contract='Root-relative, no rotation or scale fitting; strict surface crossings exclude adjacent/tangent/coplanar pairs.',
              warmup_excluded_from_latency_first_n=1)
    meta.update(worker=args.worker,workers=args.workers)
    (args.output/f'{args.model}_{args.phase}{suffix}_metadata.json').write_text(json.dumps(meta,indent=2),encoding='utf8')
    count=0;start=time.monotonic();last=time.monotonic()
    for order_index,i in enumerate(samples.order()):
        if order_index%args.workers!=args.worker:continue
        row=samples.rows[i]
        if row['id'] in done:continue
        if args.limit and count>=args.limit:break
        status='complete';elapsed=None;metrics={};payload={}
        try:
            row,image,gt=samples.read(i)
            hands,elapsed=model.predict(image)
            for h in hands:
                if h['xyz'].shape!=(21,3) or not np.isfinite(h['xyz']).all():raise ValueError('Invalid 3D points')
                h['geometry']=geometric_flags(h,model.faces)
            metrics=match_and_measure(hands,gt)
            metrics['gt_evaluable']=row['dataset']!='ReInterHand sample'
            if not metrics['gt_evaluable']:metrics['extra_hands']=0
            metrics['mesh_crossing_hands']=sum(bool(h['geometry']['mesh_crossing_pairs']) for h in hands) if model.faces is not None else None
            metrics['between_hands_crossing_pairs']=None
            if model.faces is not None and len(hands)==2:
                a,b=hands
                cross,ia,ib=inter_mesh_crossings(np.ascontiguousarray(a['vertices']+a['camera_t'],np.float64),
                                               np.ascontiguousarray(b['vertices']+b['camera_t'],np.float64),
                                               np.ascontiguousarray(model.faces,np.int64))
                metrics['between_hands_crossing_pairs']=int(cross)
                hands[0]['geometry']['between_hands_faces']=ia.tolist();hands[1]['geometry']['between_hands_faces']=ib.tolist()
            payload=dict(row=row,gt=gt,hands=hands,image_size=[image.shape[1],image.shape[0]])
        except Exception as e:
            status='error';metrics=dict(error_type=type(e).__name__,message=str(e))
            print('ERROR',row['id'],type(e).__name__,str(e),flush=True)
            # A systematic setup error must not turn the full dataset into a fast failure run.
            if count<3:traceback.print_exc();raise
        packed=zlib.compress(json.dumps(jsonable(payload),separators=(',',':'),allow_nan=False).encode(),3)
        db.execute('INSERT OR REPLACE INTO results VALUES (?,?,?,?,?,?,?,?)',(row['id'],row['dataset'],row['split'],status,elapsed,json.dumps(jsonable(metrics)),packed,datetime.now(timezone.utc).isoformat()))
        db.commit()
        count+=1
        if count%10==0 or time.monotonic()-last>10:
            db.commit();last=time.monotonic()
            progress=dict(model=args.model,phase=args.phase,worker=args.worker,processed=db.execute('SELECT count(*) FROM results').fetchone()[0],total=len(samples),new_this_run=count,
                          elapsed_s=round(time.monotonic()-start,2),state='running',updated_at=datetime.now(timezone.utc).isoformat())
            write_progress(args.output/f'{args.model}_{args.phase}{suffix}_progress.json',progress)
            print(json.dumps(progress),flush=True)
    db.commit()
    total=db.execute('SELECT count(*) FROM results').fetchone()[0]
    final=dict(model=args.model,phase=args.phase,processed=total,total=len(samples),state='complete' if total==len(samples) else 'partial',updated_at=datetime.now(timezone.utc).isoformat())
    write_progress(args.output/f'{args.model}_{args.phase}{suffix}_progress.json',final)
    print(json.dumps(final),flush=True)
    db.close()


if __name__=='__main__':main()
