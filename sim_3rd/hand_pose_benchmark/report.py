"""Standalone visual review and aggregate counts from actual saved inference."""
import argparse
import base64
import collections
import csv
import io
import json
import pathlib
import sqlite3
import zlib
from datetime import datetime,timezone

import numpy as np
from PIL import Image

from data import Samples
from metrics import EDGES

ROOT=pathlib.Path(r'E:\hand_recog\vision-results\20261026')
NAMES=['mediapipe','wilor','hamer']


def summarize(rows):
    ok=[r for r in rows if r['status']=='complete'];matches=[m for r in ok for m in r['metrics'].get('matches',[])]
    latency=[r['elapsed_ms'] for r in ok if r['elapsed_ms'] is not None][1:]
    eligible=[r for r in ok if r['metrics'].get('detected_hands',0)>0]
    mesh=[r for r in eligible if r['metrics'].get('mesh_crossing_hands') is not None]
    both=[r for r in ok if r['metrics'].get('between_hands_crossing_pairs') is not None]
    regions=[r for r in mesh if 'different_finger_hands' in r['metrics']]
    return dict(images=len(rows),errors=len(rows)-len(ok),gt_hands=sum(r['metrics'].get('gt_hands',0) for r in ok),
                matched_hands=len(matches),missed_hands=sum(r['metrics'].get('missed_hands',0) for r in ok),
                extra_hands=sum(r['metrics'].get('extra_hands',0) for r in ok),
                mpjpe_mm=float(np.mean([m['mpjpe_mm'] for m in matches])) if matches else None,
                bend_mae_deg=float(np.mean([m['joint_angle_mae_deg'] for m in matches])) if matches else None,
                bad_bend_hands=sum(m['bad_bends_30deg']>0 for m in matches),
                different_finger_images=sum(bool(r['metrics']['different_finger_hands']) for r in regions),region_eligible_images=len(regions),
                mesh_crossing_images=sum(bool(r['metrics']['mesh_crossing_hands']) for r in mesh),mesh_eligible_images=len(mesh),
                between_crossing_images=sum(bool(r['metrics']['between_hands_crossing_pairs']) for r in both),between_eligible_images=len(both),
                latency_p50_ms=float(np.median(latency)) if latency else None,
                latency_p95_ms=float(np.percentile(latency,95)) if latency else None)


def build(output=ROOT,case_limit=120,phase='eval'):
    samples=Samples(phase);index={r['id']:i for i,r in enumerate(samples.rows)}
    by_model={};dbs={};summary={};dataset_summary={};region_maps={}
    for name in NAMES:
        path=output/f'{name}_{phase}.sqlite'
        if not path.exists():continue
        region_path=output/f'{name}_{phase}_regions.sqlite'
        region_maps[name]={}
        if region_path.exists():
            rd=sqlite3.connect(f'file:{region_path.as_posix()}?mode=ro',uri=True)
            region_maps[name]={sid:json.loads(v) for sid,v in rd.execute('SELECT id,data FROM regions')};rd.close()
        dbs[name]=sqlite3.connect(f'file:{path.as_posix()}?mode=ro',uri=True)
        rows=[]
        for sid,ds,split,status,ms,met in dbs[name].execute('SELECT id,dataset,split,status,elapsed_ms,metrics FROM results'):
            met=json.loads(met)
            if ds=='ReInterHand sample':met['gt_evaluable']=False;met['extra_hands']=0
            if sid in region_maps[name]:
                met['different_finger_hands']=sum(h['different_finger_pairs']>0 for h in region_maps[name][sid])
                met['different_finger_pairs']=sum(h['different_finger_pairs'] for h in region_maps[name][sid])
            rows.append(dict(id=sid,dataset=ds,split=split,status=status,elapsed_ms=ms,metrics=met))
        by_model[name]={r['id']:r for r in rows};summary[name]=summarize(rows)
        dataset_summary[name]={ds:summarize([r for r in rows if r['dataset']==ds]) for ds in sorted({r['dataset'] for r in rows})}
    common=set(index)
    for name in NAMES:common &= set(by_model.get(name,{}))
    paired={name:[] for name in NAMES}
    for sid in sorted(common):
        maps={name:{m['gt']:m for m in by_model[name][sid]['metrics'].get('matches',[])} for name in NAMES}
        shared=set.intersection(*(set(v) for v in maps.values()))
        for g in shared:
            for name in NAMES:paired[name].append(maps[name][g]['mpjpe_mm'])
    paired_summary={name:dict(matched_hands=len(v),mpjpe_mm=float(np.mean(v)) if v else None) for name,v in paired.items()}
    selected=[]
    def add(ids):
        for sid in ids:
            if sid in common and sid not in selected:selected.append(sid)
    # The gallery is intentionally selected for diagnosis; aggregate statistics use all rows.
    add(sid for sid in sorted(common) if 'inspection/' in sid)
    for name in ['wilor','hamer']:
        values=[r for sid,r in by_model.get(name,{}).items() if sid in common]
        add(r['id'] for r in sorted(values,key=lambda r:r['metrics'].get('different_finger_pairs',0),reverse=True)[:8])
        add(r['id'] for r in sorted(values,key=lambda r:r['metrics'].get('between_hands_crossing_pairs') or 0,reverse=True)[:8])
    for name in NAMES:
        values=[r for sid,r in by_model.get(name,{}).items() if sid in common]
        def worst(r):return max([m['mpjpe_mm'] for m in r['metrics'].get('matches',[])],default=0)
        add(r['id'] for r in sorted(values,key=worst,reverse=True)[:18])
        add(r['id'] for r in sorted(values,key=lambda r:r['metrics'].get('missed_hands',0),reverse=True)[:6])
        add(r['id'] for r in sorted(values,key=lambda r:r['metrics'].get('mesh_crossing_hands') or 0,reverse=True)[:8])
        add(r['id'] for r in sorted(values,key=lambda r:r['metrics'].get('between_hands_crossing_pairs') or 0,reverse=True)[:8])
    rng=np.random.default_rng(26)
    rest=sorted(common);rng.shuffle(rest);add(rest[:20])
    selected=selected[:case_limit]
    cases=[]
    for sid in selected:
        row,image,gt=samples.read(index[sid]);canvas=Image.fromarray(image[:,:,::-1]);canvas.thumbnail((640,640))
        stream=io.BytesIO();canvas.save(stream,format='JPEG',quality=86)
        c=dict(id=sid,dataset=row['dataset'],split=row['split'],image='data:image/jpeg;base64,'+base64.b64encode(stream.getvalue()).decode(),
               image_size=[image.shape[1],image.shape[0]],gt=[{k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in h.items()} for h in gt],models={})
        for name in NAMES:
            packed=dbs[name].execute('SELECT payload FROM results WHERE id=?',(sid,)).fetchone()
            d=json.loads(zlib.decompress(packed[0]));r=by_model[name][sid]
            for h,region in zip(d.get('hands',[]),region_maps[name].get(sid,[])):h['geometry'].update(region)
            c['models'][name]=dict(hands=d.get('hands',[]),metrics=r['metrics'],status=r['status'],ms=r['elapsed_ms'])
        cases.append(c)
    faces={name:np.load(output/f'{name}_faces.npy').tolist() for name in ['wilor','hamer'] if (output/f'{name}_faces.npy').exists()}
    result=dict(updated_at=datetime.now(timezone.utc).isoformat(),phase=phase,total_current_inputs=len(samples),common_completed=len(common),paired_summary=paired_summary,
                inventory=samples.inventory,summary=summary,dataset_summary=dataset_summary,cases=cases,faces=faces,edges=EDGES,
                broader_scope='Server raw has 12 folders. Full server-only archives have not been inferred in this run. FreiHAND evaluation + RHD evaluation + 14 explicitly separate inspection images are the current complete-input pass.',
                pipeline='MediaPipe HandLandmarker; WiLoR-mini and original HaMeR both use the WiLoR YOLO hand detector (conf 0.3), with no GT or MediaPipe boxes. Float32.',
                known_limits=['No claim of independent unseen generalization: pretrained-model training exposure may overlap these public datasets.',
                              'Root-relative errors use metric output directly, with no fitted scale or rotation.',
                              'Strict nonadjacent triangle surface intersections are geometric flags, not measured tissue penetration volume or depth.',
                              'MediaPipe has no mesh or common-camera hand translation; its mesh/inter-hand intersection metric is unavailable.',
                              'Reported model latency excludes image decode, metric calculation, database writes, rendering and camera; parallel workers share the host.'])
    for db in dbs.values():db.close()
    dest=output/('report' if phase=='eval' else 'report_train');dest.mkdir(exist_ok=True)
    template=pathlib.Path(__file__).with_name('viewer.html').read_text(encoding='utf8')
    payload=json.dumps(result,ensure_ascii=False,separators=(',',':'),allow_nan=False).replace('</','<\\/')
    html=template.replace('__REPORT_DATA__',payload)
    tmp=dest/'hand_pose_review.tmp';tmp.write_text(html,encoding='utf8');tmp.replace(dest/'hand_pose_review.html')
    aggregate={k:v for k,v in result.items() if k not in ['cases','faces','edges']}
    (dest/'summary.json').write_text(json.dumps(aggregate,ensure_ascii=False,indent=2),encoding='utf8')
    with (dest/'per_image_metrics.csv').open('w',newline='',encoding='utf-8-sig') as f:
        writer=csv.writer(f);writer.writerow(['model','id','dataset','status','gt_hands','matched_hands','missed_hands','MPJPE_mm_matched','bend_MAE_deg','bad_bend_hands_30deg','self_crossing_hands','different_finger_hands','between_hands_pairs','inference_ms'])
        for name,rr in by_model.items():
            for sid,r in rr.items():
                m=r['metrics'];a=m.get('matches',[])
                writer.writerow([name,sid,r['dataset'],r['status'],m.get('gt_hands'),m.get('matched_hands'),m.get('missed_hands'),np.mean([x['mpjpe_mm'] for x in a]) if a else '',np.mean([x['joint_angle_mae_deg'] for x in a]) if a else '',sum(x['bad_bends_30deg']>0 for x in a),m.get('mesh_crossing_hands'),m.get('different_finger_hands'),m.get('between_hands_crossing_pairs'),r['elapsed_ms']])
    print(json.dumps({'report':str(dest/'hand_pose_review.html'),'common':len(common),'cases':len(cases),'summary':summary}),flush=True)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=pathlib.Path,default=ROOT);parser.add_argument('--cases',type=int,default=120)
    parser.add_argument('--phase',default='eval',choices=['eval','train'])
    args=parser.parse_args();build(args.output,args.cases,args.phase)
