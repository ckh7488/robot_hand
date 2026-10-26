"""Exact-commit same-CAD replay of posture corrections and retained failures."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True


def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    if a.output.exists() or a.output.resolve().is_relative_to(ROOT):raise ValueError('New external output required')
    def git(*args):return subprocess.check_output(['git',*args],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    if git('rev-parse','HEAD')!=a.commit or git('status','--porcelain=v1'):raise ValueError('Wrong/dirty target commit')
    manifest=read(ROOT/'checkpoints/POSTURE005.manifest.json')
    def guard():return all(sha(ROOT/p)==row['sha256'] for p,row in manifest['files'].items())
    if not guard():raise ValueError('Changed manifest closure')
    frozen_index=read(ROOT/'AUDIT_EVIDENCE_INDEX.json');frozen={}
    for key in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(frozen_index[key])
    if not all(sha(ROOT/p)==h for p,h in frozen.items()):raise ValueError('Changed frozen evidence')
    spec=importlib.util.spec_from_file_location('_posture_replay',HERE/'audit.py');audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit);p=audit.prior
    native,v,old,scene,parts=p.load();engine=native.PairScreen(v,parts,scene['hand'],old['design'],0.,True)
    source=read(HERE/'reviews/audit.json')
    if not source['completed'] or not source['input_hash_guard']:raise ValueError('Incomplete source')
    byid={c['id']:c for c in source['cases']};wanted={'before_cross_middle_18','before_pinch_index','cross_front_index_0','near_tip_0'}
    wanted.update(s['case'] for s in source['selected']);probes=[]
    for ident in sorted(wanted):
        if ident in byid:probes.append(dict(id=ident,expected=byid[ident]['native'],case=byid[ident]))
    for path in source['paths']:
        samples=path.get('forward_observations',[])
        if not samples:continue
        if path['status']=='sampled_approach_release_clear':
            chosen={0,len(samples)//4,len(samples)//2,3*len(samples)//4,len(samples)-1}
        else:chosen={len(samples)-1}
        for i in sorted(chosen):probes.append(dict(id=path['id']+'@'+str(i),expected=samples[i]['native']))
    differences=[];results=[];metrics=[]
    def signature(rows):return {tuple(sorted([r['a'],r['b']])):r['material_intersection_mm3'] for r in rows}
    body={r['name']:r['body'] for r in scene['hand']}
    for probe in probes:
        q=probe['expected']['angles_deg'];got=p.compact(engine,q);before=signature(probe['expected']['collision_details']);after=signature(got['collision_details'])
        if before.keys()!=after.keys() or len(probe['expected']['operation_failures'])!=len(got['operation_failures']):differences.append(dict(id=probe['id'],reason='Pair or uncertainty set changed'))
        for pair in before.keys() & after.keys():
            if abs(before[pair]-after[pair])>1e-5:differences.append(dict(id=probe['id'],pair=pair,expected=before[pair],actual=after[pair]))
        c=probe.get('case',{})
        if 'native_tip_surface_gap_mm' in c:
            frames=v.frames(old['design'],q);names=['index_segment3_rounded_tip_cap','thumb_segment3_rounded_tip_cap']
            shapes=[v.relocate(parts[n],frames[body[n]],engine.neutral[body[n]]) for n in names];gap=float(shapes[0].distance(shapes[1]))
            if abs(gap-c['native_tip_surface_gap_mm'])>1e-5:differences.append(dict(id=probe['id'],reason='Tip gap changed'))
            metrics.append(dict(id=probe['id'],tip_surface_gap_mm=gap))
        if 'crossings' in c:
            rerun=p.cross_surfaces(v,old['design'],parts,scene,q,[{k:r[k] for k in ['index_segment','middle_segment','xy_mm','signed_depth_mm']} for r in c['crossings']])
            if len(rerun)!=len(c['crossings']):differences.append(dict(id=probe['id'],reason='Crossing count changed'))
            for actual,expected in zip(rerun,c['crossings']):
                if abs(actual['segment_surface_distance_mm']-expected['segment_surface_distance_mm'])>1e-5:differences.append(dict(id=probe['id'],reason='Crossing gap changed'))
            metrics.append(dict(id=probe['id'],crossings=rerun))
        results.append(dict(id=probe['id'],native=got));print(json.dumps(dict(replayed=probe['id'],collisions=got['collision_count'])),flush=True)
    clean=not git('status','--porcelain=v1');last_guard=guard()
    result=dict(target_commit=a.commit,passed=not differences and clean and last_guard,manifest_files=len(manifest['files']),frozen_files=len(frozen),
        input_hash_guard=last_guard,clean_checkout=clean,core_probes=results,metric_replays=metrics,differences=differences,
        scope='Selected full364part successful/failed endpoints, five points per clear forward path, last tested point of failed paths, selected exact surface gaps. Not all search seeds, full paths, native root map, old object task corpus, rendering, continuous or physical testing.',
        independent_audit=False,whole_hand_or_manufacturing_approved=False)
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,separators=(',',':'),allow_nan=False)+'\n',encoding='utf-8')
    if not result['passed']:raise ValueError('Posture reproduction changed')


if __name__=='__main__':main()
