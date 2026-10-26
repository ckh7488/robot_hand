"""Exact-commit native stage replay and motion-preview provenance checks."""
import argparse
import importlib.util
from pathlib import Path
import subprocess
import sys

import numpy as np
from PIL import Image

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args()
    if args.output.exists() or args.output.resolve().is_relative_to(ROOT):raise ValueError('New external output required')
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    if git('rev-parse','HEAD')!=args.commit or git('status','--porcelain=v1'):raise ValueError('Wrong/dirty target')
    spec=importlib.util.spec_from_file_location('_new_sequence_core',HERE/'final_sequence.py')
    f=importlib.util.module_from_spec(spec);spec.loader.exec_module(f)
    q,c,b,p=f.q,f.c,f.b,f.p
    manifest=p.read(ROOT/'checkpoints/ASSEMBLY013.manifest.json')
    frozen={}
    for key in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[key])
    def guard():return all(b.sha(ROOT/r)==v['sha256'] for r,v in manifest['files'].items()) and all(b.sha(ROOT/r)==v for r,v in frozen.items())
    if not guard():raise ValueError('Changed manifest/frozen inputs')
    saved,new,retained,arm=c.load_scene();groups={r['module']:r for r in saved['groups']}
    initial=p.read(HERE/'reviews/sequence.json');final=p.read(HERE/'reviews/final_sequence.json');legacy=p.read(HERE/'reviews/tilt_probe.json')
    assert initial['completed'] and final['completed'] and final['sampled_order_clear'] and legacy['completed']
    configs={r['id']:r for r in f.configurations(saved,new,retained)}
    oldconfigs={r['id']:r for r in q.configurations(saved,new,retained)}
    results=[];differences=[]
    def compare(label,got,want):
        sig=lambda r:{(v['lead'],v['part']):v['material_intersection_mm3'] for v in r['overlaps']}
        aa,bb=sig(got),sig(want)
        matched=aa.keys()==bb.keys() and all(abs(aa[k]-bb[k])<=1e-5 for k in aa.keys()&bb.keys()) and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope']
        results.append(dict(id=label,native=got,matched=matched))
        if not matched:differences.append(label)
        print(label,matched,flush=True)
    row=oldconfigs['carrier_ring_dip_actuator'];sample=initial['paths'][row['id']]['samples'][4]
    compare('initial_carrier_ring_dip_y8',q.material([dict(name=r['name'],shape=r['shape'].translate(sample['translation_mm'])) for r in row['moving']],row['fixed']),sample['native'])
    sample=legacy['samples'][1];row=oldconfigs['unit_'+sample['module']]
    compare('legacy_pitch_with_early_drum_and_horizontal_advance',q.material([f.a.a.pose(r,groups[sample['module']],sample['x_mm'],sample['pitch_deg']) for r in row['moving']],row['fixed']),sample['native'])
    selection=[('carrier_middle_dip_actuator',4),('carrier_ring_dip_actuator',4),
        ('unit_index_spread_actuator',5),('unit_middle_pip_actuator',5),('unit_ring_pip_actuator',5),
        ('unit_middle_pip_actuator',11),('rack_M3_before_motor_units',2),('base_nuts_after_units',2),
        ('drums_positioned_after_units',2),('bench_ring_pip_actuator',3)]
    for key,index in selection:
        row=configs[key];want=final['paths'][key]
        assert want['moving']==[r['name'] for r in row['moving']] and want['obstacles']==[r['name'] for r in row['fixed']]
        pose=row['poses'][index];assert pose==want['samples'][index]['pose']
        name=key.split('_',1)[1] if key.startswith(('carrier_','unit_','bench_')) else None
        got=q.material([f.move(r,pose,groups.get(name)) for r in row['moving']],row['fixed'])
        compare(key+'_'+str(index),got,want['samples'][index]['native'])
    parts=q.split(saved,new,retained)
    screws=[r for r in parts['units'] if r['name'].endswith('_base_screws')]
    cradles=[r for r in parts['units'] if r['name'].endswith('_cradle')]
    got=q.material([dict(name=r['name'],shape=r['shape'].translate((4,0,0))) for r in screws],cradles)
    matched=not got['overlaps'] and not got['operation_failures']
    results.append(dict(id='preload_subset_cradles_x4',native=got,matched=matched))
    if not matched:differences.append('preload_subset')
    preview=p.read(HERE/'previews/report.json');copies=[];image_checks=[]
    for clip in preview['clips']:
        matched=b.sha(ROOT/clip['output'])==b.sha(ROOT/clip['file'])==clip['sha256']
        copies.append(dict(id=clip['id'],matched=matched))
        if not matched:differences.append('motion_copy_'+clip['id'])
    with Image.open(ROOT/preview['montage']['path']) as movie:
        assert movie.n_frames==27 and movie.size==(1280,1640) and movie.info.get('loop')==0
        for index in [0,13,26]:
            movie.seek(index);pixels=movie.convert('RGB')
            for k,clip in enumerate(preview['clips']):
                with Image.open(ROOT/clip['file']) as source:
                    source.seek(preview['montage']['source_frame_indices'][index][k]);expected=source.convert('RGB')
                expected.thumbnail((620,690),Image.Resampling.LANCZOS)
                x=(k%2)*640+(640-expected.width)//2;y=92+(k//2)*750+36+(690-expected.height)//2
                actual=pixels.crop((x,y,x+expected.width,y+expected.height))
                error=float(np.abs(np.asarray(actual,dtype=np.float32)-np.asarray(expected,dtype=np.float32)).mean())
                image_checks.append(dict(frame=index,clip=clip['id'],mean_absolute_pixel_error=error,matched=error<5.))
                if error>=5.:differences.append('montage_'+str(index)+'_'+str(k))
    clean=not git('status','--porcelain=v1')
    report=dict(completed=True,target_commit=args.commit,passed=not differences and guard() and clean,
        differences=differences,key_samples=results,preview_copy_checks=copies,preview_frame_checks=image_checks,
        manifest_files=len(manifest['files']),frozen_files=len(frozen),input_hash_guard=guard(),checkout_clean=clean,
        independent_audit=False,native_shapes_reimported=353,neutral_arm_reimported=10,
        scope='13native probes including original failed orders,staged alternatives and preload subset,4original-GIF copies and12source/montage crops. Full original studies,hand collision paths,rendering,tools/jig/continuous collision/coupling/electrical/physical assembly not rerun/approved.')
    p.write(args.output,report)
    if not report['passed']:raise ValueError('Exact-commit review differs')


if __name__=='__main__':main()
