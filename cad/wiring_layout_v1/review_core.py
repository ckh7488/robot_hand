"""Exact-SHA current-hand versus saved provisional motor placement replay."""
import argparse
import importlib.util
from pathlib import Path
import subprocess
import sys

import cadquery as cq
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--commit',required=True)
    ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args()
    if a.output.exists() or a.output.resolve().is_relative_to(ROOT):
        raise ValueError('New external output required')
    def git(*args):
        return subprocess.check_output(['git',*args],cwd=ROOT,text=True,encoding='utf-8',
            creationflags=subprocess.CREATE_NO_WINDOW).strip()
    if git('rev-parse','HEAD')!=a.commit or git('status','--porcelain=v1'):
        raise ValueError('Wrong/dirty checkout')
    spec=importlib.util.spec_from_file_location('_wire_core',HERE/'build_screen.py')
    m=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    p=m.p
    manifest=p.read(ROOT/'checkpoints/WIRE008.manifest.json')
    def guard():
        return all(m.sha(ROOT/r)==row['sha256'] for r,row in manifest['files'].items())
    frozen={}
    for key,value in p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json').items():
        if key in ['geometry_sha256','code_sha256','report_sha256']:
            frozen.update(value)
    if not guard() or not all(m.sha(ROOT/r)==h for r,h in frozen.items()):
        raise ValueError('Changed snapshot/frozen evidence')
    native,v,cat,scene,parts=p.load()
    source=p.read(HERE/'reviews/screen.json')
    refined=p.read(HERE/'reviews/refined.json') if (HERE/'reviews/refined.json').exists() else None
    final_source=refined if refined and refined.get('selected') else source
    before=m.read_saved_packaging()
    after=[]
    metadata=HERE/'generated/refined_packaging.json' if refined and refined.get('selected') else HERE/'generated/packaging.json'
    for row in p.read(metadata)['parts']:
        if m.sha(ROOT/row['step'])!=row['sha256_step']:
            raise ValueError('Changed saved proposed geometry')
        after.append(dict(name=row['name'],body=row['body'],module=row.get('module'),shape=cq.importers.importStep(str(ROOT/row['step'])).val()))
    probes=[('old_open',before,source['baseline'][0]['native'])]
    for row in final_source.get('cases',final_source.get('proposed',[])):
        if row['id'] in ['open','combined']:
            probes.append(('new_'+row['id'],after,row['native']))
    differences=[]
    results=[]
    def signature(row):
        return {(r['hand'],r['packaging']):r['material_intersection_mm3'] for r in row['overlaps']}
    for ident,packaging,expected in probes:
        got=m.screen(scene['hand'],packaging,v,scene,parts,cat['design'],expected['angles_deg'])
        b,c=signature(expected),signature(got)
        if b.keys()!=c.keys() or any(abs(b[k]-c[k])>1e-5 for k in b.keys() & c.keys()) or len(expected['operation_failures'])!=len(got['operation_failures']):
            differences.append(dict(id=ident,reason='Hand/packaging intersection or uncertainty changed'))
        results.append(dict(id=ident,native=got))
        print(ident,len(got['overlaps']),len(got['operation_failures']),flush=True)
    armrows=[]
    armparts={}
    for row in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference']:
        armparts[row['name']]=cq.importers.importStep(str(ROOT/'cad/modular_roots_v9/parts'/(row['asset']+'.step'))).val()
        armrows.append(dict(name=row['name'],body='palm'))
    arm=m.screen(armrows,after,v,scene,armparts,cat['design'],np.zeros(20))
    b,c=signature(final_source['arm_screen']),signature(arm)
    if b.keys()!=c.keys() or any(abs(b[k]-c[k])>1e-5 for k in b.keys() & c.keys()) or len(final_source['arm_screen']['operation_failures'])!=len(arm['operation_failures']):
        differences.append(dict(id='neutral_reference_arm',reason='Arm/packaging intersection changed'))
    cross=None
    if refined and refined.get('selected'):
        refspec=importlib.util.spec_from_file_location('_wire_cross_replay',HERE/'refine.py')
        refmodule=importlib.util.module_from_spec(refspec)
        refspec.loader.exec_module(refmodule)
        cross=refmodule.cross_modules(v,after)
        expected=refined['cross_module_screen']
        def cross_signature(report):
            return {(r['a'],r['b']):r['material_intersection_mm3'] for r in report['overlaps']}
        b,c=cross_signature(expected),cross_signature(cross)
        if b.keys()!=c.keys() or any(abs(b[k]-c[k])>1e-5 for k in b.keys() & c.keys()) or len(expected['operation_failures'])!=len(cross['operation_failures']):
            differences.append(dict(id='cross_modules',reason='Different-module packaging intersection changed'))
    clean=not git('status','--porcelain=v1')
    result=dict(target_commit=a.commit,passed=not differences and clean and guard(),clean_checkout=clean,input_hash_guard=guard(),
        manifest_files=len(manifest['files']),frozen_files=len(frozen),core_probes=results,neutral_arm=arm,cross_module_screen=cross,differences=differences,
        independent_audit=False,scope='Old open, proposed open/combined hand poses and neutral arm versus all saved packaging. No full motion, module internal/mating, source rebuilding, rendering, harness or physical assembly approval.')
    p.write(a.output,result)
    if not result['passed']:
        raise ValueError('Core reproduction differs')


if __name__=='__main__':
    main()
