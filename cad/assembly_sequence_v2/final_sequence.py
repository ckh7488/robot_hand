"""Rear-to-front carriers and a3degree motor-unit entry pitch.

Keep the unsuccessful straight-entry study. Reuse only identical completed
clear paths, and explicitly test all changed poses and stage obstacles.
"""
import importlib.util
from pathlib import Path
import sys
import time

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_sequence_entry_pitch',HERE/'entry_probe.py')
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
q,c,b,m,p=a.q,a.c,a.b,a.m,a.p


def configurations(saved,new,retained):
    rows=q.configurations(saved,new,retained)
    groups={r['module']:r for r in saved['groups']}
    spools=[r for r in retained if r.get('module') in groups and r['name'].endswith('_spool')]
    spool_names={r['name'] for r in spools}
    for row in rows:
        name=row['id'].split('_',1)[1] if row['id'].startswith(('carrier_','unit_')) else None
        group=groups.get(name)
        if group and row['id'].startswith('unit_'):
            row['fixed']=[r for r in row['fixed'] if r['name'] not in spool_names]
            row['explicitly_later']=sorted(set(row['explicitly_later'])|spool_names)
            row['work']+=' Drums are positioned after motor units;their actual horn/hub coupling remains unverified.'
        if group and row['id'].startswith('carrier_') and group['motor_centre_mm'][0]>0:
            # Each rear carrier enters towards+Y. Forward rows physically
            # belong to later assembly stages, preventing the known clash.
            later=[r for r in row['fixed'] if r.get('module') in groups and groups[r['module']]['motor_centre_mm'][1]>group['motor_centre_mm'][1]]
            later_names={r['name'] for r in later}
            row['fixed']=[r for r in row['fixed'] if r['name'] not in later_names]
            row['explicitly_later']=sorted(set(row['explicitly_later'])|later_names)
            row['work']+=' Within the right column,install rear rows first and forward rows later.'
        if group and row['id'].startswith('unit_') and group['motor_centre_mm'][0]<0:
            row['poses']=[dict(x_mm=6.,pitch_deg=3.,extra_lift_mm=z) for z in [-20.,-15.,-10.,-5.,0.]]
            row['poses'] += [dict(x_mm=x,pitch_deg=3.) for x in [4.,2.,1.]]
            row['poses'] += [dict(x_mm=1.,pitch_deg=t) for t in [1.5,0.]]
            row['poses'] += [dict(x_mm=x,pitch_deg=0.) for x in [.5,0.]]
            row['work']+=' Lift from20mm below at+X6mm/+3degree pitch,advance along the tilted screw axes,straighten at+X1mm,then seat.'
        else:
            row['poses']=[dict(translation_mm=v,pitch_deg=0.) for v in row['translations']]
    parts=q.split(saved,new,retained)
    rows.append(dict(id='drums_positioned_after_units',moving=spools,
        fixed=[r for r in new+retained if r['name'] not in spool_names],explicitly_later=[],
        work='Drum space/reference positioning only,not hub/horn connection or torque transfer approval.',
        poses=[dict(translation_mm=[0.,0.,z],pitch_deg=0.) for z in [-10.,-7.5,-5.,-2.5,0.]]))
    for name in q.REPRESENTATIVES:
        moving=[r for r in parts['units'] if r['module']==name and r['name'].endswith(('_motor','_rotor','_body_screws'))]
        fixed=[r for r in parts['units'] if r['module']==name and r['name'].endswith(('_cradle','_base_screws'))]
        rows.append(dict(id='bench_'+name,moving=moving,fixed=fixed,
            explicitly_later=sorted(r['name'] for r in new+retained if r not in moving+fixed),
            work='Off-rack unit assembly:base screws loaded in cradle first;motor/rotor/body-screw reference unit seats before attaching to adapter. Factory body-screw first fastening/tools unverified.',
            poses=[dict(translation_mm=[x,0.,0.],pitch_deg=0.) for x in [20.,15.,10.,5.,0.]]))
    return rows


def move(row,pose,group):
    if 'x_mm' in pose:return a.pose(row,group,pose['x_mm'],pose['pitch_deg'],pose.get('extra_lift_mm',0.))
    return dict(name=row['name'],shape=row['shape'].translate(pose['translation_mm']))


def main():
    out=HERE/'reviews/final_sequence.json'
    if out.exists():raise FileExistsError('Keep earlier final candidate')
    probe=p.read(HERE/'reviews/entry_probe.json')
    if not probe['completed'] or not probe['sampled_clear']:raise ValueError('Entry pitch probe is not clear')
    initial=p.read(HERE/'reviews/sequence.json')
    if not initial['completed']:raise ValueError('Initial complete study required')
    started=time.monotonic()
    saved,new,retained,arm=c.load_scene()
    source=dict(initial['source_sha256'])
    for path in [HERE/'final_sequence.py',HERE/'entry_probe.py',HERE/'tilt_probe.py',HERE/'reviews/sequence.json',HERE/'reviews/entry_probe.json',HERE/'reviews/tilt_probe.json',
        ROOT/'cad/assembly_sequence_v1/reviews/preassemble.json']:
        source[path.relative_to(ROOT).as_posix()]=b.sha(path)
    prior=p.read(ROOT/'cad/assembly_sequence_v1/reviews/preassemble.json')
    preload=prior['paths']['base_screws_before_motor']
    unitparts=q.split(saved,new,retained)['units']
    screw_names={r['name'] for r in unitparts if r['name'].endswith('_base_screws')}
    cradle_names={r['name'] for r in unitparts if r['name'].endswith('_cradle')}
    assert screw_names==set(preload['moving']) and cradle_names<=set(preload['obstacles'])
    assert all(not r['native']['overlaps'] and not r['native']['operation_failures'] for r in preload['samples'])
    assert all(b.sha(ROOT/r)==h for r,h in prior['source_sha256'].items())
    report=dict(completed=False,source_sha256=source,paths={},reused_identical_paths=[],start_separation=[],
        preload_inheritance=dict(report='cad/assembly_sequence_v1/reviews/preassemble.json',moving=sorted(screw_names),
            actual_fixed=sorted(cradle_names),old_superset_obstacles=preload['obstacles'],samples=5,
            identical_geometry_hashes=True,subset_obstacle_proof=True),
        geometry_changed=False,fixture_qualified=False,full_assembly_qualified=False,
        reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,
        scope='6representative carrier/unit paths and6bench paths plus rack/base fasteners;finite samples. Other10carrier/unit paths,physical captive hardware,tools,jig,load/torque,continuous collision,rack-to-arm,electrical/tendon integration unverified.')
    groups={r['module']:r for r in saved['groups']}
    for config in configurations(saved,new,retained):
        key=config['id'];old=initial['paths'].get(key)
        meta=dict(moving=[r['name'] for r in config['moving']],obstacles=[r['name'] for r in config['fixed']],
            explicitly_later=config['explicitly_later'],work=config['work'],samples=[])
        report['paths'][key]=meta
        same=old and old['moving']==meta['moving'] and old['obstacles']==meta['obstacles'] and len(old['samples'])==len(config['poses'])
        same=same and all('translation_mm' in pose and pose['translation_mm']==old['samples'][i]['translation_mm'] for i,pose in enumerate(config['poses']))
        same=same and all(not r['native']['overlaps'] and not r['native']['operation_failures'] for r in old['samples'])
        if same:
            meta['samples']=[dict(pose=pose,native=row['native']) for pose,row in zip(config['poses'],old['samples'])]
            report['reused_identical_paths'].append(key)
        else:
            name=key.split('_',1)[1] if key.startswith(('carrier_','unit_','bench_')) else None
            group=groups.get(name)
            for pose in config['poses']:
                got=q.material([move(r,pose,group) for r in config['moving']],config['fixed'])
                meta['samples'].append(dict(pose=pose,native=got));p.write(out,report)
            if key.startswith('unit_'):
                adapter=next(r['shape'] for r in retained if r['name']==name+'_adapter')
                gap=min(float(move(r,config['poses'][0],group)['shape'].distance(adapter)) for r in config['moving'])
                report['start_separation'].append(dict(module=name,minimum_gap_mm=gap,fully_separated=gap>1e-5))
        print(key,'reuse' if same else 'fresh',sum(len(r['native']['overlaps']) for r in meta['samples']),flush=True)
    report.update(completed=True,elapsed_s=time.monotonic()-started,
        input_hash_guard=all(b.sha(ROOT/r)==h for r,h in source.items()),
        sampled_order_clear=all(not row['native']['overlaps'] and not row['native']['operation_failures'] for path in report['paths'].values() for row in path['samples']),
        tilted_starts_separated=all(r['fully_separated'] for r in report['start_separation']))
    p.write(out,report)
    if not report['input_hash_guard']:raise ValueError('Changed final sequence inputs')


if __name__=='__main__':main()
