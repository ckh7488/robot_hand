"""Guide/adapter carriers first, captive-screw motor/cradle units second.

All robot solids are retained. A base screw is loaded before its motor closes
the access path; the nut can then approach from behind the fixed adapter.
"""
import importlib.util
from pathlib import Path
import sys
import time
import types

import cadquery as cq

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_sequence_prior',ROOT/'cad/assembly_sequence_v1/check.py')
c=importlib.util.module_from_spec(spec);spec.loader.exec_module(c)
g,b,m,p=c.g,c.b,c.m,c.p
REPRESENTATIVES=['index_spread_actuator','middle_spread_actuator','middle_pip_actuator',
                 'middle_dip_actuator','ring_pip_actuator','ring_dip_actuator']
UNIT_SUFFIXES=('_motor','_rotor','_cradle','_body_screws','_base_screws')


def split(saved,new,retained):
    names={x['module'] for x in saved['groups']}
    carriers=[r for r in new if r['kind']!='replacement_M3x12_envelope']
    carriers += [r for r in retained if r.get('module') in names and r['name'].endswith('_adapter')]
    units=[r for r in retained if r.get('module') in names and r['name'].endswith(UNIT_SUFFIXES)]
    rack=[r for r in retained if r['name']=='Forearm_twenty_axis_module_rack']
    bolts=[r for r in new if r['kind']=='replacement_M3x12_envelope']
    racknuts=[r for r in retained if r.get('module') in names and '_rack_nut_' in r['name']]
    basenuts=[r for r in retained if r.get('module') in names and r['name'].endswith('_base_nuts')]
    assert len(carriers)==144 and len(units)==80 and len(rack)==1
    assert len(bolts)==len(racknuts)==32 and len(basenuts)==16
    return dict(carriers=carriers,units=units,rack=rack,bolts=bolts,racknuts=racknuts,basenuts=basenuts)


def configurations(saved,new,retained):
    parts=split(saved,new,retained)
    left={g['module'] for g in saved['groups'] if g['motor_centre_mm'][0]<0}
    configs=[]
    allrows=new+retained
    def add(key,moving,fixed,vectors,work):
        names={r['name'] for r in moving+fixed}
        configs.append(dict(id=key,moving=moving,fixed=fixed,translations=vectors,work=work,
                            explicitly_later=sorted(r['name'] for r in allrows if r['name'] not in names)))
    for name in REPRESENTATIVES:
        moving=[r for r in parts['carriers'] if r['module']==name]
        fixed=parts['rack']+[r for r in parts['carriers'] if r['module']!=name and (name in left or r['module'] not in left)]
        if name in left:
            steps=[[x,0.,0.] for x in [-20.,-15.,-10.,-5.,0.]]
        else:
            steps=[[-15.,8.,z] for z in [-20.,-15.,-10.,-5.,0.]]
            steps += [[-15.,y,0.] for y in [8.,6.,4.,2.,0.]]
            steps += [[x,0.,0.] for x in [-15.,-11.25,-7.5,-3.75,0.]]
        add('carrier_'+name,moving,fixed,steps,'Offline rack: right carriers before left carriers; motor/cradle units and rack M3 hardware fitted later.')
    add('rack_M3_before_motor_units',parts['bolts'],parts['carriers']+parts['rack'],
        [[0.,0.,z] for z in [-12.,-9.,-6.,-3.,0.]],'Carriers held by assembly jig until M3 bolts/nuts secure them.')
    add('rack_nuts_after_M3',parts['racknuts'],parts['carriers']+parts['rack']+parts['bolts'],
        [[0.,0.,z] for z in [8.,6.,4.,2.,0.]],'Rack still off the arm; base hardware/motor units fitted later.')
    for name in REPRESENTATIVES:
        moving=[r for r in parts['units'] if r['module']==name]
        moving_names={r['name'] for r in moving}
        def later(r):
            if r['name'] in moving_names or r['name']==name+'_base_nuts':return True
            if name in left and r.get('module') not in left:
                return r in parts['units'] or r in parts['basenuts']
            return False
        fixed=[r for r in allrows if not later(r)]
        add('unit_'+name,moving,fixed,[[x,0.,0.] for x in [8.,6.,4.,2.,0.]],
            'Preassemble motor/cradle with captive base screws away from rack; seat left units before right units; insert own base nuts afterwards.')
    nut_names={r['name'] for r in parts['basenuts']}
    add('base_nuts_after_units',parts['basenuts'],[r for r in allrows if r['name'] not in nut_names],
        [[x,0.,0.] for x in [-8.,-6.,-4.,-2.,0.]],'All retained source solids, including captive base screws, are present; no positive-overlap exemptions.')
    return configs


def material(moving,fixed):
    return b.test_against(moving,fixed,types.SimpleNamespace(bounds=m.pack._bbox))


def main():
    out=HERE/'reviews/sequence.json'
    if out.exists():raise FileExistsError('Preserve prior sequence candidates')
    started=time.monotonic()
    saved,new,retained,arm=c.load_scene()
    source=dict(p.read(ROOT/'cad/assembly_sequence_v1/reviews/base_hardware.json')['source_sha256'])
    for path in [HERE/'check.py',ROOT/'cad/assembly_sequence_v1/check.py',ROOT/'cad/wire_guides_v1/generated/scene.json']:
        source[path.relative_to(ROOT).as_posix()]=b.sha(path)
    for row in new+retained:source[row['step']]=b.sha(ROOT/row['step'])
    report=dict(completed=False,source_sha256=source,paths={},start_separation=[],
        geometry_changed=False,reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,
        representative_modules=REPRESENTATIVES,fixture_qualified=False,full_assembly_qualified=False,
        scope='Finite carrier insertion and captive-screw motor/cradle-unit seating at6representatives,plus32M3 bolts/32rack nuts/16base-nut compounds. Offline rack,actual jig/tools/torque/load/continuous collision and other10module paths unverified.',
        sequence=['Preload base M2screws into each cradle before seating its motor on the bench. Base nuts/adapter not attached to this unit yet.',
          'Insert right guide/adapter carriers first,then left carriers,without motor/cradle units or rack M3hardware. Jig holds contacting parts.',
          'Insert rack M3bolts/spacers and rack nuts to secure carriers before motor units close access.',
          'Seat left motor/cradle units with their captive base screws from+X8mm,then seat right units.',
          'Fit base M2nuts from-X;route wires and fit lids;verify rack-to-arm installation separately.'])
    for config in configurations(saved,new,retained):
        key=config['id']
        record=dict(moving=[r['name'] for r in config['moving']],obstacles=[r['name'] for r in config['fixed']],
                    explicitly_later=config['explicitly_later'],work=config['work'],samples=[])
        report['paths'][key]=record
        for vector in config['translations']:
            moved=[dict(name=r['name'],shape=r['shape'].translate(vector)) for r in config['moving']]
            result=material(moved,config['fixed'])
            record['samples'].append(dict(translation_mm=vector,native=result))
            p.write(out,report)
        if key.startswith('unit_'):
            name=key[5:];adapter=next(r['shape'] for r in retained if r['name']==name+'_adapter')
            gap=min(float(r['shape'].translate((8,0,0)).distance(adapter)) for r in config['moving'])
            report['start_separation'].append(dict(module=name,unit_to_adapter_start_gap_mm=gap,fully_separated=gap>1e-5))
        print(key,sum(len(r['native']['overlaps']) for r in record['samples']),flush=True)
    report.update(completed=True,elapsed_s=time.monotonic()-started,
        input_hash_guard=all(b.sha(ROOT/r)==h for r,h in source.items()),
        sampled_order_clear=all(not row['native']['overlaps'] and not row['native']['operation_failures'] for path in report['paths'].values() for row in path['samples']),
        motor_units_initially_separated=all(r['fully_separated'] for r in report['start_separation']))
    p.write(out,report)
    if not report['input_hash_guard']:raise ValueError('Changed sequence inputs')


if __name__=='__main__':main()
