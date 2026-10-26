"""Sample removable parts and an explicit staged base-installation order."""
import importlib.util
from pathlib import Path
import sys
import types

import cadquery as cq

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_guide_service',HERE/'model.py')
g=importlib.util.module_from_spec(spec)
spec.loader.exec_module(g)
b,m,p=g.b,g.m,g.p


def load(rows):
    return [dict(row,shape=cq.importers.importStep(str(ROOT/row['step'])).val()) for row in rows]


def tests(saved,new,retained,wires,arm):
    bounds=types.SimpleNamespace(bounds=m.pack._bbox)
    def samples(moving,fixed,steps,axis):
        result=[]
        for amount in steps:
            vector=[0.,0.,0.];vector[axis]=amount
            moved=[dict(name=r['name'],shape=r['shape'].translate(vector)) for r in moving]
            result.append(dict(translation_mm=vector,native=b.test_against(moved,fixed,bounds)))
        return result
    lids=[r for r in new if r['kind']=='removable_lid']
    cap_screws=[r for r in new if r['kind']=='M2x6_envelope']
    bolts=[r for r in new if r['kind']=='replacement_M3x12_envelope']
    bases=[r for r in new if r['kind']=='guide_base']
    left={r['module'] for r in saved['groups'] if r['motor_centre_mm'][0]<0}
    right={r['module'] for r in saved['groups'] if r['motor_centre_mm'][0]>0}
    result=dict(scope='Finite translated solid samples, not continuous collision, real tool access, full motor installation or physical assembly certification.',
        assembly_sequence=['Fit right-column guides while left-column motor modules are absent; wires and lid hardware are not fitted during base insertion.',
            'Fit left-column motor modules, then insert their guides from the left. Full motor-module installation path itself is not evaluated here.',
            'Insert longer M3 bolts/spacers, lay wires, fit nuts/lids and M2 screws. Actual supplier tool/torque/thread/preload remains unverified.'],
        lid_screw_removal=samples(cap_screws,[r for r in new if r['kind']!='M2x6_envelope']+retained+wires+arm,[0.,2.,4.,6.,8.],2),
        lid_removal_with_neutral_arm=samples(lids,[r for r in new if r['kind'] not in ['removable_lid','M2x6_envelope']]+retained+wires+arm,[0.,3.,6.,9.,12.],2),
        M3_insertion=samples(bolts,[r for r in new if r['kind']!='replacement_M3x12_envelope']+retained+wires+arm,[-12.,-9.,-6.,-3.,0.],2),
        full_population_base_attempt=samples(bases,retained+arm,[-20.,-15.,-10.,-5.,0.],0),
        right_base_staged=samples([r for r in bases if r['module'] in right],[r for r in retained if r.get('module') not in left]+arm,[-20.,-15.,-10.,-5.,0.],0),
        left_base_staged=samples([r for r in bases if r['module'] in left],retained+[r for r in new if r['module'] in right]+arm,[-20.,-15.,-10.,-5.,0.],0))
    required=['lid_screw_removal','lid_removal_with_neutral_arm','M3_insertion','right_base_staged','left_base_staged']
    result['required_sampled_paths_clear']=all(not r['native']['overlaps'] and not r['native']['operation_failures'] for key in required for r in result[key])
    result['full_population_base_insertion_blocked']=any(r['native']['overlaps'] for r in result['full_population_base_attempt'])
    return result


def main():
    out=HERE/'reviews/service.json'
    if out.exists():
        raise FileExistsError('Preserve prior service path evidence')
    saved=p.read(HERE/'generated/scene.json')
    check=p.read(HERE/'reviews/check.json')
    if not check['completed'] or not check['final_nominal_clear']:
        raise ValueError('No final clear guide')
    source=dict(check['source_sha256'])
    for path in [HERE/'service_check.py',HERE/'generated/scene.json',HERE/'reviews/check.json']:
        source[path.relative_to(ROOT).as_posix()]=b.sha(path)
    for row in saved['new_parts']:
        source[row['step']]=row['sha256_step'];source[row['mesh']]=row['sha256_stl']
    new=load(saved['new_parts']);retained=load(saved['retained_packaging'])
    wires=load(p.read(ROOT/saved['wire_scene'])['parts'])
    arm=[]
    for row in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference']:
        rel='cad/modular_roots_v9/parts/'+row['asset']+'.step';source[rel]=b.sha(ROOT/rel)
        arm.append(dict(name=row['name'],shape=cq.importers.importStep(str(ROOT/rel)).val()))
    report=tests(saved,new,retained,wires,arm)
    report.update(completed=True,source_sha256=source,input_hash_guard=all(b.sha(ROOT/r)==h for r,h in source.items()))
    p.write(out,report)
    if not report['input_hash_guard']:
        raise ValueError('Changed service input')


if __name__=='__main__':
    main()
