"""Alternative assembly order; preserve the obstructed original paths.

This proves incremental clearance of the new guide parts. Existing module
hardware installation, fixtures/tools and the complete robot assembly are
not qualified by temporarily omitting hardware that is fitted later.
"""
import importlib.util
from pathlib import Path
import sys
import types

import cadquery as cq

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_guide_order',HERE/'service_check.py')
s=importlib.util.module_from_spec(spec);spec.loader.exec_module(s)
g,b,m,p=s.g,s.b,s.m,s.p


def configurations(saved,new,retained,wires,arm):
    left={r['module'] for r in saved['groups'] if r['motor_centre_mm'][0]<0}
    right={r['module'] for r in saved['groups'] if r['motor_centre_mm'][0]>0}
    bases=[r for r in new if r['kind']=='guide_base' and r['module'] in right]
    stages=[('right_lift',[[-15.,10.,z] for z in [-20.,-15.,-10.,-5.,0.]]),
        ('right_shift_y',[[-15.,y,0.] for y in [10.,7.5,5.,2.5,0.]]),
        ('right_shift_x',[[x,0.,0.] for x in [-15.,-11.25,-7.5,-3.75,0.]])]
    fixed_right=[r for r in retained if r.get('module') not in left]+arm
    hardware=lambda r:r['name'].endswith(('_base_screws','_base_nuts'))
    bolts=[r for r in new if r['kind']=='replacement_M3x12_envelope']
    fixed_bolt=[r for r in new if r['kind']!='replacement_M3x12_envelope']+[r for r in retained if not hardware(r)]+wires+arm
    configs=[(key,bases,fixed_right,positions) for key,positions in stages]
    configs.append(('M3_before_module_base_hardware',bolts,fixed_bolt,[[0.,0.,z] for z in [-12.,-9.,-6.,-3.,0.]]))
    # Later factory fastener paths are compared to NEW geometry only. The
    # original module's own hardware/case/tool fit remains outside this proof.
    configs.append(('module_base_screws_vs_new_only',[r for r in retained if r['name'].endswith('_base_screws')],new,[[x,0.,0.] for x in [8.,6.,4.,2.,0.]]))
    configs.append(('module_base_nuts_vs_new_only',[r for r in retained if r['name'].endswith('_base_nuts')],new,[[x,0.,0.] for x in [-8.,-6.,-4.,-2.,0.]]))
    return configs


def main():
    out=HERE/'reviews/assembly_order.json'
    if out.exists():raise FileExistsError('Preserve earlier alternate order')
    previous=p.read(HERE/'reviews/service.json')
    if not previous['completed']:raise ValueError('Original service study incomplete')
    saved=p.read(HERE/'generated/scene.json')
    new=s.load(saved['new_parts']);retained=s.load(saved['retained_packaging']);wires=s.load(p.read(ROOT/saved['wire_scene'])['parts'])
    arm=[dict(name=r['name'],shape=cq.importers.importStep(str(ROOT/'cad/modular_roots_v9/parts'/(r['asset']+'.step'))).val())
        for r in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference']]
    source=dict(previous['source_sha256']);source['cad/wire_guides_v1/assembly_order.py']=b.sha(HERE/'assembly_order.py')
    source['cad/wire_guides_v1/reviews/service.json']=b.sha(HERE/'reviews/service.json')
    report=dict(completed=False,source_sha256=source,paths={},
        original_required_paths_clear=False,original_service_report='cad/wire_guides_v1/reviews/service.json',
        scope='Conditional new-guide insertion and hardware order only. Module fixtures, own factory hardware/case insertion, tool access, complete motor installation and physical assembly remain unverified.',
        assembly_sequence=['Before left-column modules, insert each right guide at X-15/Y+10 from20mm below; lift,shiftY,then shiftX into place.',
            'Insert rack M3 bolts BEFORE fitting the motor/cradle base M2 screw/nut hardware; support unattached subassemblies with an assembly fixture, not modelled/qualified here.',
            'Factory base M2 screw/nut paths are checked against new parts only; own module insertion/tool fit is NOT approved.',
            'Left guides use the original left-stage path after modules are in place; place wires and close lids later.'])
    bounds=types.SimpleNamespace(bounds=m.pack._bbox)
    for key,moving,fixed,positions in configurations(saved,new,retained,wires,arm):
        report['paths'][key]=[]
        for vector in positions:
            got=b.test_against([dict(name=r['name'],shape=r['shape'].translate(vector)) for r in moving],fixed,bounds)
            report['paths'][key].append(dict(translation_mm=vector,native=got))
            p.write(out,report)
        print(key,sum(len(r['native']['overlaps']) for r in report['paths'][key]),flush=True)
    report.update(completed=True,input_hash_guard=all(b.sha(ROOT/r)==h for r,h in source.items()),
        conditional_sampled_order_clear=all(not r['native']['overlaps'] and not r['native']['operation_failures'] for rows in report['paths'].values() for r in rows))
    p.write(out,report)
    if not report['input_hash_guard']:raise ValueError('Changed order inputs')


if __name__=='__main__':main()
