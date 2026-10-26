"""Shift the entry8mm in Y to clear the first rack crossbar while lifting."""
import importlib.util
from pathlib import Path
import sys
import types

import cadquery as cq

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_guide_order2',HERE/'assembly_order.py')
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
g,b,m,p,s=a.g,a.b,a.m,a.p,a.s


def configurations(saved,new,retained,wires,arm):
    result=[]
    for key,moving,fixed,positions in a.configurations(saved,new,retained,wires,arm):
        if key=='right_lift':positions=[[-15.,8.,z] for z in [-20.,-15.,-10.,-5.,0.]]
        if key=='right_shift_y':positions=[[-15.,y,0.] for y in [8.,6.,4.,2.,0.]]
        result.append((key,moving,fixed,positions))
    return result


def main():
    out=HERE/'reviews/assembly_order_v2.json'
    if out.exists():raise FileExistsError('Preserve prior second order')
    first=p.read(HERE/'reviews/assembly_order.json')
    if not first['completed'] or not first['input_hash_guard']:raise ValueError('First order incomplete/changed')
    saved=p.read(HERE/'generated/scene.json');new=s.load(saved['new_parts']);retained=s.load(saved['retained_packaging']);wires=s.load(p.read(ROOT/saved['wire_scene'])['parts'])
    arm=[dict(name=r['name'],shape=cq.importers.importStep(str(ROOT/'cad/modular_roots_v9/parts'/(r['asset']+'.step'))).val())
        for r in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference']]
    source=dict(first['source_sha256'])
    for path in [HERE/'assembly_order_v2.py',HERE/'reviews/assembly_order.json']:source[path.relative_to(ROOT).as_posix()]=b.sha(path)
    report=dict(first,completed=False,source_sha256=source,paths=dict(first['paths']),
        supersedes_entry_only='First Y+10 lift hit the first crossbar at1sample; preserve it and change entry toY+8. Other four path groups are inherited under identical hashes.',
        assembly_sequence=[v.replace('Y+10','Y+8') for v in first['assembly_sequence']])
    bounds=types.SimpleNamespace(bounds=m.pack._bbox)
    for key,moving,fixed,positions in configurations(saved,new,retained,wires,arm):
        if key not in ['right_lift','right_shift_y']:continue
        report['paths'][key]=[]
        for vector in positions:
            got=b.test_against([dict(name=r['name'],shape=r['shape'].translate(vector)) for r in moving],fixed,bounds)
            report['paths'][key].append(dict(translation_mm=vector,native=got));p.write(out,report)
        print(key,sum(len(r['native']['overlaps']) for r in report['paths'][key]),flush=True)
    report.update(completed=True,input_hash_guard=all(b.sha(ROOT/r)==h for r,h in source.items()),
        conditional_sampled_order_clear=all(not r['native']['overlaps'] and not r['native']['operation_failures'] for rows in report['paths'].values() for r in rows))
    p.write(out,report)
    if not report['input_hash_guard']:raise ValueError('Changed second-order inputs')


if __name__=='__main__':main()
