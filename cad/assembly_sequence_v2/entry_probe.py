"""Pitch entry along the tilted fastener axis, with drums fitted later."""
import importlib.util
from pathlib import Path
import math
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_aligned_entry',HERE/'tilt_probe.py')
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
q,c,b,m,p=a.q,a.c,a.b,a.m,a.p


def pose(row,group,x,angle,lift=0.):
    result=a.pose(row,group,x,angle)
    dz=-(x+1.4)*math.tan(math.radians(angle))+lift
    result['shape']=result['shape'].translate((0,0,dz))
    return result


def main():
    out=HERE/'reviews/entry_probe.json'
    if out.exists():raise FileExistsError('Keep earlier aligned entry probe')
    saved,new,retained,arm=c.load_scene()
    configs=q.configurations(saved,new,retained)
    remote={r['module'] for r in saved['groups']}
    spools={r['name'] for r in retained if r.get('module') in remote and r['name'].endswith('_spool')}
    source={rel:b.sha(ROOT/rel) for rel in ['cad/assembly_sequence_v2/entry_probe.py','cad/assembly_sequence_v2/tilt_probe.py','cad/assembly_sequence_v2/check.py','cad/wire_guides_v1/generated/scene.json']}
    for row in new+retained:source[row['step']]=b.sha(ROOT/row['step'])
    report=dict(completed=False,source_sha256=source,samples=[],geometry_changed=False,spools_fitted_later=sorted(spools),
        scope='Engineering3degree assembly pitch with vertical correction to centre the screw axes in the2.8mm adapter plate;drums physically fitted later,not motor-joint movement or complete coupling approval.')
    for name in ['index_spread_actuator','middle_pip_actuator']:
        config=next(r for r in configs if r['id']=='unit_'+name)
        fixed=[r for r in config['fixed'] if r['name'] not in spools]
        group=next(r for r in saved['groups'] if r['module']==name)
        for x,angle,lift in [(6.,3.,-20.),(6.,3.,0.),(4.,3.,0.),(1.,3.,0.),(1.,0.,0.)]:
            got=q.material([pose(r,group,x,angle,lift) for r in config['moving']],fixed)
            report['samples'].append(dict(module=name,x_mm=x,pitch_deg=angle,extra_lift_mm=lift,native=got))
            p.write(out,report);print(name,x,angle,lift,len(got['overlaps']),flush=True)
    report.update(completed=True,input_hash_guard=all(b.sha(ROOT/r)==h for r,h in source.items()),
        sampled_clear=all(not r['native']['overlaps'] and not r['native']['operation_failures'] for r in report['samples']))
    p.write(out,report)


if __name__=='__main__':main()
