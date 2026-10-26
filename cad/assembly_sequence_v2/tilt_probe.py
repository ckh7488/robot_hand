"""Finite entry-pitch probe at two representative left motor units."""
import importlib.util
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_entry_sequence',HERE/'check.py')
q=importlib.util.module_from_spec(spec);spec.loader.exec_module(q)
c,b,m,p=q.c,q.b,q.m,q.p


def pose(row,module,x,angle):
    cx,cy,cz=module['motor_centre_mm']
    pivot=(cx-9.1,cy,-94.9)
    shape=row['shape'].rotate(pivot,(pivot[0],pivot[1]+1,pivot[2]),angle).translate((x,0,0))
    return dict(name=row['name'],shape=shape)


def main():
    out=HERE/'reviews/tilt_probe.json'
    if out.exists():raise FileExistsError('Keep earlier entry probe')
    saved,new,retained,arm=c.load_scene()
    configs=q.configurations(saved,new,retained)
    source={rel:b.sha(ROOT/rel) for rel in ['cad/assembly_sequence_v2/tilt_probe.py','cad/assembly_sequence_v2/check.py','cad/wire_guides_v1/generated/scene.json']}
    for row in new+retained:source[row['step']]=b.sha(ROOT/row['step'])
    report=dict(completed=False,source_sha256=source,samples=[],geometry_changed=False,
        angle_deg=3.,pivot_rule_mm='(motor_centre_x-9.1,motor_centre_y,-94.9),axis+Y',
        scope='Engineering assembly entry pitch probe,not operating joint angles or complete assembly certification.')
    for name in ['index_spread_actuator','middle_pip_actuator']:
        config=next(r for r in configs if r['id']=='unit_'+name)
        group=next(r for r in saved['groups'] if r['module']==name)
        for x,angle in [(8.,3.),(4.,3.),(1.,3.),(1.,0.)]:
            got=q.material([pose(r,group,x,angle) for r in config['moving']],config['fixed'])
            report['samples'].append(dict(module=name,x_mm=x,pitch_deg=angle,native=got))
            p.write(out,report)
            print(name,x,angle,len(got['overlaps']),flush=True)
    report.update(completed=True,input_hash_guard=all(b.sha(ROOT/r)==h for r,h in source.items()),
        sampled_clear=all(not r['native']['overlaps'] and not r['native']['operation_failures'] for r in report['samples']))
    p.write(out,report)


if __name__=='__main__':main()
