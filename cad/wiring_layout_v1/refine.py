"""Move motor modules within the palm cavity; never cut the accepted hand."""
import importlib.util
import json
from pathlib import Path
import sys
import time

import cadquery as cq
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_wire_refine',HERE/'build_screen.py')
m=importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
p=m.p
GROUPS={'thumb_base_sweep_actuator','thumb_opposition_actuator','ring_mcp_actuator'}


def cross_modules(v,parts):
    bounds={r['name']:np.array(v.bounds(r['shape'])) for r in parts}
    overlaps=[]
    errors=[]
    exact=0
    for i,row in enumerate(parts):
        for other in parts[i+1:]:
            # In-module contact/threads are outside this packaging-only test.
            if row.get('module')==other.get('module'):
                continue
            a,b=bounds[row['name']],bounds[other['name']]
            if np.any(np.minimum(a[1],b[1])-np.maximum(a[0],b[0])<=0):
                continue
            exact+=1
            try:
                volume=float(row['shape'].intersect(other['shape'],tol=1e-6).Volume())
                if volume>.02:
                    overlaps.append(dict(a=row['name'],b=other['name'],material_intersection_mm3=volume))
            except Exception as exc:
                errors.append(dict(a=row['name'],b=other['name'],error=repr(exc)))
    return dict(overlaps=overlaps,operation_failures=errors,exact_pairs=exact,
        scope='Different modules and rack only; in-module coupling/threads remain unverified.')


def main():
    out=HERE/'reviews/refined.json'
    if out.exists():
        raise FileExistsError('Preserve all tested placement candidates')
    started=time.monotonic()
    initial=p.read(HERE/'reviews/screen.json')
    if not initial['completed'] or not initial['input_hash_guard']:
        raise ValueError('Incomplete initial screening')
    source=dict(initial['source_sha256'])
    for rel in ['cad/wiring_layout_v1/refine.py','cad/wiring_layout_v1/reviews/screen.json','cad/wiring_layout_v1/generated/packaging.json']:
        source[rel]=m.sha(ROOT/rel)
    saved=p.read(HERE/'generated/packaging.json')['parts']
    parts=[]
    for row in saved:
        source[row['step']]=row['sha256_step']
        source[row['mesh']]=row['sha256_stl']
        if m.sha(ROOT/row['step'])!=row['sha256_step']:
            raise ValueError('Changed saved initial placement')
        parts.append(dict(row,shape=cq.importers.importStep(str(ROOT/row['step'])).val()))
    native,v,cat,scene,hand=p.load()
    report=dict(completed=False,source_sha256=source,candidates=[],selected=None,cases=[],arm_screen=None,cross_module_screen=None,
        hand_geometry_changed=False,hand_axes_changed=False,hand_lengths_changed=False,
        installed=False,whole_hand_harness_complete=False,connector_fit_verified=False,
        interpretation='Packaging candidates only. Lifting palm modules leaves mounting seats needing redesigned attachment; no load path/drive integration approval.')
    chosen=None
    for z in [3.,5.,7.]:
        deltas={name:[0.,0.,z] for name in GROUPS}
        deltas['ring_mcp_actuator']=[4.,-8.,z]
        candidate=[dict(row,shape=row['shape'].translate(deltas[row['module']]) if row.get('module') in GROUPS else row['shape']) for row in parts]
        got=m.screen(scene['hand'],candidate,v,scene,hand,cat['design'],np.zeros(20))
        external=cross_modules(v,candidate)
        clear=not(got['overlaps'] or got['operation_failures'] or external['overlaps'] or external['operation_failures'])
        report['candidates'].append(dict(id='palm_lift_'+format(z,'g'),translations_mm=deltas,native_open=got,cross_module_screen=external,clear=clear))
        p.write(out,report)
        print(json.dumps(dict(candidate=z,hand=len(got['overlaps']),cross=len(external['overlaps']),clear=clear)),flush=True)
        if clear:
            chosen=candidate
            report['selected']=report['candidates'][-1]['id']
            report['cross_module_screen']=external
            break
    if chosen:
        rows=[]
        for row in chosen:
            folder=HERE/'generated/refined_parts'
            folder.mkdir(parents=True,exist_ok=True)
            step=folder/(row['name']+'.step')
            mesh=folder/(row['name']+'.stl')
            cq.exporters.export(row['shape'],str(step))
            cq.exporters.export(row['shape'],str(mesh),tolerance=.2,angularTolerance=.2)
            rows.append(dict(name=row['name'],body=row['body'],module=row.get('module'),color=row['color'],
                step=step.relative_to(ROOT).as_posix(),mesh=mesh.relative_to(ROOT).as_posix(),
                sha256_step=m.sha(step),sha256_stl=m.sha(mesh),bbox_mm=np.asarray(v.bounds(row['shape'])).tolist()))
        p.write(HERE/'generated/refined_packaging.json',dict(parts=rows,motors=20,palm=4,remote=16,
            translations_mm=report['candidates'][-1]['translations_mm'],installed=False,hand_geometry_changed=False,
            limitations=['Palm seats require new mounting attachment; old seat contact is not preserved by a placement translation.',
                'In-module torque coupling, tendon routing, connector mating, harness, full wrist/arm motion, strength and hardware are unfinished.']))
        for old in initial['proposed']:
            got=m.screen(scene['hand'],chosen,v,scene,hand,cat['design'],old['native']['angles_deg'])
            report['cases'].append(dict(id=old['id'],native=got))
            p.write(out,report)
            print(json.dumps(dict(pose=old['id'],overlaps=len(got['overlaps']))),flush=True)
        armrows=[]
        armparts={}
        for row in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference']:
            armparts[row['name']]=cq.importers.importStep(str(ROOT/'cad/modular_roots_v9/parts'/(row['asset']+'.step'))).val()
            armrows.append(dict(name=row['name'],body='palm'))
        report['arm_screen']=m.screen(armrows,chosen,v,scene,armparts,cat['design'],np.zeros(20))
    report.update(completed=True,input_hash_guard=all(m.sha(ROOT/r)==h for r,h in source.items()),elapsed_s=time.monotonic()-started,
        hand_endpoints_clear=bool(chosen) and all(not r['native']['overlaps'] and not r['native']['operation_failures'] for r in report['cases']),
        neutral_arm_clear=bool(chosen) and not report['arm_screen']['overlaps'] and not report['arm_screen']['operation_failures'])
    p.write(out,report)
    if not report['input_hash_guard']:
        raise ValueError('Inputs changed')


if __name__=='__main__':
    main()
