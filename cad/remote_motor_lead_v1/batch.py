"""Apply the corrected tail to all16remote candidate motors in one screen."""
import importlib.util
from pathlib import Path
import sys
import time

import cadquery as cq
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_batch_tail',HERE/'corrected.py')
c=importlib.util.module_from_spec(spec)
spec.loader.exec_module(c)
b,m,p=c.b,c.m,c.p


def wire_pairs(leads,v):
    boxes=[np.asarray(v.bounds(r['shape'])) for r in leads]
    overlaps=[]
    failures=[]
    exact=0
    for i,row in enumerate(leads):
        for j in range(i+1,len(leads)):
            if np.any(np.minimum(boxes[i][1],boxes[j][1])-np.maximum(boxes[i][0],boxes[j][0])<=0):
                continue
            exact+=1
            try:
                vol=float(row['shape'].intersect(leads[j]['shape'],tol=1e-6).Volume())
                if not np.isfinite(vol) or vol < -1e-9:
                    raise ValueError('Invalid volume')
                if vol>.001:
                    overlaps.append(dict(lead=row['name'],part=leads[j]['name'],material_intersection_mm3=vol))
            except Exception as exc:
                failures.append(dict(lead=row['name'],part=leads[j]['name'],error=repr(exc)))
    return dict(pairs_in_scope=len(leads)*(len(leads)-1)//2,exact_pairs=exact,overlaps=overlaps,operation_failures=failures)


def main():
    out=HERE/'reviews/batch.json'
    if out.exists():
        raise FileExistsError('Preserve prior batch')
    start=time.monotonic()
    first=p.read(HERE/'reviews/corrected.json')
    if not first['completed'] or not first['nominal_leads_clear']:
        raise ValueError('No screened source route')
    source=dict(first['source_sha256'])
    for rel in ['cad/remote_motor_lead_v1/batch.py','cad/remote_motor_lead_v1/reviews/corrected.json',
        'cad/form_candidate_v1/generated/scene.json']:
        source[rel]=b.sha(ROOT/rel)
    native,v,cat,scene,hand=p.load()
    obstacles=c.load_obstacles()
    modules=[r for r in p.read(ROOT/'cad/wiring_layout_v1/generated/layout.json')['modules'] if r['zone']!='palm']
    assert len(modules)==16
    ref=p.read(ROOT/'cad/wiring_access_v10/connector_reference.json')['connector']
    baseplug=cq.Solid.makeBox(*ref['registered_outer_size_mm'],cq.Vector(*ref['registered_outer_origin_mm'])).translate((-14.5,0,0))
    leads=[]
    plugs=[]
    stub=next(r['stub_mm'] for r in first['candidates'] if r['id']==first['selected'])
    for motor in modules:
        for i,y in enumerate([-11.5,-9.,-6.5],1):
            shape,desc=c.lead_shape(stub,y)
            leads.append(dict(name=motor['name']+'_lead'+str(i),module=motor['name'],local=desc,
                transform=motor['transform_world_mm'],shape=m.pack._transform(shape,motor['transform_world_mm'])))
        plugs.append(dict(name=motor['name']+'_plug_outer',module=motor['name'],shape=m.pack._transform(baseplug,motor['transform_world_mm'])))
    package=b.test_against(leads,obstacles,v)
    pairs=wire_pairs(leads,v)
    plugcheck=b.test_against(plugs,obstacles,v)
    result=dict(completed=False,source_sha256=source,remote_motor_candidates=16,nominal_electrical_leads=48,
        reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,packaging=package,wire_pairs=pairs,
        provisional_plug_check=plugcheck,hand_pose_checks=[],neutral_arm_check=None,
        connector_fit_verified=False,wire_sku_verified=False,bend_life_validated=False,
        strain_relief_complete=False,board_modelled=False,upstream_motor_layout_adopted=False,
        scope='Fixed remote motor lead space only. All motor positions and all hand geometry preserved; no connected bus, board, tendon or installation approval.')
    p.write(out,result)
    neutral=v.frames(cat['design'],np.zeros(20))
    for case in first['hand_pose_checks']:
        frames=v.frames(cat['design'],case['angles_deg'])
        current=[dict(name=r['name'],shape=v.relocate(hand[r['name']],frames[r['body']],neutral[r['body']])) for r in scene['hand']]
        got=b.test_against(leads,current,v)
        result['hand_pose_checks'].append(dict(id=case['id'],angles_deg=case['angles_deg'],native=got))
        p.write(out,result)
    arm=[]
    for r in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference']:
        rel='cad/modular_roots_v9/parts/'+r['asset']+'.step'
        source[rel]=b.sha(ROOT/rel)
        arm.append(dict(name=r['name'],shape=cq.importers.importStep(str(ROOT/rel)).val()))
    result['neutral_arm_check']=b.test_against(leads,arm,v)
    checks=[package,pairs,result['neutral_arm_check'],*[r['native'] for r in result['hand_pose_checks']]]
    bad={r['lead'] for chk in checks for r in chk['overlaps']+chk['operation_failures']}
    bad.update(r['part'] for r in pairs['overlaps']+pairs['operation_failures'] if 'part' in r)
    result['module_nominal_wire_clear']={motor['name']:not any(r['name'] in bad for r in leads if r['module']==motor['name']) for motor in modules}
    folder=HERE/'generated/batch_parts'
    folder.mkdir(exist_ok=False)
    saved=[]
    for row in leads+plugs:
        step=folder/(row['name']+'.step')
        mesh=folder/(row['name']+'.stl')
        cq.exporters.export(row['shape'],str(step))
        cq.exporters.export(row['shape'],str(mesh),tolerance=.05,angularTolerance=.1)
        rec=dict(name=row['name'],module=row['module'],role='nominal_electrical_wire_space' if row in leads else 'catalog_outer_box_unverified_mating',
            step=step.relative_to(ROOT).as_posix(),mesh=mesh.relative_to(ROOT).as_posix(),sha256_step=b.sha(step),sha256_stl=b.sha(mesh))
        if 'local' in row:
            rec.update(local=row['local'],transform_world_mm=row['transform'],
                world_end_mm=(np.asarray(row['transform'])@np.r_[row['local']['end_mm'],1.])[:3].tolist())
        saved.append(rec)
    p.write(HERE/'generated/batch_scene.json',dict(parts=saved,remote_motor_candidates=16,nominal_electrical_leads=48,
        board_termination='Open outboard tail endpoints only, no connected board/bus or restraint.',upstream_layout_adopted=False))
    result.update(completed=True,input_hash_guard=all(b.sha(ROOT/r)==h for r,h in source.items()),
        all_nominal_leads_clear=all(not r['overlaps'] and not r['operation_failures'] for r in checks),elapsed_s=time.monotonic()-start)
    p.write(out,result)
    if not result['input_hash_guard']:
        raise ValueError('Changed batch input')


if __name__=='__main__':
    main()
