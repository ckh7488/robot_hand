"""Native mount, contact and bounded staged assembly checks."""
import importlib.util
from pathlib import Path
import sys
import time

import cadquery as cq
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_palm_mount_model',HERE/'model.py')
g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
c,p,b,m=g.c,g.p,g.b,g.m


def contacts(new,palm,params):
    parts={r['name']:r['shape'] for r in new}
    result=[]
    for k,(x,y) in enumerate(params['posts_xy_mm'],1):
        for angle in [-135,-45,45,135]:
            v=np.array([np.cos(np.radians(angle)),np.sin(np.radians(angle)),0.])
            centre=np.array([x,y,params['floor_z_mm']+params['collar_height_mm']/2])
            inner=centre+(params['post_radius_mm']-.05)*v
            outer=centre+(params['post_radius_mm']+.05)*v
            name='Palm_four_motor_carrier' if angle<0 else 'Palm_collar_'+str(k)+'_cap'
            states=[bool(s.isInside(cq.Vector(*point),1e-6)) for s,point in [(palm,inner),(palm,outer),(parts[name],inner),(parts[name],outer)]]
            result.append(dict(post=k,angle_deg=angle,part=name,inner_mm=inner.tolist(),outer_mm=outer.tolist(),
                               states=states,matched=states==[True,False,False,True]))
    return result


def configurations(new,retained,handrows,hand,groups,params,layout):
    motors={r['name']:r for r in layout['modules'] if r['zone']=='palm'}
    moving_modules=set(motors)
    units={n:[r for r in retained if r.get('module')==n and r['name'].rsplit(n+'_',1)[-1] in ['motor','rotor','cradle','body_screws','base_screws']] for n in motors}
    nuts=[r for r in retained if r.get('module') in moving_modules and r['name'].endswith('_base_nuts')]
    spools=[r for r in retained if r.get('module') in moving_modules and r['name'].endswith('_spool')]
    assert all(len(z)==5 for z in units.values()) and len(nuts)==len(spools)==4
    carrier=next(r for r in new if r['kind']=='four_adapter_carrier')
    caps=[r for r in new if r['kind']=='removable_collar_cap']
    bolts=[r for r in new if r['kind']=='nominal_M2x12_socket_envelope']
    capnuts=[r for r in new if r['kind']=='nominal_M2_nut_envelope']
    unitnames={r['name'] for z in units.values() for r in z}
    absent_names=unitnames|{r['name'] for r in nuts+spools}
    outside=[r for r in retained if r['name'] not in absent_names]
    uncovered=[dict(name=r['name'],shape=hand[r['name']]) for r in handrows if r['name']!='Palm_shaped_front_cover' and not r['name'].startswith('Cover_screw_')]
    result=[]
    # On the workbench the hand/posts, caps and reference drums are not fitted.
    for name in sorted(motors):
        axis=np.asarray(motors[name]['transform_world_mm'])[:3,2]
        otherunits=[r for n,z in units.items() if n!=name for r in z]
        result.append(dict(id='bench_unit_'+name,moving=units[name],fixed=[carrier]+otherunits,
            translations=[(axis*v).tolist() for v in [8,6,4,2,0]],
            stage='Off-hand carrier;motor/cradle with preloaded base screws is input. Other units conservatively seated. Base nuts,clamp caps and drums not fitted. Vendor internal/preloading/tool work remains separately scoped.'))
    result.append(dict(id='bench_base_nuts',moving=nuts,fixed=[carrier]+[r for z in units.values() for r in z],
        translations_by_part=[{r['name']:(-np.asarray(motors[r['module']]['transform_world_mm'])[:3,2]*v).tolist() for r in nuts} for v in [8,6,4,2,0]],
        stage='Base nuts fitted off-hand after the four motor units;hand/posts/clamps/drums absent.'))
    result.append(dict(id='populated_carrier_down_into_palm',moving=[carrier]+[r for z in units.values() for r in z]+nuts,
        fixed=outside+uncovered,translations=[[0,0,z] for z in range(70,-1,-5)],
        stage='Carrier with four motor/cradle units and base nuts lowered from above. Front cover and6cover head envelopes absent;two clamp caps/their fasteners and4reference drums fitted later. Rack/remote hardware and remaining hand present.'))
    for cap in caps:
        result.append(dict(id='cap_'+cap['name'],moving=[cap],fixed=[carrier]+retained+uncovered+[r for r in caps if r['name']!=cap['name']],
            translations=[[0,y,0] for y in [10,8,6,4,2,0]],stage='Front cover absent;other clamp cap conservatively seated;clamp bolts/nuts not yet fitted.'))
    result.append(dict(id='clamp_nuts_from_above',moving=capnuts,fixed=[carrier]+caps+retained+uncovered,
        translations=[[0,0,z] for z in [12,9,6,3,0]],stage='Nuts held at the south ear faces before screws. Operator holding/full wrench or jig is not modelled.'))
    result.append(dict(id='clamp_screws_from_north',moving=bolts,fixed=[carrier]+caps+capnuts+retained+uncovered,
        translations=[[0,y,0] for y in [8,6,4,2,0]],stage='Caps and nuts seated;front cover absent. Nominal screw envelopes only;actual tightening/preload is unverified.'))
    tools=[dict(name='Driver_'+r['id'],shape=g.tool(r,params)) for r in groups]
    result.append(dict(id='declared_tool_stems',moving=tools,fixed=new+retained+uncovered,
        translations=[[0,y,0] for y in [8,4,0]],stage='Four15mmstem/1mmhex-tip tool envelopes,handles and supplier tool/operator access unverified.'))
    return result


def moved(config,index):
    vectors=config.get('translations_by_part')
    return [dict(name=r['name'],shape=r['shape'].translate(vectors[index][r['name']] if vectors else config['translations'][index])) for r in config['moving']]


def main():
    out=HERE/'reviews/check.json'
    if out.exists():raise FileExistsError('Preserve earlier mount study')
    started=time.monotonic();source={r:b.sha(ROOT/r) for r in ['cad/palm_motor_mount_v1/model.py','cad/palm_motor_mount_v1/check.py','cad/palm_motor_mount_v1/parameters.json',
        'cad/palm_motor_layout_v1/selected/generated/scene.json','cad/palm_motor_layout_v1/selected/generated/layout.json',
        'cad/palm_motor_layout_v1/check.py','cad/wire_guides_v1/model.py','cad/palm_motor_layout_v1/reviews/check2.json']}
    adapters,retained=g.load_scene(source);new,groups=g.build(adapters);params=p.read(HERE/'parameters.json')
    rows=[]
    for row in new:
        folder=HERE/'generated/parts';folder.mkdir(parents=True,exist_ok=True)
        step=folder/(row['name']+'.step');mesh=folder/(row['name']+'.stl')
        cq.exporters.export(row['shape'],str(step));cq.exporters.export(row['shape'],str(mesh),tolerance=.15,angularTolerance=.15)
        rows.append(dict(name=row['name'],body='palm',kind=row['kind'],module=row['module'],color=row['color'],
            step=step.relative_to(ROOT).as_posix(),mesh=mesh.relative_to(ROOT).as_posix(),sha256_step=b.sha(step),sha256_stl=b.sha(mesh),bbox_mm=m.pack._bbox(row['shape'])))
    clean=lambda r:{k:v for k,v in r.items() if k!='shape'}
    p.write(HERE/'generated/scene.json',dict(new_parts=rows,retained_parts=[clean(r) for r in retained],replaced_adapters=[clean(r) for r in adapters],groups=groups,
        hand_unchanged=True,motor_positions_unchanged=True,remote_wiring_unchanged=True,installed=False,clamp_strength_validated=False,params=params))
    _,v,cat,scene,hand=p.load()
    for row in scene['hand']:source.update({row['step']:row['sha256_step'],row['mesh']:row['sha256_stl']})
    neutral=v.frames(cat['design'],np.zeros(20))
    report=dict(completed=False,source_sha256=source,hand_geometry_changed=False,motor_positions_changed=False,
        reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,new_parts=11,retained_parts=349,replaced_adapters=4,
        actual_motor_installation_approved=False,clamp_strength_validated=False,cases=[],paths={},
        scope='Native nominal mount/finite staged insertion only. No hand self-motion replay,continuous collision,friction/preload/strength,full tools/fixtures,coupling,electrical or physical approval.')
    for goal in p.read(ROOT/'cad/palm_motor_layout_v1/reviews/check2.json')['cases']:
        frames=v.frames(cat['design'],goal['angles_deg'])
        current=[dict(name=r['name'],shape=v.relocate(hand[r['name']],frames[r['body']],neutral[r['body']])) for r in scene['hand']]
        got=c.material(new,current);report['cases'].append(dict(id=goal['id'],angles_deg=goal['angles_deg'],native=got));p.write(out,report)
        print(goal['id'],len(got['overlaps']),len(got['operation_failures']),flush=True)
    report['retained_packaging']=c.material(new,retained)
    pairresults=[]
    for i,row in enumerate(new):
        if i+1<len(new):pairresults.append(c.material([row],new[i+1:]))
    report['new_pairs']=dict(pairs_in_scope=sum(z['pairs_in_scope'] for z in pairresults),exact_pairs=sum(z['exact_pairs'] for z in pairresults),
        overlaps=[q for z in pairresults for q in z['overlaps']],operation_failures=[q for z in pairresults for q in z['operation_failures']])
    wires=c.load(p.read(ROOT/'cad/remote_motor_lead_v2/generated/scene.json')['parts'],source)
    report['wires_and_plug_envelopes']=c.material(new,wires)
    arm=[]
    for r in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference']:
        rel='cad/modular_roots_v9/parts/'+r['asset']+'.step';source[rel]=b.sha(ROOT/rel)
        arm.append(dict(name=r['name'],shape=cq.importers.importStep(str(ROOT/rel)).val()))
    report['neutral_arm']=c.material(new,arm)
    report['post_contacts']=contacts(new,hand['Palm_tapered_chassis'],params)
    report['carrier_floor_distance_mm']=next(r['shape'] for r in new if r['kind']=='four_adapter_carrier').distance(hand['Palm_tapered_chassis'])
    tests=[z['native'] for z in report['cases']]+[report[k] for k in ['retained_packaging','new_pairs','wires_and_plug_envelopes','neutral_arm']]
    report['final_nominal_clear']=all(not z['overlaps'] and not z['operation_failures'] for z in tests)
    p.write(out,report)
    layout=p.read(ROOT/'cad/palm_motor_layout_v1/selected/generated/layout.json')
    for config in configurations(new,retained,scene['hand'],hand,groups,params,layout):
        record=dict(moving=[r['name'] for r in config['moving']],obstacles=[r['name'] for r in config['fixed']],stage=config['stage'],samples=[])
        report['paths'][config['id']]=record
        n=len(config.get('translations_by_part',config.get('translations',[])))
        for i in range(n):
            got=c.material(moved(config,i),config['fixed'])
            record['samples'].append(dict(index=i,translations=config.get('translations_by_part',config.get('translations'))[i],native=got))
            p.write(out,report)
        print(config['id'],n,sum(len(z['native']['overlaps']) for z in record['samples']),flush=True)
    report.update(completed=True,elapsed_s=time.monotonic()-started,
        input_hash_guard=all(b.sha(ROOT/r)==h for r,h in source.items()),
        sampled_paths_clear=all(not z['native']['overlaps'] and not z['native']['operation_failures'] for path in report['paths'].values() for z in path['samples']),
        nominal_post_contact_confirmed=all(z['matched'] for z in report['post_contacts']))
    p.write(out,report)
    print('completed',report['final_nominal_clear'],report['sampled_paths_clear'],report['nominal_post_contact_confirmed'],flush=True)
    if not report['input_hash_guard']:raise ValueError('Changed source')


if __name__=='__main__':main()
