"""Correct obstructed fastener/tool approaches without changing CAD geometry."""
import importlib.util
from pathlib import Path
import sys
import time

import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_mount_service_final',HERE/'selected3/check.py')
f=importlib.util.module_from_spec(spec);spec.loader.exec_module(f)
g,c,p,b,m=f.g,f.c,f.p,f.b,f.m


def configurations(new,retained,handrows,hand,groups,params,layout):
    raw={r['id']:r for r in f.configurations(new,retained,handrows,hand,groups,params,layout)}
    nut=next(r for r in new if r['name']=='Palm_collar_2_right_nut')
    other=[r for r in new if r['kind']=='nominal_M2_nut_envelope' and r['name']!=nut['name']]
    side_sign={r['nut']:(-1 if r['id'].endswith('left') else 1) for r in groups}
    vectors=[{r['name']:[side_sign[r['name']]*4,-1,z] for r in other} for z in [12,9,6,3,0]]
    vectors += [{r['name']:[side_sign[r['name']]*x,-1,0] for r in other} for x in [3,2,1,0]]
    vectors += [{r['name']:[0,y,0] for r in other} for y in [-.5,0]]
    three=dict(id='three_nuts_offset_drop_then_seat',moving=other,fixed=raw['clamp_nuts_from_above']['fixed'],
        translations_by_part=vectors,
        stage='Three nuts drop withX4mmoutward/Y−1mmoffset,then move inward and axially into their actual nut bearing seats. Fourth nut and clamp screws later;full wrench/holding unverified.')
    side=dict(id='one_nut_side_then_south',moving=[nut],fixed=raw['clamp_nuts_from_above']['fixed']+other,
        translations=[[x,-3,0] for x in [12,9,6,3,0]]+[[0,y,0] for y in [-2.25,-1.5,-.75,0]],
        stage='Three other nuts already seated from above;one nut enters laterally fromX+12 atY−3,then slides3mm into its actual hex bearing recess below the motor. Clamp screws later;manual holding/full wrench remains unverified.')
    tool=dict(raw['declared_tool_stems'],id='tool_stems_short_entry',translations=[[0,y,0] for y in [4,3,2,1,0]],
        stage='Declared15mmstems start with the hex tips3mm outside their own screw heads,then approach/withdraw4mm. Full handles/supplier tools/operator entry remain unverified.')
    return three,side,tool,raw


def main():
    out=HERE/'reviews/final_service.json'
    if out.exists():raise FileExistsError('Preserve prior service study')
    expected=p.read(HERE/'selected3/reviews/check.json')
    assert expected['completed'] and expected['input_hash_guard'] and expected['final_nominal_clear'] and expected['nominal_post_contact_confirmed']
    badnuts=[v for s in expected['paths']['clamp_nuts_from_above']['samples'] for v in s['native']['overlaps']]
    assert badnuts and all(v['lead'].startswith('Palm_collar_') and v['lead'].endswith('_nut') for v in badnuts)
    source={z:b.sha(ROOT/z) for z in ['cad/palm_motor_mount_v1/service_correction.py','cad/palm_motor_mount_v1/selected3/reviews/check.json','cad/palm_motor_mount_v1/selected3/generated/scene.json']}
    saved=p.read(HERE/'selected3/generated/scene.json');new=c.load(saved['new_parts'],source);retained=c.load(saved['retained_parts'],source)
    _,v,cat,scene,hand=p.load();neutral=v.frames(cat['design'],np.zeros(20));frames=v.frames(cat['design'],expected['assembly_hand_angles_deg'])
    assembly={r['name']:v.relocate(hand[r['name']],frames[r['body']],neutral[r['body']]) for r in scene['hand']}
    layout=p.read(ROOT/'cad/palm_motor_layout_v1/selected/generated/layout.json')
    three,side,tool,raw=configurations(new,retained,scene['hand'],assembly,saved['groups'],saved['params'],layout)
    reject={'clamp_nuts_from_above'}
    badtools=any(s['native']['overlaps'] or s['native']['operation_failures'] for s in expected['paths']['declared_tool_stems']['samples'])
    if badtools:reject.add('declared_tool_stems')
    inherited=[key for key in expected['paths'] if key not in reject]
    assert all(not s['native']['overlaps'] and not s['native']['operation_failures'] for key in inherited for s in expected['paths'][key]['samples'])
    report=dict(completed=False,source_sha256=source,geometry_changed=False,assembly_hand_angles_deg=expected['assembly_hand_angles_deg'],paths={},
        inherited_full_paths=inherited,
        original_failures_preserved=True,tool_entry_changed=badtools,reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,
        scope='Three nut offset/drop/axial entries,one nut side/axial entry,conditional short tool stem entry;reuse explicitly unchanged complete paths.No fulltool/fixture/thread/torque/friction/strength or physical installation approval.')
    for config in [three,side]+([tool] if badtools else []):
        record=dict(moving=[r['name'] for r in config['moving']],obstacles=[r['name'] for r in config['fixed']],stage=config['stage'],samples=[])
        report['paths'][config['id']]=record
        for i,pose in enumerate(config.get('translations_by_part',config.get('translations'))):
            result=c.material(f.moved(config,i),config['fixed']);record['samples'].append(dict(index=i,translations=pose,native=result));p.write(out,report)
        print(config['id'],sum(len(s['native']['overlaps']) for s in record['samples']),flush=True)
    newmap={r['name']:r for r in new}
    report['tool_start_head_distances_mm']=[dict(tool=r['id'],distance_mm=g.tool(r,saved['params']).translate((0,4,0)).distance(newmap[r['screw']]['shape'])) for r in saved['groups']]
    report['side_nut_start_carrier_distance_mm']=side['moving'][0]['shape'].translate(side['translations'][0]).distance(next(r['shape'] for r in new if r['kind']=='four_adapter_carrier'))
    inherited_count=sum(len(expected['paths'][key]['samples']) for key in inherited)
    fresh_count=sum(len(r['samples']) for r in report['paths'].values())
    report.update(completed=True,input_hash_guard=all(b.sha(ROOT/z)==h for z,h in source.items()),
        inherited_samples=inherited_count,new_samples=fresh_count,selected_samples=inherited_count+fresh_count,
        selected_sampled_order_clear=all(not s['native']['overlaps'] and not s['native']['operation_failures'] for path in report['paths'].values() for s in path['samples']))
    p.write(out,report)
    print('complete',report['selected_sampled_order_clear'],report['selected_samples'],flush=True)
    if not report['input_hash_guard']:raise ValueError('Changed inputs')


if __name__=='__main__':main()
