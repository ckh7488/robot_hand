"""Retain completed native groups after an optional-metadata schema error."""
from pathlib import Path
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parent))
import model as g
from bbox_check import hand_boxes
c,b,m,p,cq,np,HERE,ROOT=g.c,g.b,g.m,g.p,g.cq,g.np,g.HERE,g.ROOT
original=g.module('_bus_initial_checker',HERE/'build_check.py')


def main():
    out=HERE/'reviews/final.json'
    if out.exists():raise FileExistsError('Preserve final assessment')
    started=time.monotonic();report=p.read(HERE/'reviews/check.json');assert not report['completed']
    source=report['source_sha256'];assert all(b.sha(ROOT/z)==h for z,h in source.items())
    for key in ['packaging','existing_wires_and_outerboxes','neutral_arm']:assert g.clear(report[key])
    for rel in ['cad/remote_bus_v1/resume.py','cad/remote_bus_v1/bbox_check.py','cad/remote_bus_v1/reviews/check.json']:
        source[rel]=b.sha(ROOT/rel)
    oldrows=p.read(ROOT/'cad/remote_motor_lead_v2/generated/scene.json')['parts']
    wires=g.build(oldrows);terms=g.terminations(wires);new=wires+terms
    old=c.load([z for z in oldrows if z.get('module') in g.MODULES and z.get('role')=='nominal_electrical_wire_space'])
    report['new_pairs']=g.pairs(new);report['continuity']=original.continuity(wires,old)
    report['hand_pose_bounds']=hand_boxes(new,source)
    report['termination_faces']=[dict(wire=z['name'],region=t['name'],distance_mm=float(z['shape'].distance(t['shape'])))
        for z in wires for t in terms if z['module']==t['module']]
    folder=HERE/'generated/parts';folder.mkdir(parents=True,exist_ok=False);rows=[]
    for z in new:
        step=folder/(z['name']+'.step');mesh=folder/(z['name']+'.stl')
        cq.exporters.export(z['shape'],str(step));cq.exporters.export(z['shape'],str(mesh),tolerance=.05,angularTolerance=.1)
        rows.append(dict(**{k:v for k,v in z.items() if k!='shape'},step=step.relative_to(ROOT).as_posix(),mesh=mesh.relative_to(ROOT).as_posix(),
            sha256_step=b.sha(step),sha256_stl=b.sha(mesh),bbox_mm=m.pack._bbox(z['shape'])))
    p.write(HERE/'generated/scene.json',dict(parts=rows,wire_extensions=6,temporary_spaces=2,space_only=True,installed=False,
        source_remote_leads='cad/remote_motor_lead_v2/generated/scene.json',current_packaging='cad/palm_motor_mount_v1/selected3/generated/scene.json',
        unconnected_remote_modules=14,complete_bus=False,assumptions='OD1.9mm and10/12/14mm static centreline bends;not supplied wire limits.14x20x6mm temporary connector regions are not a selected board or component.'))
    checks=[report[k] for k in ['packaging','existing_wires_and_outerboxes','neutral_arm','new_pairs']]
    report.update(completed=True,nominal_space_clear=all(g.clear(z) for z in checks) and all(z['clear'] for z in report['hand_pose_bounds']['cases']),
        continuity_confirmed=all(z['matched'] for z in report['continuity']),input_hash_guard=all(b.sha(ROOT/z)==h for z,h in source.items()),
        reused_completed_native_groups=['packaging','existing_wires_and_outerboxes','neutral_arm'],
        reuse_basis='Identical new geometry,obstacles and source hashes;initial interruption was missing optional valid metadata during hand-box iteration,not CAD interference.Original completed=false result preserved.',
        resumed_elapsed_s=time.monotonic()-started)
    p.write(out,report);print('complete',report['nominal_space_clear'],report['continuity_confirmed'],report['hand_pose_bounds']['hand_parts_native_imported'],flush=True)


if __name__=='__main__':main()
