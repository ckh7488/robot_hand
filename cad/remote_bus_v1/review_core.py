"""Exact-SHA replay of the partial bus geometry and its stated scope."""
import argparse
from pathlib import Path
import subprocess
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
import model as g
from bbox_check import hand_boxes
c,b,m,p,cq,np,HERE,ROOT=g.c,g.b,g.m,g.p,g.cq,g.np,g.HERE,g.ROOT
from OCP.OSD import OSD_ThreadPool


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    if args.output.exists() or args.output.resolve().is_relative_to(ROOT):raise ValueError('New external output required')
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1')
    manifest=p.read(ROOT/'checkpoints/BUS017.manifest.json');frozen={}
    for key in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[key])
    def guard():return all(b.sha(ROOT/z)==r['sha256'] for z,r in manifest['files'].items()) and all(b.sha(ROOT/z)==h for z,h in frozen.items())
    assert guard()
    expected=p.read(HERE/'reviews/final.json');assert expected['completed'] and expected['input_hash_guard'] and expected['nominal_space_clear'] and expected['continuity_confirmed']
    saved=p.read(HERE/'generated/scene.json');new=c.load(saved['parts']);wires=[z for z in new if z['role']=='provisional_continuous_wire_extension'];terms=[z for z in new if z not in wires]
    oldrows=p.read(ROOT/'cad/remote_motor_lead_v2/generated/scene.json')['parts'];old=c.load(oldrows)
    fresh=g.build(oldrows);fresh+=g.terminations(fresh);freshmap={z['name']:z for z in fresh}
    sig=g.module('_bus_analytic',ROOT/'cad/remote_motor_lead_v1/roundtrip_review.py').signatures
    differences=[];analytic=[];probes=[]
    for row in new:
        shape=freshmap[row['name']]['shape'];a,z=sig(shape,m.pack._bbox),sig(row['shape'],m.pack._bbox);bad=[];numbers=[]
        if not a['valid'] or not z['valid'] or a['solids']!=1 or z['solids']!=1:bad.append('valid/solid')
        for key in ['global_values','vertices']:
            aa,zz=np.asarray(a[key]),np.asarray(z[key])
            if aa.shape!=zz.shape or not np.allclose(aa,zz,rtol=0,atol=1e-5):bad.append(key)
            else:numbers.extend(abs(aa-zz).ravel().tolist())
        for key in ['faces','edges']:
            if len(a[key])!=len(z[key]):bad.append(key+' count');continue
            for aa,zz in zip(a[key],z[key]):
                if aa['kind']!=zz['kind'] or len(aa['values'])!=len(zz['values']) or not np.allclose(aa['values'],zz['values'],rtol=0,atol=1e-5):bad.append(key+' analytic')
                else:numbers.extend(abs(np.asarray(aa['values'])-np.asarray(zz['values'])).tolist())
        states=[]
        if row in wires:
            for point in [row['arc_middle_mm'],(np.asarray(row['arc_end_mm'])+row['end_mm'])/2]:
                state=[bool(s.isInside(cq.Vector(*point),1e-6)) for s in [shape,row['shape']]];states.append(state)
                if state!=[True,True]:bad.append('span interior')
        analytic.append(dict(name=row['name'],matched=not bad,mismatches=bad,max_numeric_difference=max(numbers or [0.]),span_interior_states=states))
        if bad:differences.append('analytic_'+row['name'])
    def compare(label,got,want):
        def values(z):return {(r['lead'],r['part']):r['material_intersection_mm3'] for r in z['overlaps']}
        a,z=values(got),values(want)
        matched=a.keys()==z.keys() and all(abs(a[k]-z[k])<=1e-5 for k in a.keys()&z.keys()) and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope']
        probes.append(dict(id=label,matched=matched,native=got))
        if not matched:differences.append(label)
        print(label,matched,flush=True)
    mount=p.read(ROOT/'cad/palm_motor_mount_v1/selected3/generated/scene.json');pack=c.load(mount['new_parts']+mount['retained_parts'])
    compare('360_current_packaging',c.material(new,pack),expected['packaging'])
    compare('48oldwires_16outerboxes',c.material(new,old),expected['existing_wires_and_outerboxes'])
    arm=[dict(name=z['name'],shape=cq.importers.importStep(str(ROOT/'cad/modular_roots_v9/parts'/(z['asset']+'.step'))).val())
         for z in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference']]
    compare('neutral_arm10',c.material(new,arm),expected['neutral_arm'])
    compare('28new_pairs',g.pairs(new),expected['new_pairs'])
    checker=g.module('_bus_continuity',HERE/'build_check.py')
    continuity=checker.continuity(wires,old);oldmap={z['name']:z for z in old}
    for got,want in zip(continuity,expected['continuity']):
        row=next(z for z in wires if z['name']==got['name']);oldrow=oldmap[row['source_lead']]
        edge=cq.Edge.makeThreePointArc(cq.Vector(*row['start_mm']),cq.Vector(*row['arc_middle_mm']),cq.Vector(*row['arc_end_mm']))
        direction=np.asarray(edge.tangentAt(0).toTuple());previous=np.asarray(oldrow['transform_world_mm'])[:3,:3]@(np.asarray(oldrow['local']['end_mm'])-oldrow['local']['arc_end_mm']);previous/=np.linalg.norm(previous)
        dot=float(np.dot(direction,previous));got['independent_arc_tangent_dot']=dot
        if not got['matched'] or abs(dot-1)>1e-8 or any(abs(got[k]-want[k])>1e-5 for k in ['end_start_distance_mm','common_volume_mm3','combined_volume_error_mm3']):differences.append('continuity_'+got['name'])
    bounds=hand_boxes(new);boundsmatch=len(bounds['cases'])==5
    for a,z in zip(bounds['cases'],expected['hand_pose_bounds']['cases']):
        boundsmatch &= a['id']==z['id'] and a['native_candidates']==z['native_candidates'] and a['clear']==z['clear'] and abs(a['minimum_separating_axis_gap_mm']-z['minimum_separating_axis_gap_mm'])<1e-5
    if not boundsmatch:differences.append('five_hand_pose_boxes')
    faces=[]
    for row in wires:
        term=next(z for z in terms if z['module']==row['module']);distance=float(row['shape'].distance(term['shape']))
        want=next(z for z in expected['termination_faces'] if z['wire']==row['name'])
        matched=abs(distance-want['distance_mm'])<1e-5 and distance<1e-5
        faces.append(dict(wire=row['name'],region=term['name'],distance_mm=distance,matched=matched))
        if not matched:differences.append('termination_'+row['name'])
    clean=not git('status','--porcelain=v1')
    result=dict(completed=True,target_commit=args.commit,passed=not differences and guard() and clean,differences=differences,
        key_samples=probes,analytic_comparisons=analytic,continuity=continuity,hand_pose_bounds=bounds,hand_pose_bounds_matched=bool(boundsmatch),termination_faces=faces,
        manifest_files=len(manifest['files']),frozen_files=len(frozen),input_hash_guard=guard(),checkout_clean=clean,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),
        independent_audit=False,scope='8regenerated/saved analytic shapes,12wire-span points;4native static groups,6continuity/fuse/tangent and6temporary face distances;5savedhand-box poses.No364hand native reimport when boxes are disjoint.No full bus14branches,clamp/strainrelief,assembly,future board,harnessSKU,current/voltdrop,dynamic arm/cable or physical approval.')
    p.write(args.output,result)
    if not result['passed']:raise ValueError('Bus replay differs')


if __name__=='__main__':main()
