"""Exact-SHA replay of saved guide/housing assembly and bounded failure evidence."""
import sys,argparse,subprocess,struct
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import evidence as e
s,t,a,m,g,c,p,np,ROOT,HERE=e.s,e.t,e.a,e.m,e.g,e.c,e.p,e.np,e.ROOT,e.HERE
k=e.k;measure=e.measure
from OCP.OSD import OSD_ThreadPool


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1') and not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
    manifest=p.read(ROOT/'checkpoints/SEAT041.manifest.json');frozen={}
    for key in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[key])
    def guard():return all(m.drive.sha(ROOT/f)==z['sha256'] for f,z in manifest['files'].items()) and all(m.drive.sha(ROOT/f)==h for f,h in frozen.items())
    assert guard();raw=p.read(HERE/'reviews/check.json');proof=p.read(HERE/'reviews/overlap_evidence.json');case,data,full,groups,hw=e.inputs(raw);result=dict(completed=False,target_commit=args.commit,differences=[],native_samples=[],material_checks=[],geometry_comparisons=[],adaptive_common_pairs=[]);saved_states={}
    def save():a.save(args.output,result)
    def compare(label,got,want):
        aa={(z['lead'],z['part']):z['material_intersection_mm3'] for z in got['overlaps']};bb={(z['lead'],z['part']):z['material_intersection_mm3'] for z in want['overlaps']};err=max([abs(aa[k]-bb[k]) for k in aa.keys()&bb.keys()]+[0.]);ok=aa.keys()==bb.keys() and err<1e-5 and got['pairs_in_scope']==want['pairs_in_scope'] and got['operation_failures']==want['operation_failures'];result['native_samples'].append(dict(id=label,native=got,matched=bool(ok),overlap_locations_match=aa.keys()==bb.keys(),max_basic_numeric_difference=err))
        if not ok:result['differences'].append(label)
        print(label,ok,flush=True);save()
    assert m.drive.sha(HERE/'seat_model.py')==m.drive.sha(HERE/'history/seat_model_attempt1.py.txt');result['boundary_checks']=[];result['arm_visual_unit_checks']=[]
    image=p.read(HERE/'images/report.json');arm_by={z['name']:z for key,zz in full if key=='arm' for z in zz}
    for z in image['arm_visual_references']:
        mesh=(ROOT/z['mesh']).read_bytes();count=struct.unpack('<I',mesh[80:84])[0];dt=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')]);points=np.frombuffer(mesh,dtype=dt,count=count,offset=84)['vertices'].reshape(-1,3);actual=np.array([points.min(axis=0),points.max(axis=0)]);expected=np.asarray(c.m.pack._bbox(arm_by[z['name']]['shape']));error=float(np.max(abs(actual-expected)));ok=z['mesh_units']=='mm' and error<.2;result['arm_visual_unit_checks'].append(dict(name=z['name'],mesh_bbox_mm=actual,native_STEP_bbox_mm=expected,maximum_bbox_difference_mm=error,display_only_tessellation_bbox_tolerance_mm=.2,matched=bool(ok)))
        if not ok:result['differences'].append('arm_visual_units_'+z['name'])
    previous=p.read(ROOT/'cad/whole_line_threading_v1/reviews/check.json')
    for i in [0,1]:
        old=next(z for z in reversed(previous['cases']) if z['kind']=='housing' and z['line']==i);_,_,start=s.edges(case,i,'feed',0.);_,_,feed_end=s.edges(case,i,'feed',s.definitions(case,i)[4]);_,_,bend_start=s.edges(case,i,'bend',0.);_,T,end=s.edges(case,i,'bend',1.);error=max(float(np.max(abs(T-np.eye(4)))),float(np.max(abs(np.asarray(end['housing_free_end_mm'])-s.m.endpoints(case['q'],i)['motor_seat']))))
        error=max(error,max(float(np.max(abs(np.asarray(z['world_mm'])-v['world_mm']))) for z,v in zip(start['material_points'],old['material_points'])),max(float(np.max(abs(np.asarray(z['world_mm'])-v['world_mm']))) for z,v in zip(feed_end['material_points'],bend_start['material_points'])));ok=error<1e-5;result['boundary_checks'].append(dict(line=i,maximum_difference_mm=error,THREAD040_start_and_feed_carry_boundary_and_nominal_motor_seat_match=bool(ok)))
        if not ok:result['differences'].append('boundary_'+str(i))
    for record in raw['cases']:
        for want in record['lines']:
            es,T,got=s.edges(case,want['line'],want['phase'],want['value']);err=max(abs(got['actual_length_mm']-want['actual_length_mm']),float(np.max(abs(T-np.asarray(want['guide_transform'])))),abs(got['PAIR_rear_material_s_mm']-want['PAIR_rear_material_s_mm']))
            for z,v in zip(got['material_points'],want['material_points']):err=max(err,float(np.max(abs(np.asarray(z['world_mm'])-v['world_mm']))))
            ok=err<1e-5;result['material_checks'].append(dict(id=record['id'],line=want['line'],maximum_difference=err,matched=bool(ok)))
            if not ok:result['differences'].append('material_'+record['id']+'_'+str(want['line']))
        if 'scene' not in record:continue
        saved=c.load(p.read(ROOT/record['scene'])['parts']);saved_states[record['id']]=saved
        for key,zz in groups:compare(record['id']+'_'+key,c.material(saved,zz),record['native'][key])
        compare(record['id']+'_internal',k.combine([c.material([z],saved[j+1:]) for j,z in enumerate(saved)]),record['native']['internal'])
    last=raw['cases'][-1];saved_last=saved_states.get(last['id']);full_map={z['name']:z for _,zz in full for z in zz}
    if raw['restoration']:
        assert saved_last;saved_by={z['name']:z for z in saved_last};housing=[z for z in saved_last if z.get('kind')=='housing']
        for path in raw['restoration']:
            installed=[saved_by[n] if n in saved_by else full_map[n] for n in path['installed_before']]
            for j,want in enumerate(path['samples']):
                if 'scene' not in want:continue
                moving=c.load(p.read(ROOT/want['scene'])['parts']);got=c.material(moving,[z for _,zz in groups for z in zz]+housing+installed);compare('restore_'+path['moving']+'_'+str(j),got,k.combine(list(want['native'].values())))
    if raw.get('final'):
        saved=c.load(p.read(ROOT/raw['final']['scene'])['parts']);saved_states['final']=saved
        for key,zz in full:compare('final_'+key,c.material(saved,zz),raw['final']['native'][key])
        compare('final_internal',k.combine([c.material([z],saved[j+1:]) for j,z in enumerate(saved)]),raw['final']['native']['internal'])
    chosen=next((z for z in raw['cases'] if z['id']=='carry_3'),next((z for z in reversed(raw['cases']) if z['id'].startswith('carry_')),raw['cases'][-1]));fresh,built=e.fresh_state(case,chosen,hw);saved=saved_states[chosen['id']];by={z['name']:z for z in fresh};selection=[z for z in saved if z['name'].startswith('motor_')]
    for i in [0,1]:
        ph=chosen['states'][i]['phase'];es,_,_=s.edges(case,i,ph,chosen['states'][i]['value']);curved=[z for z in saved if z.get('kind')=='housing' and z['physical_line']==i and es[z['segment']].geomType()!='LINE'];selection += [z for z in curved if z['segment'] in [3,4]] if ph=='bend' else curved[:1]
    for z in selection:
        test=dict(id=chosen['id'],name=z['name'],**measure.match(measure.metrics(by[z['name']]['shape']),measure.metrics(z['shape'])));result['geometry_comparisons'].append(test)
        if not test['matched']:result['differences'].append('metrics_'+z['name'])
        save()
    initial=not result['differences'] and guard() and not git('status','--porcelain=v1');bad,installed=e.failure(raw)
    if proof['pairs']:
        if bad['id'] in saved_states:objects=saved_states[bad['id']]
        elif bad['id']=='restoration':objects=saved_last+c.load(p.read(ROOT/bad['scene'])['parts'])
        else:objects=saved_states['final']
        lookup=dict(full_map);lookup.update({z['name']:z for z in objects})
        for want in proof['pairs']:
            pair=want['pair'];sh=lookup[pair['lead']]['shape'].intersect(lookup[pair['part']]['shape'],tol=1e-6);vals=measure.metrics(sh)['global_values'];err=float(np.max(abs(np.asarray(vals)-want['adaptive_source_common_values'])));ok=err<1e-5;result['adaptive_common_pairs'].append(dict(pair=pair,STEP_STEP_values=vals,maximum_difference=err,matched=bool(ok)))
    precise=all(z['matched'] for z in result['adaptive_common_pairs']);result.update(completed=True,initial_core_passed=initial,passed=initial and precise,explicit_results_reproduced=initial and precise,adaptive_common_pairs_passed=precise,manifest_files=len(manifest['files']),frozen_files=len(frozen),input_hash_guard=guard(),checkout_clean=not git('status','--porcelain=v1'),OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),independent_audit=False,physical_threading_forces_material_motion_or_whole_hand_approved=False);save()
if __name__=='__main__':main()
