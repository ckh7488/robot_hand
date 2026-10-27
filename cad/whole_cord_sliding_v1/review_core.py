"""Replay new driven states from a fresh exact-SHA checkout."""
import sys,argparse,subprocess
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import slide_model as s
k=s.m.drive.module('_slide045_native_aggregate',s.HERE/'check.py')
from OCP.OSD import OSD_ThreadPool


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*args):return subprocess.check_output(['git',*args],cwd=s.ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1') and not args.output.exists() and not args.output.resolve().is_relative_to(s.ROOT)
    manifest=s.p.read(s.ROOT/'checkpoints/SLIDE045.manifest.json');frozen={}
    for key in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(s.p.read(s.ROOT/'AUDIT_EVIDENCE_INDEX.json')[key])
    def guard():return all(s.m.drive.sha(s.ROOT/f)==z['sha256'] for f,z in manifest['files'].items()) and all(s.m.drive.sha(s.ROOT/f)==h for f,h in frozen.items())
    assert guard();raw=s.p.read(s.HERE/'reviews/verified_check.json');ev=s.p.read(s.HERE/'reviews/evidence.json');source,groups,housing,rotor,child=s.context()
    result=dict(completed=False,target_commit=args.commit,differences=[],native_samples=[],geometry_comparisons=[],profile_checks=[],adaptive_common_pairs=[])
    def save():s.a.save(args.output,result)
    def compare(label,got,want):
        aa={(z['lead'],z['part']):z['material_intersection_mm3'] for z in got['overlaps']};bb={(z['lead'],z['part']):z['material_intersection_mm3'] for z in want['overlaps']};err=max([abs(aa[q]-bb[q]) for q in aa.keys()&bb.keys()]+[0.])
        ok=aa.keys()==bb.keys() and err<1e-5 and got['pairs_in_scope']==want['pairs_in_scope'] and got['operation_failures']==want['operation_failures']
        result['native_samples'].append(dict(id=label,native=got,matched=bool(ok),maximum_basic_difference=err))
        if not ok:result['differences'].append(label)
        print(label,ok,flush=True);save()
    chosen=[raw['states'][j] for j in sorted({0,len(raw['states'])-1})];scenes={}
    for rec in chosen:
        q=rec['IP_deg'];rows=s.c.load(s.p.read(s.ROOT/rec['scene'])['parts']);lookup={z['name']:z for z in rows};scenes[q]=lookup
        cord=[z for z in rows if z.get('kind')=='cord'];changed=[z for z in rows if z.get('kind')!='cord'];posed=[(key,[lookup.get(z['name'],z) for z in zz]) for key,zz in groups]
        fixed=[z for _,zz in posed for z in zz if z['name'] not in rotor|child]
        tests={key:s.c.material(cord,zz) for key,zz in posed};tests['whole_housings']=s.c.material(cord,housing);tests['cord_self']=k.flat([s.c.material([z],cord[j+1:]) for j,z in enumerate(cord)])
        tests['moving_hardware_fixed']=s.c.material(changed,fixed);tests['moving_hardware_housings']=s.c.material(changed,housing);tests['rotor_child']=s.c.material([z for z in changed if z['name'] in rotor],[z for z in changed if z['name'] in child])
        for key,test in tests.items():compare(str(q)+'_'+key,test,rec['native'][key])
    # All seven material profiles are algebraic re-evaluations, not old free
    # curve solves or a physical hysteresis simulation.
    for rec in raw['profiles']:
        for i,want in enumerate(rec['lines']):
            got=s.profile(rec['IP_deg'],i);keys=['actual_length_mm','maximum_join_gap_mm','PAIR_rear_material_coordinate_mm','PAIR_rear_material_coordinate_change_mm']
            errors=[abs(got[z]-want[z]) for z in keys]+[float(s.np.max(abs(s.np.array(got['piece_lengths_mm'])-want['piece_lengths_mm'])))]+[float(s.np.max(abs(s.np.array(got[z])-want[z]))) for z in ['child_transform','motor_transform','tail_no_slip_endpoint_errors_mm']]
            errors += [float(s.np.max(abs(s.np.array(x['world_mm'])-y['world_mm']))) for x,y in zip(got['material_points'],want['material_points'])]
            errors.append(float(s.np.max(abs(s.np.array(got['axis_sliding_witness']['world_mm'])-want['axis_sliding_witness']['world_mm']))))
            ok=max(errors)<1e-5;result['profile_checks'].append(dict(IP_deg=rec['IP_deg'],line=i,matched=bool(ok),maximum_difference=max(errors)))
            if not ok:result['differences'].append('profile_'+str(rec['IP_deg'])+'_'+str(i))
            save()
    last=raw['states'][-1];q=last['IP_deg'];saved=scenes[q];fresh={z['name']:z for z in s.cords(q)+s.hardware(groups,q,rotor,child)[1]}
    selected=[z for z in fresh if z.startswith('sliding_cord_') and fresh[z]['path_segment'] in [0,1,22 if fresh[z]['physical_line']==0 else 26,23 if fresh[z]['physical_line']==0 else 27]]
    selected+=['thumb_ip_two_tail_spool','thumb_ip_tail_clamp_1','thumb_segment3_tip_core','thumb_ip_tendon_clamp']
    for name in selected:
        got=dict(name=name,**s.e.measure.match(s.e.measure.metrics(fresh[name]['shape']),s.e.measure.metrics(saved[name]['shape'])));result['geometry_comparisons'].append(got)
        if not got['matched']:result['differences'].append('metric_'+name)
        save()
    for z in ev['new_common_pairs']:
        saved_all={v['name']:v for v in [u for _,zz in groups for u in zz]+housing};saved_all.update(scenes[z['IP_deg']]);pair=z['pair']
        sh=saved_all[pair['lead']]['shape'].intersect(saved_all[pair['part']]['shape'],tol=1e-6);got=s.e.measure.metrics(sh)['global_values'];err=float(max(abs(s.np.array(got)-z['adaptive_common_values'])));ok=err<1e-5
        result['adaptive_common_pairs'].append(dict(pair=pair,STEP_common_values=got,source_common_values=z['adaptive_common_values'],maximum_difference=err,matched=bool(ok)))
        if not ok:result['differences'].append('common_'+pair['lead'])
        save()
    result.update(completed=True,passed=not result['differences'] and guard() and not git('status','--porcelain=v1'),
        manifest_files=len(manifest['files']),frozen_files=len(frozen),input_hash_guard=guard(),checkout_clean=not git('status','--porcelain=v1'),OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),independent_audit=False,
        inherited_CORD043_and_BIARC039_core_false_preserved=True,physical_fixation_or_hand_motion_approved=False)
    save()


if __name__=='__main__':main()
