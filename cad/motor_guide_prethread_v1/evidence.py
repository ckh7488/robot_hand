"""Actual saved assembly context and bounded overlap evidence, no group replay."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import check_seat as k
s,t,a,m,g,c,p,np,ROOT,HERE=k.s,k.t,k.a,k.m,k.g,k.c,k.p,k.np,k.ROOT,k.HERE
measure=t.prior.measure


def inputs(raw):
    case=t.neutral();data,full=t.prior.inputs(case);pack=dict(full)['packaging'];hw=[next(z for z in pack if z['name']=='motor_'+str(i)+'_jaw_with_fixed_bend') for i in [0,1]];groups=[(key,[z for z in zz if z['name'] not in raw['initial_missing_clamp_names']]) for key,zz in full]
    return case,data,full,groups,hw


def fresh_state(case,record,hw):
    built=[s.build(case,z['line'],z['phase'],z['value'],hw[z['line']]) for z in record['states']]
    return sum([z[0] for z in built],[])+[built[i][1] for i in record['active_guides']],built


def failure(raw):
    bad=next((z for z in raw['cases'] if not z['clear']),None)
    if bad:return bad,[]
    for path in raw['restoration']:
        for sample in path['samples']:
            if not sample['clear']:return dict(id='restoration',moving_name=path['moving'],**sample),path['installed_before']
    if raw.get('final') and not raw['final']['clear']:return dict(id='final',**raw['final']),[]
    return None,[]


def main():
    out=HERE/'reviews/overlap_evidence.json';assert not out.exists();raw=p.read(HERE/'reviews/check.json');assert raw['completed'];case,data,full,groups,hw=inputs(raw);bad,installed=failure(raw);results=[];evidence=[]
    if bad:
        moving,_=fresh_state(case,next(z for z in raw['cases'] if z['id']==bad['id']) if bad['id'] not in ['restoration','final'] else raw['cases'][-1],hw);lookup={z['name']:z for _,zz in full for z in zz};lookup.update({z['name']:z for z in moving})
        if bad['id']=='restoration':
            original=next(z for _,zz in full for z in zz if z['name']==bad['moving_name']);i=int(bad['moving_name'].split('_')[3]);axis=s.b.frame(i)[:3,2]*(-1 if '_nut_' in bad['moving_name'] else 1);lookup[original['name']]=dict(original,shape=original['shape'].translate(tuple(axis*bad['distance_mm'])))
        for key,test in bad['native'].items():
            for pair in test['overlaps']:
                sh=lookup[pair['lead']]['shape'].intersect(lookup[pair['part']]['shape'],tol=1e-6);values=measure.metrics(sh)['global_values'];results.append(dict(group=key,pair=pair,adaptive_source_common_values=values));evidence.append(dict(name='seat_actual_common_'+str(len(evidence)),shape=sh,body='palm',installed_part_candidate=False,color=[.95,.08,.07],pair=pair))
    scene=k.export(evidence,'actual_common') if evidence else None
    for f in ['cad/motor_guide_prethread_v1/evidence.py','cad/motor_guide_prethread_v1/seat_model.py','cad/motor_guide_prethread_v1/reviews/check.json']:data[0][f]=m.drive.sha(ROOT/f)
    a.save(out,dict(completed=True,failed_state=bad['id'] if bad else None,source_sha256=data[0],pairs=results,evidence_scene=scene,native_group_replays=0,original_basic_values_preserved=True,adaptive_epsilon=1e-10,absolute_comparison_tolerance=1e-5,input_hash_guard=all(m.drive.sha(ROOT/f)==h for f,h in data[0].items())))
if __name__=='__main__':main()
