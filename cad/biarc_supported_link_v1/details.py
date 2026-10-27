"""Preplanned adaptive measurements of new overlap pairs, without group replay."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import whole_cord as w
l,a,m,g,c,p,np,ROOT,HERE=w.l,w.a,w.m,w.g,w.c,w.p,w.np,w.ROOT,w.HERE
support=m.drive.module('_biarc039_details_obstacles',ROOT/'cad/support_attachment_basis_v1/check.py')
measure=m.drive.module('_biarc039_details_metrics',ROOT/'cad/long_housing_route_v1/review_core.py')

def inputs(case):
    data,pack,guide,wrist=a.s.prior.inputs();hw=c.load([z for z in p.read(ROOT/'cad/support_attachment_basis_v1/generated/checked/scene.json')['parts'] if z['installed_part_candidate']],data[0]);removed=[z['replaced_part'] for z in hw];pack=[z for z in pack if z['name'] not in removed];cc=dict(angles_deg=case['q'],solution=dict(wrist_H=case['H'],wrist_P=case['P'],parent_F=case['F']));groups=support.kobs(data,pack,guide,wrist,cc);return data,[(key,zz+hw if key=='packaging' else zz) for key,zz in groups]

def main():
    out=HERE/'reviews/overlap_precision.json';assert not out.exists();raw=p.read(HERE/'reviews/check.json');case=p.read(HERE/'reviews/analysis.json')['cases'][0];data,groups=inputs(case);fresh,_,_=w.build(case);lookup={z['name']:z for z in fresh+[v for _,zz in groups for v in zz]};pairs=[]
    for group,test in [*raw['native'].items(),('internal',dict(overlaps=[z for test in raw['internal'] for z in test['overlaps']]))]:
        for z in test['overlaps']:pairs.append(dict(group=group,**z))
    chosen=[]
    for group in ['hand','packaging','internal']:
        subset=[z for z in pairs if z['group']==group]
        if subset:chosen.append(max(subset,key=lambda z:z['material_intersection_mm3']))
    evidence=[];results=[]
    for j,z in enumerate(pairs):
        sh=lookup[z['lead']]['shape'].intersect(lookup[z['part']]['shape'],tol=1e-6);values=measure.metrics(sh)['global_values'];results.append(dict(pair=z,source_common_adaptive_values=values,basic_source_common_mm3=sh.Volume()))
        if z in chosen:evidence.append(dict(name='biarc_actual_common_'+str(j),shape=sh,body='palm',color=[.95,.08,.07],installed_part_candidate=False,pair=z))
    scene=a.a.prior.export(evidence,HERE/'generated/actual_common');data[0]['cad/biarc_supported_link_v1/details.py']=m.drive.sha(Path(__file__));data[0]['cad/biarc_supported_link_v1/reviews/check.json']=m.drive.sha(HERE/'reviews/check.json');a.save(out,dict(completed=True,source_sha256=data[0],source_common_pairs=results,evidence_scene=scene,evidence_rows=len(evidence),native_group_replays=0,original_native_values_preserved=True,adaptive_epsilon=1e-10,absolute_comparison_tolerance=1e-5,input_hash_guard=all(m.drive.sha(ROOT/f)==h for f,h in data[0].items())));print('precision_pairs',len(results),'evidence',len(evidence),flush=True)
if __name__=='__main__':main()
