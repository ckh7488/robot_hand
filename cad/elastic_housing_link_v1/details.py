"""Record actual intersections and the limited ideal-rod length results."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import check_elastic as b
a,m,g,c,p,np,cq,ROOT,HERE=b.a,b.m,b.g,b.c,b.p,b.np,b.cq,b.ROOT,b.HERE

def main():
    out=HERE/'reviews/details.json';assert not out.exists();raw=p.read(HERE/'reviews/check.json');assert raw['completed'];case=next((z for z in raw['native_cases'] if not z['clear']),raw['native_cases'][0]);rows=c.load(p.read(ROOT/case['scene'])['parts']);lookup={z['name']:z for z in rows};data,pack,guide,hardware=b.inputs();P,H=a.a.wrist_frames(case['wrist_flex_deg'],case['wrist_deviation_deg']);hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,case['angles_deg']);groups={'hand':[a.a.transform(z,H) for z in hand],'packaging':b.scope.posed_pack(pack,H),'electrical':data[6],'arm':a.a.posed_arm(data[7],P,H),'wrist_hardware':[a.a.transform(z,H) for z in hardware]};commons=[];pairs=[]
    for key,test in case['native'].items():
        if not test['overlaps']:continue
        hit=max(test['overlaps'],key=lambda z:z['material_intersection_mm3']);ob=next(z for z in groups[key] if z['name']==hit['part']);sh=lookup[hit['lead']]['shape'].intersect(ob['shape'],tol=1e-6);commons.append(dict(name='actual_'+key+'_common',shape=sh,body='palm',color=[.99,.03,.03],evidence_only=True));pairs.append(dict(group=key,**hit,common_name=commons[-1]['name'],saved_STEP_default_common_volume_mm3=sh.Volume()))
    internal=[hit for test in case['internal'] for hit in test['overlaps']]
    if internal:
        hit=max(internal,key=lambda z:z['material_intersection_mm3']);sh=lookup[hit['lead']]['shape'].intersect(lookup[hit['part']]['shape'],tol=1e-6);commons.append(dict(name='actual_internal_common',shape=sh,body='palm',color=[.99,.03,.03],evidence_only=True));pairs.append(dict(group='internal',**hit,common_name=commons[-1]['name'],saved_STEP_default_common_volume_mm3=sh.Volume()))
    evidence=b.export(commons,HERE/'generated/actual_common') if commons else None
    lengths=[];joins=[];sliding=[]
    for i in [0,1]:
        entries=[z['solutions'][i]['guide_material_coordinate_mm'] for z in raw['analytic_cases'] if z['admissible']];sliding.append(dict(physical_line=i,relative_to_open_range_mm=[float(min(entries)-entries[0]),float(max(entries)-entries[0])]))
    for z in raw['analytic_cases']:
        if not z['admissible']:continue
        for s in z['solutions']:
            up,down=s['upstream_arcs'],s['downstream_arcs'];e=s['anchors'];i=s['physical_line'];arcs=up+down
            gaps=[np.linalg.norm(np.asarray(v['end'])-w['start']) for chain in [up,down] for v,w in zip(chain[:-1],chain[1:])];gaps +=[np.linalg.norm(np.asarray(up[0]['start'])-e['A']),np.linalg.norm(np.asarray(up[-1]['end'])-e['U']),np.linalg.norm(np.asarray(down[0]['start'])-e['V']),np.linalg.norm(np.asarray(down[-1]['end'])-e['B'])]
            dots=[np.dot(v['tangent_end'],w['tangent_start']) for chain in [up,down] for v,w in zip(chain[:-1],chain[1:])];dots +=[np.dot(up[0]['tangent_start'],e['tA']),np.dot(up[-1]['tangent_end'],e['tG']),np.dot(down[0]['tangent_start'],e['tG']),np.dot(down[-1]['tangent_end'],e['tB'])]
            lengths.append(dict(case_id=z['id'],physical_line=i,housing_length_mm=17+96+25+sum(v['length_mm'] for v in arcs),middle_cord_length_mm=12+96+(23 if i==0 else 26)+sum(v['length_mm'] for v in arcs),supplier_cut_length_selected=False))
            joins.append(dict(case_id=z['id'],physical_line=i,max_centerline_join_gap_mm=float(max(gaps)),minimum_tangent_dot=float(min(dots))))
    repeat=[]
    for j,k in [(0,8),(1,7),(2,6),(3,5)]:
        if k>=len(raw['analytic_cases']):continue
        for i in [0,1]:
            s,t=raw['analytic_cases'][j]['solutions'][i],raw['analytic_cases'][k]['solutions'][i];diff=max(np.linalg.norm(np.asarray(v[key])-w[key]) for v,w in zip(s['upstream_arcs']+s['downstream_arcs'],t['upstream_arcs']+t['downstream_arcs']) for key in ['start','mid','end']);repeat.append(dict(approach=j,release=k,physical_line=i,max_centerline_sample_difference_mm=float(diff),numerical_local_solution_difference_not_physical_hysteresis=True))
    p.write(out,dict(completed=True,source_sha256=data[0],displayed_case_id=case['id'],actual_pairs=pairs,common_scene=evidence,analytic_lengths=lengths,analytic_joins=joins,guide_sliding=sliding,approach_release_same_angle_comparison=repeat,minimum_radius_range_mm=[min(s['minimum_arc_radius_mm'] for z in raw['analytic_cases'] for s in z['solutions']),max(s['minimum_arc_radius_mm'] for z in raw['analytic_cases'] for s in z['solutions'])],new_hardware_rows=0,existing_actual_supports_only=True,unsupported_integration_nodes_not_installation_guides=True,EI_gravity_contact_twist_friction_material_supply_unvalidated=True,continuous_material_force_equilibrium_or_mesh_convergence_verified=False,whole_tendon_installed_threaded_or_pretensioned=False))
if __name__=='__main__':main()
