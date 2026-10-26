"""Two bounded installed clamp roll alternatives, no hand angle changes."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import retention_model as m
check=m.drive.module('_retain026_probe_inputs',Path(__file__).resolve().parent/'check.py')
g,c,p,np,cq,ROOT,HERE=check.g,check.c,check.p,check.np,check.cq,check.ROOT,check.HERE
def rolled(rows,design,angle):
    port=next(z for z in design['ports'] if z['tag']=='distal_housing_clamp_1');Q=np.asarray(port['Q']);a=np.radians(angle);T=np.eye(4);T[:3,:3]=[[np.cos(a),0,np.sin(a)],[0,1,0],[-np.sin(a),0,np.cos(a)]];W=Q@T@np.linalg.inv(Q)
    return [dict(z,shape=g.original.transform(z['shape'],W)) if z['name'].startswith('distal_housing_clamp_1_') else z for z in rows]
def main():
    out=HERE/'reviews/roll_probe.json';assert not out.exists();r=p.read(HERE/'reviews/check.json');fresh=c.load(p.read(ROOT/r['scene'])['parts']);data,pack,guide,housing=check.inputs(r['design']);report=dict(completed=False,candidates=[],source_sha256=data[0],hand_angles_unchanged=True,geometry_rotated_not_cut=True)
    for angle in [90.,-90.]:
        rows=rolled(fresh,r['design'],angle);moving=[z for z in rows if z['name'].startswith('distal_housing_clamp_1_')];rest=[z for z in rows if z not in moving];cases=[]
        for label in ['parent_three_quarter','parent_full','open']:
            case=next(z for z in r['cases'] if z['id']==label);hand,v,f,neutral=m.s.k.e.prior.hand_rows(data,guide,case['angles_deg']);posed=[dict(z,shape=v.relocate(z['shape'],f[z['body']],neutral[z['body']])) for z in moving];others=[dict(z,shape=v.relocate(z['shape'],f[z['body']],neutral[z['body']])) if z['body']!='forearm' else z for z in rest];hous=[dict(z,shape=v.relocate(z['shape'],f[z['body']],neutral[z['body']])) if z['body']!='forearm' else z for z in housing]
            tests={k:c.material(posed,ob) for k,ob in [('hand',hand),('other_new',others),('housing',hous),('pack',pack),('electrical',data[6]),('arm',data[7])]};cases.append(dict(id=label,native=tests,clear=all(m.drive.clear(z) for z in tests.values())));print(angle,label,{k:len(z['overlaps']) for k,z in tests.items()},flush=True)
        got=dict(angle_deg=angle,cases=cases,clear=all(z['clear'] for z in cases));report['candidates'].append(got);p.write(out,report)
        if got['clear']:break
    report.update(completed=True,selected_roll_deg=next((z['angle_deg'] for z in report['candidates'] if z['clear']),None),whole_candidate_rechecked=False,input_hash_guard=all(m.drive.sha(ROOT/z)==h for z,h in data[0].items()));p.write(out,report)
if __name__=='__main__':main()
