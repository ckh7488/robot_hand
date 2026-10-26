"""Two representative clamp installations on already-seated terminal bodies.

The replacement ferrule's earlier installation is not inherited as approved.
"""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import retention_model as m
check=m.drive.module('_retain026_service_inputs',Path(__file__).resolve().parent/'check.py')
g,c,p,np,cq,ROOT,HERE=check.g,check.c,check.p,check.np,check.cq,check.ROOT,check.HERE
def main():
    out=HERE/'reviews/service.json';assert not out.exists();r=p.read(HERE/'selected/reviews/check.json');assert r['static_candidate_clear'];data,pack,guide,housing=check.inputs(r['design']);rows=c.load(p.read(ROOT/r['scene'])['parts']);hand,_,_,_=m.s.k.e.prior.hand_rows(data,guide,np.zeros(20));record=dict(completed=False,source_sha256=data[0],sites=[],replacement_ferrule_installation_tested=False,already_seated_terminal_bodies=True,cords_not_yet_threaded=True,physical_fixture_and_tools_verified=False,whole_assembly_approved=False)
    for tag in ['motor_housing_clamp_0','distal_housing_clamp_1']:
        port=next(z for z in r['design']['ports'] if z['tag']==tag);Q=np.asarray(port['Q']);axis=Q[:3,2];R=port['R'];names=[z['name'] for z in rows if z['name'].startswith(tag+'_')];fixed=[z for z in rows if z['name'] not in names];paths=[];installed=[]
        sequence=[(tag+'_jaw_-1',-axis,[14,7,2,0]),(tag+'_jaw_1',axis,[14,7,2,0])]+[(tag+'_nut_'+str(j),-axis,[10,5,0]) for j in [0,1]]+[(tag+'_bolt_'+str(j),axis,[10,5,0]) for j in [0,1]]
        for name,direction,distances in sequence:
            z=next(z for z in rows if z['name']==name);ob=hand+pack+data[6]+data[7]+housing+fixed+[next(z for z in rows if z['name']==n) for n in installed];samples=[]
            for distance in distances:
                delta=direction*distance;T=np.eye(4);T[:3,3]=delta;test=c.material([dict(z,shape=z['shape'].translate(tuple(delta)))],ob);samples.append(dict(translation_world_mm=delta.tolist(),native=test))
            path=dict(moving=name,installed_before=list(installed),other_clamps_and_replacement_ferrules_present=[z['name'] for z in fixed],samples=samples,clear=all(m.drive.clear(z['native']) for z in samples));paths.append(path);print(tag,name,path['clear'],flush=True)
            if not path['clear']:break
            installed.append(name)
        record['sites'].append(dict(tag=tag,paths=paths,all_six_parts_installed=len(installed)==6,clear=len(installed)==6));p.write(out,record)
    record.update(completed=True,samples=sum(len(z['samples']) for a in record['sites'] for z in a['paths']),required_representatives_clear=all(z['clear'] for z in record['sites']),input_hash_guard=all(m.drive.sha(ROOT/z)==h for z,h in data[0].items()));p.write(out,record)
if __name__=='__main__':main()
