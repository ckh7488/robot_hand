"""Only the selected upper keeper approach, with the old lower guide present."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import selected_check as k
m=k.m;g,c,p,np,cq,ROOT,HERE=m.g,m.c,m.p,m.np,m.cq,m.ROOT,m.HERE
from OCP.OSD import OSD_ThreadPool
def main():
    out=HERE/'reviews/keeper_entry.json';assert not out.exists();parts,guide,design=m.build();data,pack=k.inputs(design);data[0]['cad/thumb_sheath_terminal_v2/keeper_entry.py']=m.drive.sha(Path(__file__))
    original=c.load(p.read(ROOT/'cad/thumb_guide_assembly_v2/generated/parts/scene.json')['parts'],data[0]);absent=['liner_flange_keeper_1','liner_keeper_M2x10_1','liner_keeper_M2_nut_1'];installed=[z for z in original if z['name'] not in absent]
    hand,_,_,_=k.e.prior.hand_rows(data,installed,np.zeros(20));obstacles=hand+pack+data[6]+data[7];cap=next(z for z in parts if z['name']=='distal_housing_keeper_1');axis=g.original.layout()[1]['socket_transform'][:3,2]
    r=dict(completed=False,source_sha256=data[0],OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,hand_angles_deg=[0.]*20,moving=[cap['name']],omitted_original_guide=absent,installed_original_guide=[z['name'] for z in installed],missing_hand_parts=[],not_yet_installed=[z['name'] for z in parts if z['name']!=cap['name']]+['housing_reference_0','housing_reference_1','extension_cord'],samples=[],scope='Only upper keeper replacement approach.Existing thin liner and child/pin remain installed;lower guide still the original GUIDE021.Other new distal hardware/housing/cord not yet installed.Not full module assembly or screw/tool/jig/strength qualification.')
    for d in [14.,7.,3.,1.,.5,0.]:
        T=np.eye(4);T[:3,3]=axis*d;W=g.original.WORLD@T@g.original.INV
        r['samples'].append(dict(distance_mm=d,transform_local=T.tolist(),native=c.material([dict(name=cap['name'],shape=g.original.transform(cap['shape'],W))],obstacles)));p.write(out,r)
    r.update(completed=True,sampled_upper_keeper_entry_clear=all(m.drive.clear(z['native']) for z in r['samples']),input_hash_guard=all(m.drive.sha(ROOT/z)==h for z,h in data[0].items()));p.write(out,r);print('keeperentry',r['sampled_upper_keeper_entry_clear'],flush=True)
if __name__=='__main__':main()
