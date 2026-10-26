"""Keep the same nut/hand geometry; approach the two seats from the open side."""
import sys,copy,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from probe import *
from material_cache import install
install(c)
def main():
    out=HERE/'reviews/selected_service.json';assert not out.exists();start=time.monotonic()
    old=p.read(HERE/'reviews/final_service.json');assert old['completed'] and old['required_samples']==88
    assert all(g.g.drive.sha(ROOT/z)==h for z,h in old['source_sha256'].items())
    report=copy.deepcopy(old);report['completed']=False
    static=p.read(HERE/'reviews/static.json');design=static['design'];data=check.inputs(design)
    saved=c.load(p.read(ROOT/static['scene'])['parts']);by={z['name']:z for z in saved}
    for i in [1,0]:
        path=next(z for z in report['paths'] if z['id']=='liner_keeper_M2_nut_'+str(i)+'_radial');names=path['moving'];Ts=[translate([x,0,0]) for x in [-12,-9,-6,-3,0]]
        obstacles=[dict(name=z['name'],shape=data[4][z['name']]) for z in data[3]['hand'] if z['name'] not in path['missing_hand_parts']+design['replaced_hand_parts']+names]
        obstacles+=data[5]+data[6]+data[7]+[by[n] for n in path['installed_new'] if n not in names]
        samples=[]
        for T in Ts:
            moving=[dict(name=n,shape=g.original.transform(by[n]['shape'],g.original.WORLD@T@g.original.INV)) for n in names]
            samples.append(dict(transform_local=T.tolist(),native=c.material(moving,obstacles)))
        path.update(id='liner_keeper_M2_nut_'+str(i)+'_side_insert',samples=samples,clear=all(g.g.drive.clear(z['native']) for z in samples),scope='Same nut/seat/installed and absent parts,approach from local-X12mm at seated radial height,not through parent material along the inward radial direction.Manual nut retention;tool and preload unverified.')
        path.pop('reused_from',None);print(path['id'],path['clear'],flush=True)
    report['source_sha256']['cad/thumb_guide_assembly_v2/correct_nut_entry.py']=g.g.drive.sha(Path(__file__))
    report['source_sha256']['cad/thumb_guide_assembly_v2/reviews/final_service.json']=g.g.drive.sha(HERE/'reviews/final_service.json')
    report.update(completed=True,required_sampled_paths_clear=all(z['clear'] for z in report['paths']),new_required_samples=10,same_required_samples_reused=78,prior_report='reviews/final_service.json',
        reuse_basis='Only two nut paths changed;all other78samples have identical source geometry,transforms,installed/absent rows,obstacles and nominal angles.Roll probes reused without change.',elapsed_correction_s=time.monotonic()-start)
    report['input_hash_guard']=all(g.g.drive.sha(ROOT/z)==h for z,h in report['source_sha256'].items());p.write(out,report);assert report['input_hash_guard']
if __name__=='__main__':main()
