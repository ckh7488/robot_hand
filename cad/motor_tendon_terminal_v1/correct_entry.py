"""Change only bracket approach direction; same staged missing/installed rows."""
import sys,copy,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import terminal_selected as model
g,c,p,np,cq,HERE,ROOT=model.g,model.c,model.p,model.np,model.cq,model.HERE,model.ROOT
check=model.drive.module('_terminal022_entry_inputs',Path(__file__).resolve().parent/'build_check.py');check.model=model;check.HERE=HERE
def main():
    out=HERE/'reviews/selected_service.json';assert not out.exists();start=time.monotonic();old=p.read(HERE/'reviews/service.json');assert old['completed']
    assert all(model.drive.sha(ROOT/z)==h for z,h in old['source_sha256'].items())
    static=p.read(HERE/'reviews/check.json');data,retained,guide=check.inputs(static['design']);parts=c.load(p.read(ROOT/static['scene'])['parts']);by={z['name']:z for z in parts}
    hand=[dict(name=z['name'],shape=data[4][z['name']]) for z in data[3]['hand'] if z['name'] not in ['thumb_segment2_dark_faceplate_clamp1_bolt','thumb_segment2_dark_faceplate_clamp2_bolt']]+guide
    report=copy.deepcopy(old);path=report['paths'][0];assert path['id']=='frame_from_rear'
    obstacles=[z for z in retained if z['name'] not in path['not_installed_packaging']+path['moving']]+[z for z in data[6] if z['name'] not in path['not_connected_electrical']]+data[7]+hand+[by[n] for n in path['already_installed_new']]
    samples=[]
    for dz in [-25,-18,-12,-6,0]:
        vector=[0,0,dz];moved=[dict(name=n,shape=by[n]['shape'].translate(vector)) for n in path['moving']];samples.append(dict(translation_mm=vector,native=c.material(moved,obstacles)))
    path.update(id='frame_from_below',samples=samples,clear=all(model.drive.clear(z['native']) for z in samples),scope=path['scope']+' Approach frombelow25mm instead of rear35mm;all actual geometry and stage obstacles unchanged.')
    report['source_sha256']['cad/motor_tendon_terminal_v1/correct_entry.py']=model.drive.sha(Path(__file__));report['source_sha256']['cad/motor_tendon_terminal_v1/selected/reviews/service.json']=model.drive.sha(HERE/'reviews/service.json')
    report.update(required_sampled_paths_clear=all(z['clear'] for z in report['paths']),new_required_service_samples=5,same_required_service_samples_reused=35,elapsed_entry_correction_s=time.monotonic()-start,prior_report='selected/reviews/service.json')
    report['input_hash_guard']=all(model.drive.sha(ROOT/z)==h for z,h in report['source_sha256'].items());p.write(out,report);assert report['input_hash_guard'];print('corrected below',path['clear'],flush=True)
if __name__=='__main__':main()
