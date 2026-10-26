"""Correct only the obstructed upper keeper approach; keep original failures."""
import sys,time,copy
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import guide_candidate3 as g
check=g.g.drive.module('_guide020_corrected_service_inputs',Path(__file__).resolve().parent/'build_check.py')
check.g=g;check.HERE=g.HERE
c,p,np,cq,ROOT=g.c,g.p,g.np,g.cq,g.ROOT

def main():
    out=g.HERE/'reviews/final_service.json';assert not out.exists();start=time.monotonic();old=p.read(g.HERE/'reviews/service_scopes.json');assert old['completed'] and old['input_hash_guard']
    assert all(g.g.drive.sha(ROOT/z)==h for z,h in old['source_sha256'].items())
    report=copy.deepcopy(old);label='liner_flange_keeper_1_forward_then_side_then_seat';prior=next(z for z in old['paths'] if z['id']==label)
    failed=[z['id'] for z in old['paths'] if z['id']!='initial_upper_keeper_straight_side' and not z['clear']]
    assert label in failed,failed
    parts,design=g.build();data=check.inputs(design);v,cat,scene,hand=data[1:5];neutral=v.frames(cat['design'],np.zeros(20));by={z['name']:z for z in parts};q=prior['hand_angles_deg'];frames=v.frames(cat['design'],q)
    def pose(s):return v.relocate(g.original.transform(s,g.original.WORLD),frames['thumb_mcp'],neutral['thumb_mcp'])
    obstacles=[dict(name=z['name'],shape=v.relocate(hand[z['name']],frames[z['body']],neutral[z['body']])) for z in scene['hand'] if z['name'] not in design['replaced_hand_parts']]
    names=prior['moving'];obstacles+=data[5]+data[6]+data[7]+[dict(name=n,shape=pose(by[n]['local_shape'])) for n in prior['already_installed_new_parts'] if n not in names]
    axis=g.original.layout()[1]['socket_transform'][:3,1]
    def translation(vector):
        T=np.eye(4);T[:3,3]=vector;return T
    transforms=[translation(np.array([x,0,0])+axis*1.5) for x in [12,9,6,3,0]]+[translation(axis*y) for y in [1.125,.75,.375,0]]
    fresh=copy.deepcopy(prior);fresh['samples']=[]
    for T in transforms:
        moving=[dict(name=n,shape=pose(g.original.transform(by[n]['local_shape'],T))) for n in names]
        fresh['samples'].append(dict(transform_local=T.tolist(),native=c.material(moving,obstacles)))
    fresh['clear']=all(g.g.drive.clear(z['native']) for z in fresh['samples']);fresh['scope']='Same geometry/parts/manualIP-15deg.Upper keeper1.5mm in front during side entry,then axial seating;6mm alternative was obstructed by the distal cover fastener/foot.Not a hand or guide dimension change.'
    report['paths']=[fresh if z['id']==label else z for z in report['paths']]
    report['source_sha256']['cad/thumb_root_guide_v1/service_correction.py']=g.g.drive.sha(Path(__file__))
    report['source_sha256']['cad/thumb_root_guide_v1/selected2/reviews/service_scopes.json']=g.g.drive.sha(g.HERE/'reviews/service_scopes.json')
    report['original_service_report']='cad/thumb_root_guide_v1/selected2/reviews/service_scopes.json';report['original_required_paths_clear']=old['required_sampled_paths_clear']
    report['new_required_samples']=len(fresh['samples']);report['same_required_samples_reused']=old['required_samples']-len(fresh['samples'])
    report['required_sampled_paths_clear']=all(z['clear'] for z in report['paths'] if z['id']!='initial_upper_keeper_straight_side')
    report['input_hash_guard']=all(g.g.drive.sha(ROOT/z)==h for z,h in report['source_sha256'].items());report['correction_elapsed_s']=time.monotonic()-start
    p.write(out,report);assert report['input_hash_guard'];print('corrected upper keeper',fresh['clear'],'required all',report['required_sampled_paths_clear'],flush=True)
if __name__=='__main__':main()
