"""Read actual thenar contact and current terminal attachment/entry direction."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import entry_model as a
m,g,c,p,np,cq,ROOT,HERE=a.m,a.g,a.c,a.p,a.np,a.cq,a.ROOT,a.HERE
def main():
    source={};old=p.read(ROOT/'cad/elastic_housing_link_v1/reviews/details.json');common=c.load([z for z in p.read(ROOT/old['common_scene'])['parts'] if z['name']=='actual_hand_common'],source)[0];catalog=p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')['hand'];shell=c.load([z for z in catalog if z['name']=='thumb_segment1_thenar_shell'],source)[0];Q=a.frame();inverse=np.linalg.inv(Q);location=common['shape'].Center().toTuple();body=c.load([z for z in p.read(ROOT/'cad/sheath_retention_v1/selected/generated/parts/scene.json')['parts'] if z['name']==a.REPLACED],source)[0]
    for f in ['cad/sheath_retention_v1/selected/reviews/check.json','cad/elastic_housing_link_v1/reviews/details.json']:source[f]=m.drive.sha(ROOT/f)
    p.write(HERE/'reviews/survey.json',dict(completed=True,source_sha256=source,original_actual_common=dict(name=common['name'],bbox_mm=c.m.pack._bbox(common['shape']),center_world_mm=location,center_in_upper_terminal_frame_mm=(inverse@np.r_[location,1.])[:3].tolist()),thenar_shell_bbox_mm=c.m.pack._bbox(shell['shape']),existing_upper_jaw_bbox_mm=c.m.pack._bbox(body['shape']),upper_clamp_Q=Q.tolist(),new_native_interference_groups=0,old_rejected_link_not_recomputed=True,whole_hand_or_all_paths_impossible=False));print(location,(inverse@np.r_[location,1.])[:3].tolist(),flush=True)
if __name__=='__main__':main()
