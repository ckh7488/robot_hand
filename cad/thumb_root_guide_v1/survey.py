import sys
from pathlib import Path
sys.dont_write_bytecode=True
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
sys.path.insert(0,str(ROOT/'cad/thumb_tendon_route_v1'))
import route_model as g
c,b,m,p,cq,np=g.c,g.b,g.m,g.p,g.cq,g.np

def main():
    out=HERE/'reviews/survey.json';assert not out.exists()
    cat=p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json');source={};rows=[z for z in cat['hand'] if z['body'] in ['thumb_opposition','thumb_mcp']]
    frame=g.v.frames(g.DESIGN,np.zeros(20))['thumb_ip'];O,R=frame;inverse=np.eye(4);inverse[:3,:3]=R.T;inverse[:3,3]=-R.T@O
    local=[]
    for z in c.load(rows,source):
        s=m.pack._transform(z['shape'],inverse)
        local.append(dict(name=z['name'],body=z['body'],bbox_mm=m.pack._bbox(s)))
    r=p.read(ROOT/'cad/gesture_hand_v3/generated/build_report.json')['fabrication_repairs']['skin_fasteners']
    fasteners=[z for z in r if z['id'].startswith('thumb_segment2_dark_faceplate')]
    p.write(out,dict(completed=True,source_sha256=source,local_frame='neutral thumb_ip;all geometry mm',parts=local,fasteners=fasteners,
        frame_origin=O.tolist(),frame_rotation=R.tolist()))
    print(local,flush=True)
if __name__=='__main__':main()
