"""Actual prototype intersections expressed in the attached upper socket frame."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import offset_model as m
g,c,p,np,cq,ROOT,HERE=m.g,m.c,m.p,m.np,m.cq,m.ROOT,m.HERE
def matrix(frame):
    O,R=frame;T=np.eye(4);T[:3,:3]=R;T[:3,3]=O;return T
def main():
    report=p.read(HERE/'reviews/static.json');cat=p.read(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json');meta=p.read(ROOT/report['scene'])['parts'];q=report['cases'][-1]['angles_deg'];v=g.g.v
    rootmeta=next(z for z in cat['hand'] if z['name']=='thumb_segment1_thenar_shell');root=c.load([rootmeta])[0];guide=c.load(p.read(ROOT/'cad/thumb_guide_assembly_v2/generated/parts/scene.json')['parts']);parts=c.load(meta);keeper=next(z for z in parts if z['name']=='distal_housing_keeper_1')
    f=v.frames(g.DESIGN,q);zero=v.frames(g.DESIGN,np.zeros(20));Q=g.original.layout()[1]['socket_transform'];S=matrix(f['thumb_mcp'])@np.linalg.inv(matrix(zero['thumb_mcp']))@g.original.WORLD@Q
    tests=[];ev=HERE/'generated/prototype_common';ev.mkdir(parents=True,exist_ok=False)
    for other in [next(z for z in guide if z['name']=='paired_liner_carrier'),next(z for z in guide if z['name']=='flanged_liner_1'),dict(root,body=rootmeta['body'])]:
        a=v.relocate(keeper['shape'],f['thumb_mcp'],zero['thumb_mcp']);b=v.relocate(other['shape'],f[other.get('body','thumb_mcp')],zero[other.get('body','thumb_mcp')]);common=a.intersect(b,tol=1e-6);local=g.original.transform(common,np.linalg.inv(S));st=ev/(other['name']+'.step');cq.exporters.export(common,str(st));tests.append(dict(part=other['name'],volume_mm3=common.Volume(),socket_bbox_mm=c.m.pack._bbox(local),world_step=st.relative_to(ROOT).as_posix()))
    p.write(HERE/'reviews/locations.json',dict(completed=True,angles_deg=q,upper_socket_world=S.tolist(),actual_intersections=tests,first_bore_cut_error='Prototype copied keeper channel cut stopped at u=-3.99 instead of+4.01.Missing downstream bore corrected separately;not accepted hand failure.'))
    print([(z['part'],z['socket_bbox_mm']) for z in tests],flush=True)
if __name__=='__main__':main()
