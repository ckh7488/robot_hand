"""Additional upright presentation of the measured, unchanged saved CAD."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import inspect_render as b
n,r,np,ROOT,HERE=b.n,b.r,b.np,b.ROOT,b.HERE


def main():
    raw=n.p.read(HERE/'reviews/baseline.json');assert raw['completed'];output=HERE/'reviews/upright.json';assert not output.exists()
    source=dict(raw['source_sha256']);parts=[]
    for j,z in enumerate(raw['mesh_parts']):
        path=ROOT/z['mesh'];assert r.sha(path)==z['sha256_stl'];color=[.64,.69,.73] if j<397 else [.36,.43,.49]
        if z['name'].startswith('motor_') and 'jaw_with_fixed_bend' in z['name']:color=[.93,.48,.1]
        if 856<=j<862:color=[.93,.48,.1]
        if j>=862:color=[.62,.21,.78] if z['physical_line']==0 else [.1,.48,.78]
        parts.append(dict(name=z['name'],body='palm',mesh_path=path,points=r.read_stl_points(path),color=color,neutral_inverse=np.eye(4)))
    assembly=dict(parts=parts,design=n.g.DESIGN);tiles=[]
    for label,normal in [('X-Y 직교도',[0,0,1]),('Y-Z 직교도',[1,0,0]),('사선',[.8,-.9,.5])]:
        view=r.view_fit([assembly],[dict(angles_deg=[0.]*20)],np.array(normal,dtype=float),1.)
        try:
            assembly['_render_state']=r.make_renderer(assembly,{'v':view},900,b.engine)
            tiles.append((label,r.render_one(assembly,[0.]*20,'v',{'v':view},900,b.engine)[0]))
        finally:
            if '_render_state' in assembly:assembly['_render_state']['renderer'].close();del assembly['_render_state']
    path=HERE/'images/opaque_packaging_upright.png';assert not path.exists()
    r.sheet(tiles,3,900,'현재 전달계 외형 기준: 같은 CAD, 불투명 표시',
        '하드웨어862 + 두 피복·두 줄118분할 / 팔 참고10도 표시 / 새 설계·축소·덮개·길이변경 없음').save(path)
    source['cad/drive_packaging_review_v1/render_upright.py']=r.sha(Path(__file__))
    source['cad/drive_packaging_review_v1/reviews/baseline.json']=r.sha(HERE/'reviews/baseline.json')
    n.a.save(output,dict(completed=True,source_sha256=source,outputs={path.relative_to(ROOT).as_posix():r.sha(path)},
        displayed_parts=980,omitted_parts=[],extra_views_only=True,new_CAD_exports=0,new_native_Boolean_groups=0,
        native_boxes_or_mesh_measurements_recomputed=0,geometry_or_exterior_improvement=False,input_hash_guard=all(r.sha(ROOT/f)==h for f,h in source.items())))


if __name__=='__main__':main()
