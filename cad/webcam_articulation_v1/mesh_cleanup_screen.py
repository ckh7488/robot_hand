"""Inspect exported mesh defects without modifying the source STL files."""
import json
import hashlib
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent


def topology(triangles):
    vertices,index=np.unique(triangles.reshape(-1,3),axis=0,return_inverse=True)
    index=index.reshape(-1,3)
    edges=np.concatenate([index[:,[0,1]],index[:,[1,2]],index[:,[2,0]]])
    edges_sorted=np.sort(edges,axis=1)
    _,inverse,count=np.unique(edges_sorted,axis=0,return_inverse=True,return_counts=True)
    direction=np.where(edges[:,0]<edges[:,1],1,-1)
    winding=np.bincount(inverse,weights=direction)
    return dict(triangles=len(triangles),boundary_edges=int(np.sum(count==1)),
        nonmanifold_edges=int(np.sum(count>2)),inconsistent_two_face_edges=int(np.sum((count==2)&(winding!=0))))


def main():
    scene=json.loads((HERE/'generated/scene.json').read_text())['hand']
    dtype=np.dtype([('normal','<f4',(3,)),('v','<f4',(3,3)),('attribute','<u2')])
    rows=[]
    for part in scene:
        raw=(HERE/part['mesh']).read_bytes();assert hashlib.sha256(raw).hexdigest()==part['sha256_stl']
        triangles=np.frombuffer(raw[84:],dtype=dtype)['v'].astype(float)
        area2=np.linalg.norm(np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]),axis=1)
        good=area2>=1e-10
        rows.append(dict(name=part['name'],sha256_stl=part['sha256_stl'],removed_degenerate_count=int(np.sum(~good)),
            as_exported=topology(triangles),diagnostic_removal_only=topology(triangles[good])))
    report=dict(source_files_modified=False,method='Exact float32 coordinate welding; diagnostic removal of triangles with twice-area <1e-10mm2; no saved repaired meshes',parts=rows)
    output=HERE/'generated/manufacturing_audit/mesh_cleanup.json'
    output.write_text(json.dumps(report,indent=2))
    print(json.dumps([r for r in rows if r['removed_degenerate_count']],indent=2),flush=True)


if __name__=='__main__':main()
