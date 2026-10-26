"""Small native2part posture map, not a root-cover CAD modification."""
import importlib.util
import json
from pathlib import Path
import sys

import cadquery as cq
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_root_map_native',ROOT/'cad/gesture_hand_v3/audit_gestures.py')
native=importlib.util.module_from_spec(spec);spec.loader.exec_module(native);v=native.load_v2(ROOT)


def main():
    out=HERE/'generated/root_posture_map.json'
    if out.exists():raise FileExistsError('Preserve earlier map')
    cat=native.read_json(ROOT/'cad/functional_motion_v3/generated/catalog.json');scene=native.read_json(ROOT/'cad/motion_clearance_v2/generated/assembly_catalog.json')
    rows={r['name']:r for r in scene['hand']};names=['Palm_shaped_front_cover','index_segment1_segment_shell']
    inputs={p:native.digest(ROOT/p) for p in ['cad/posture_correction_v1/root_map.py','cad/gesture_hand_v3/audit_gestures.py','cad/articulation_drive_v2/verify.py','cad/functional_motion_v3/generated/catalog.json']}
    shapes={}
    for name in names:
        row=rows[name]
        if native.digest(ROOT/row['step'])!=row['sha256_step']:raise ValueError('Changed STEP')
        inputs[row['step']]=row['sha256_step'];shapes[name]=cq.importers.importStep(str(ROOT/row['step'])).val()
    spreads=[-27.,-20.,-15.,-10.,-5.,0.,5.,10.,15.,20.,27.];flex=[-35.,-20.,0.,20.,40.,55.,70.,85.,100.]
    neutral=v.frames(cat['design'],np.zeros(20));values=[]
    for spread in spreads:
        line=[]
        for mcp in flex:
            q=np.zeros(20);q[:2]=[spread,mcp];f=v.frames(cat['design'],q)
            moved=v.relocate(shapes[names[1]],f['index_mcp'],neutral['index_mcp'])
            volume=float(shapes[names[0]].intersect(moved,tol=1e-6).Volume())
            line.append(volume)
        values.append(line)
    native.write_json(out,dict(completed=True,source_sha256=inputs,spread_deg=spreads,mcp_deg=flex,overlap_volume_mm3=values,
        input_hash_guard=all(native.digest(ROOT/p)==h for p,h in inputs.items()),
        scope='Exact saved index proximal shell versus palm front cover only. Bilinear interpolation is optimization guidance; final364part audit remains mandatory.'))


if __name__=='__main__':main()
