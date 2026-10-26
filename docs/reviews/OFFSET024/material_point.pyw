import sys
from pathlib import Path
import numpy as np
sys.dont_write_bytecode=True
root=Path(sys.argv[1]);out=Path(sys.argv[2]);assert not out.exists();sys.path.insert(0,str(root/'cad/thumb_sheath_terminal_v2'))
import selected_model as m
fresh=next(z['shape'] for z in m.build()[0] if z['name']=='distal_housing_keeper_1');row=next(z for z in m.p.read(root/'cad/thumb_sheath_terminal_v2/selected/generated/parts/scene.json')['parts'] if z['name']=='distal_housing_keeper_1');saved=m.c.load([row])[0]['shape'];pt=np.array([99.28777459522642,71.2950282308043,41.866736144525426]);normal=np.array(fresh.Faces()[7].normalAt(m.cq.Vector(*pt)).toTuple());rows=[]
for d in [.01,.05]:
    points=[pt-normal*d,pt+normal*d];a=[bool(fresh.isInside(m.cq.Vector(*x),1e-6)) for x in points];b=[bool(saved.isInside(m.cq.Vector(*x),1e-6)) for x in points];rows.append(dict(offset_mm=d,points_world_mm=[x.tolist() for x in points],source_inside=a,STEP_inside=b,matched=a==b and a[0]!=a[1]))
m.p.write(out,dict(completed=True,point_world_mm=pt.tolist(),normal=normal.tolist(),samples=rows,matched=all(z['matched'] for z in rows),scope='Two-sided material classification only at one ambiguous torus point.Native point-distance and initial false results remain preserved.'));print(rows,flush=True)
