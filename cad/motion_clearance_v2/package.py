"""Package the saved 364 STEP solids and independently remeasure changed geometry."""
import argparse, hashlib, importlib.util, json, math, zipfile
from pathlib import Path
import cadquery as cq
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]


def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--work',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--archive',type=Path);a=ap.parse_args()
    if a.output.exists() or a.work.exists() or (a.archive and a.archive.exists()):raise FileExistsError('Use fresh output paths')
    a.work.mkdir(parents=True)
    spec=importlib.util.spec_from_file_location('_package_fk',ROOT/'cad/articulation_drive_v2/verify.py')
    v=importlib.util.module_from_spec(spec);spec.loader.exec_module(v)
    scene=read(HERE/'generated/scene.json');design=read(HERE/'generated/design.json');parts={};hashes={}
    for row in scene['hand']:
        for key,hk in [('step','sha256_step'),('mesh','sha256_stl')]:
            path=(ROOT/row[key]).resolve()
            if not path.is_relative_to(ROOT) or sha(path)!=row[hk]:raise ValueError('Changed/escaping part')
            hashes[row[key]]=row[hk]
        parts[row['name']]=cq.importers.importStep(str(ROOT/row['step'])).val()
        if not parts[row['name']].isValid() or len(parts[row['name']].Solids())!=1:raise ValueError('Invalid saved solid')
    palm=parts['Palm_tapered_chassis']
    old=cq.importers.importStep(str(ROOT/'cad/gesture_hand_v3/generated/parts/Palm_tapered_chassis.step')).val()
    neutral=v.frames(design,np.zeros(20));before_after=[]
    for fi,f in enumerate(design['fingers']):
        q=np.zeros(20);q[fi*4+1]=-20;frames=v.frames(design,q);n=f['name']+'_segment1_segment_shell'
        shell=v.relocate(parts[n],frames[f['name']+'_mcp'],neutral[f['name']+'_mcp'])
        row=dict(finger=f['name'],mcp_deg=-20,
            original_overlap_mm3=float(old.intersect(shell,tol=1e-6).Volume()),
            candidate_overlap_mm3=float(palm.intersect(shell,tol=1e-6).Volume()),
            original_gap_mm=float(old.distance(shell)),candidate_gap_mm=float(palm.distance(shell)))
        if row['original_overlap_mm3']<=.02 or row['candidate_overlap_mm3']>1e-4 or row['candidate_gap_mm']<.5:
            raise ValueError('Before/after effect did not reproduce: '+f['name'])
        before_after.append(row)
    # Measure native cross-sections of the remaining front rim. A sparse section
    # is a geometric witness, not an FEA or a global minimum-thickness proof.
    rim_sections=[]
    for f in design['fingers']:
        x=f['root'][0]
        strip=cq.Solid.makeBox(1,150,150,cq.Vector(x-.5,0,-60))
        s=palm.intersect(strip)
        if not s.isValid():raise ValueError('Invalid section')
        rim_sections.append(dict(finger=f['name'],x_interval_mm=[x-.5,x+.5],section_material_mm3=s.Volume()))
    export=a.work/'MOTION002_hand.step'
    cq.exporters.export(cq.Compound.makeCompound(list(parts.values())),str(export))
    back=cq.importers.importStep(str(export)).val()
    back_solids=back.Solids()
    if len(back_solids)!=364 or any(not s.isValid() for s in back_solids):raise ValueError('Assembly roundtrip invalid')
    volumes=sorted(s.Volume() for s in parts.values());reloaded=sorted(s.Volume() for s in back_solids)
    max_volume_difference=max(abs(x-y) for x,y in zip(volumes,reloaded))
    if max_volume_difference>.001:raise ValueError('Assembly roundtrip volumes changed')
    mesh_path=ROOT/next(r['mesh'] for r in scene['hand'] if r['name']=='Palm_tapered_chassis')
    topology=v.topology(mesh_path)
    if any(topology[k] for k in ['boundary_edges','nonmanifold_edges','degenerate_triangles']):
        raise ValueError('Saved palm STL has invalid topology')
    if a.archive:
        a.archive.parent.mkdir(parents=True,exist_ok=True)
        with zipfile.ZipFile(a.archive,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
            z.write(export,export.name)
    result=dict(completed=True,source_sha256=hashes,part_count=364,
        original_material_outside_candidate_mm3=old.cut(palm).Volume(),candidate_material_outside_original_mm3=palm.cut(old).Volume(),
        before_after=before_after,front_section_witnesses=rim_sections,palm_mesh_topology=topology,
        assembly=dict(valid_solids=len(back_solids),step_sha256=sha(export),step_bytes=export.stat().st_size,
            sorted_volume_max_difference_mm3=max_volume_difference),
        archive=None if not a.archive else dict(file=a.archive.relative_to(ROOT).as_posix(),sha256=sha(a.archive),bytes=a.archive.stat().st_size),
        source_inputs_unchanged=all(sha(ROOT/k)==h for k,h in hashes.items()),
        manufacturing_release=False,strength_approved=False,
        limitation='Root support collisions remain; section witnesses do not prove stiffness, fatigue, manufacturing fits or complete assembly.')
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='source_sha256'},indent=2),flush=True)


if __name__=='__main__':main()
