"""Preserve the failed full-turn study and export its unchanged CAD evidence."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import model as g
c,b,m,p,cq,np,HERE,ROOT=g.c,g.b,g.m,g.p,g.cq,g.np,g.HERE,g.ROOT


def main():
    raw=p.read(HERE/'reviews/check.json');assert len(raw['rotations'])==37
    assert all(g.sha(ROOT/z)==h for z,h in raw['source_sha256'].items())
    assert any(z['lead']=='thumb_ip_two_tail_spool' for r in raw['rotations'] for z in r['packaging']['overlaps'])
    rows,design=g.build();mount=p.read(ROOT/'cad/palm_motor_mount_v1/selected3/generated/scene.json')
    meta=mount['new_parts']+mount['retained_parts'];replaced=[z for z in meta if z['name'] in [g.NAME+'_rotor',g.NAME+'_spool']];retained=[z for z in meta if z not in replaced]
    root=HERE/'generated/parts';root.mkdir(parents=True,exist_ok=False);saved=[]
    for z in rows:
        st=root/(z['name']+'.step');me=root/(z['name']+'.stl');cq.exporters.export(z['shape'],str(st));cq.exporters.export(z['shape'],str(me),tolerance=.06,angularTolerance=.1)
        saved.append(dict(**{k:v for k,v in z.items() if k not in ['shape','local_shape']},step=st.relative_to(ROOT).as_posix(),mesh=me.relative_to(ROOT).as_posix(),sha256_step=g.sha(st),sha256_stl=g.sha(me),bbox_mm=m.pack._bbox(z['shape'])))
    p.write(HERE/'generated/scene.json',dict(parts=saved,retained_parts=retained,replaced_parts=replaced,installed=False,failed_initial_candidate=True,hand_geometry_unchanged=True,motor_positions_unchanged=True))
    p.write(HERE/'reviews/initial_export.json',dict(completed=True,CAD_export_only=True,original_collision_study_completed=raw['completed'],original_rotations_completed=37,
        original_spool_install_samples=len(raw.get('spool_install',[])),original_screw_install_samples=len(raw.get('screw_install',[])),
        source_hashes_verified=True,reason='Original candidate already blocked in a full turn.Stopped later redundant assembly computation;raw completed=false partial report and source code preserved.Exported unchanged first CAD for failed-pair replay and actual common-shape illustration.',whole_design_passed=False))


if __name__=='__main__':main()
