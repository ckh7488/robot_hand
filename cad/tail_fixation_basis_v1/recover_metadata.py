"""Recover overwritten scene lists from saved files; no CAD/native recomputation."""
import sys,ast,shutil
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import basis as b
import support as s


def functions(path):
    tree=ast.parse(path.read_text(encoding='utf-8-sig'))
    return {z.name:ast.dump(z,include_attributes=False) for z in tree.body if isinstance(z,ast.FunctionDef) and z.name!='export'}


def main():
    out=b.HERE/'reviews/metadata_recovery.json';assert not out.exists()
    initial=b.HERE/'history/basis_initial.py.txt';assert functions(initial)==functions(b.HERE/'basis.py')
    raw=b.p.read(b.HERE/'reviews/check.json');sup=b.p.read(b.HERE/'reviews/support.json')
    assert raw['completed'] and sup['completed']
    assert b.m.drive.sha(initial)==raw['source_sha256']['cad/tail_fixation_basis_v1/basis.py']==sup['source_sha256']['cad/tail_fixation_basis_v1/basis.py']
    case,source,rows=b.context();lookup={z['name']:z for z in rows};saved=[]
    geometry={f.relative_to(b.ROOT).as_posix():b.m.drive.sha(f) for f in (b.HERE/'generated').rglob('*') if f.suffix in ['.step','.stl']}
    def restore(scene,meta):
        file=b.ROOT/scene;old=b.p.read(file);assert isinstance(old['parts'],str) and old['parts']==scene
        dest=b.HERE/'history/scene_lists_before_recovery'/file.parent.name/'scene.json';dest.parent.mkdir(parents=True,exist_ok=False);shutil.copyfile(file,dest)
        result=[]
        for row in meta:
            name=row['name'];step=file.parent/(name+'.step');mesh=step.with_suffix('.stl');assert step.is_file() and mesh.is_file()
            result.append(dict(**{k:v for k,v in row.items() if k not in ['shape','step','mesh','sha256_step','sha256_stl']},
                step=step.relative_to(b.ROOT).as_posix(),mesh=mesh.relative_to(b.ROOT).as_posix(),sha256_step=b.m.drive.sha(step),sha256_stl=b.m.drive.sha(mesh)))
        b.a.save(file,dict(parts=result,inspection_only=True,metadata_recovered_without_CAD_or_native_replay=True))
        saved.append(dict(scene=scene,old_scene_sha256=b.m.drive.sha(dest),new_scene_sha256=b.m.drive.sha(file),rows=len(result)))
    specs=b.specifications(case)
    def probe_meta(spec):return dict(name=spec['id']+'_actual_tail_material',body='palm',installed_part_candidate=False,kind='material_subset',physical_line=spec['line'],color=[.97,.45,.12])
    restore(raw['contact_material_scene'],[probe_meta(z) for z in specs])
    restore(sup['scene'],[probe_meta(z) for z in s.specifications(case)])
    for path in raw['paths']:
        for rec in path['samples']:
            if 'scene' in rec:
                row=dict(lookup[path['moving']],assembly_translation_mm=(b.np.array(path['axis'])*rec['outward_distance_mm']).tolist())
                restore(rec['scene'],[row])
    assert all(b.m.drive.sha(b.ROOT/f)==h for f,h in geometry.items())
    b.a.save(out,dict(completed=True,source_sha256={'cad/tail_fixation_basis_v1/basis.py':b.m.drive.sha(b.HERE/'basis.py'),
        'cad/tail_fixation_basis_v1/history/basis_initial.py.txt':b.m.drive.sha(initial),
        'cad/tail_fixation_basis_v1/recover_metadata.py':b.m.drive.sha(Path(__file__))},scenes=saved,
        geometry_sha256_preserved=geometry,all_non_export_function_AST_identical=True,
        native_material_distance_or_classifier_queries_recomputed=0,CAD_SOLIDs_or_exports_recomputed=0,
        original_native_reports_unchanged=True,reason='Export helper already wrote scene metadata and returned its path; wrapper erroneously replaced parts with that path. Correct only wrapper/list metadata.'))
    print('restored_scenes',len(saved),'geometry_hashes_preserved',len(geometry),'native_replays',0,flush=True)


if __name__=='__main__':main()
