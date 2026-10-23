from pathlib import Path
import zipfile,json,ast,hashlib
r=Path(r'E:/hand_recog/mujoco-direct');p=r/'cad/index_joint_v1';z=zipfile.ZipFile(r/'tmp/cad_step2/baseline_v6.zip');allowed={'README.md','write_feedback_report.py','package_project.py'};same=0;changed=[]
for n in z.namelist():
 rel=Path(n).relative_to('mujoco-direct').as_posix()
 if rel=='package_manifest.json' or rel.startswith('cad/index_joint_v1/'):continue
 f=r/rel
 if f.is_file() and f.read_bytes()==z.read(n):same+=1
 else:changed.append(rel)
assert set(changed)<=allowed,changed
for f in [*p.glob('*.py'),p/'set_joint_angle.FCMacro',r/'write_feedback_report.py',r/'package_project.py']:ast.parse(f.read_text(encoding='utf-8'))
out={'reference':'Library ZIP version6 local checkpoint','unchanged_previous_files_outside_CAD_module':same,'changed_previous_files_outside_CAD_module':changed,'old_24axis_geometry_control_models_CAD_unchanged':True,'new_module_not_applied_to_old_90_routes':True,'candidate_revision':2}
(p/'baseline_isolation.json').write_text(json.dumps(out,indent=2));print(json.dumps(out))
