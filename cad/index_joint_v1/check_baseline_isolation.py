from pathlib import Path
import zipfile,hashlib,json,ast
r=Path(r'E:/hand_recog/mujoco-direct');p=r/'cad/index_joint_v1'
ex={'README.md','package_project.py','write_feedback_report.py'}
old=zipfile.ZipFile(r.parent/'mujoco-direct-hand.zip');changes=[];same=0
for n in old.namelist():
 rel=Path(n).relative_to('mujoco-direct').as_posix()
 if rel=='package_manifest.json':continue
 path=r/rel
 if path.is_file() and path.read_bytes()==old.read(n):same+=1
 else:changes.append(rel)
assert set(changes)<=ex,changes
for f in [p/'build_joint.py',p/'validate_joint.py',p/'render_joint.py',p/'verify_joint_roundtrip.py',p/'set_joint_angle.FCMacro',r/'package_project.py',r/'write_feedback_report.py']:ast.parse(f.read_text(encoding='utf-8'))
out={'unchanged_previous_files':same,'changed_previous_files':changes,'allowed_changes':sorted(ex),'old_24axis_geometry_control_models_CAD_unchanged':True,'new_CAD_module':'cad/index_joint_v1','new_module_not_applied_to_old_90_routes':True}
(p/'baseline_isolation.json').write_text(json.dumps(out,indent=2));print(out)
