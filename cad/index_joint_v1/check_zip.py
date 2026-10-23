from pathlib import Path
import zipfile,json
r=Path(r'E:/hand_recog');z=zipfile.ZipFile(r/'mujoco-direct-hand.zip');p='mujoco-direct/cad/index_joint_v1/'
required=['README.md','index_root_joint.FCStd','index_root_joint.step','set_joint_angle.FCMacro','cad_validation.json','cad_roundtrip.json','sources/XL330.stp','sources/FPX330-H101.step','sources/FPX330-S101.step','assembly_preview.png']
assert all(p+n in z.namelist() for n in required)
assert not any('index_joint_v1/tmp/' in n or 'proto_meshes' in n for n in z.namelist())
print('ZIP',len(z.namelist()),'files; CAD payload',len([n for n in z.namelist() if n.startswith(p)]),'files; all required present')
