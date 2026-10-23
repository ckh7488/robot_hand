from pathlib import Path
import zipfile,json,hashlib
p=Path(r'E:/hand_recog/mujoco-direct-hand.zip');z=zipfile.ZipFile(p);root='mujoco-direct/';d=root+'cad/index_joint_v1/'
required=['wire_harness.py','joint_hardware_spec.json','cable_validation.json','verify_guide_and_connector.py','whole_assembly_overview.png','whole_hand_closeup.png','index_root_joint.FCStd','index_root_joint.step','FPX330_S101.step','Cable_guide.step','sources/cable_fastener/EH_catalog.pdf']
assert all(d+n in z.namelist() for n in required)
m=json.loads(z.read(root+'package_manifest.json'))['sha256']
for n in required:assert hashlib.sha256(z.read(d+n)).hexdigest()==m['cad/index_joint_v1/'+n]
cv=json.loads(z.read(d+'cable_validation.json'));assert cv['wire_continuous_passed'] and not cv['connector_case_fit_closed'] and not cv['manufacturing_release']
print('ZIP verified:',len(z.namelist()),'files, complete CAD/harness/images and explicit unresolved connector fit')
