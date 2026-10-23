"""Windowless CAD audit launcher. Logs live under audit_runs/; no terminals or solver launched."""
import argparse, json, os, subprocess, sys, datetime, traceback
from pathlib import Path
P=Path(__file__).resolve().parent
D=P/'cad'/'modular_roots_v9'
p=argparse.ArgumentParser();p.add_argument('--task',choices=['quick','cad','viewer'],default='quick');p.add_argument('--analysis-python');p.add_argument('--freecad-python');a=p.parse_args()
analysis=Path(a.analysis_python) if a.analysis_python else P.parent/'mujoco-lab'/'.venv'/'Scripts'/'pythonw.exe'
if not analysis.exists():analysis=Path(sys.executable)
freecad=Path(a.freecad_python or 'C:/Program Files/FreeCAD 1.0/bin/pythonw.exe')
run=P/'audit_runs'/datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ');run.mkdir(parents=True,exist_ok=True)
state={'task':a.task,'pid':os.getpid(),'state':'running','commands':[],'console_policy':'pythonw plus CREATE_NO_WINDOW and SW_HIDE; no FreeCADCmd/GUI or solver workers'}
def save(): (run/'status.json').write_text(json.dumps(state,indent=2),encoding='utf-8')
def launch(exe,args,label,wait=True):
 if not Path(exe).is_file():raise FileNotFoundError(str(exe))
 si=None;flags=0
 if os.name=='nt':
  si=subprocess.STARTUPINFO();si.dwFlags|=subprocess.STARTF_USESHOWWINDOW;si.wShowWindow=subprocess.SW_HIDE;flags=subprocess.CREATE_NO_WINDOW
 with (run/(label+'.log')).open('w',encoding='utf-8') as log:
  child=subprocess.Popen([str(exe),*map(str,args)],cwd=P,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,creationflags=flags,startupinfo=si)
  row={'label':label,'executable':str(exe),'arguments':list(map(str,args)),'pid':child.pid};state['commands'].append(row);save()
  if wait:
   row['returncode']=child.wait();save()
   if row['returncode']!=0:raise RuntimeError(label+' failed; see log')
try:
 save()
 if a.task=='viewer':launch(analysis,[D/'cad_viewer.py'],'viewer',False)
 else:
  launch(analysis,[P/'verify_audit_evidence.py'],'evidence')
  launch(analysis,[D/'verify_cad_viewer.py'],'viewer_alignment')
  launch(analysis,[D/'scan_mesh_motion.py','named'],'six_poses')
  if a.task=='cad':
   for s in ['review_geometry.py','review_arm_and_roundtrip.py','review_hand_fasteners.py','verify_subset_transfer.py']:
    launch(freecad,[D/s],s.removesuffix('.py'))
   for s in ['review_all_cartridge_nuts.py','review_module_installation.py']:
    launch(analysis,[D/s],s.removesuffix('.py'))
 state['state']='completed';save()
except Exception as e:
 state['state']='failed';state['error']=str(e);save();(run/'launcher_error.txt').write_text(traceback.format_exc(),encoding='utf-8');sys.exit(1)
