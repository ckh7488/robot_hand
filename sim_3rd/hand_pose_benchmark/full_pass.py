"""Run all locally available training images; preserve evaluation separately.

This is inference only. No weights are updated. Child processes are hidden on
Windows and all outputs remain outside the Git checkout.
"""
import json,os,pathlib,sqlite3,subprocess,sys,time
from datetime import datetime,timezone
from run import write_progress

ROOT=pathlib.Path(r'E:\hand_recog\vision-results\20261026')
HERE=pathlib.Path(__file__).resolve().parent
PYTHON=pathlib.Path(sys.executable).with_name('pythonw.exe' if os.name=='nt' else 'python')
FLAGS=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0

def count(name):
    p=ROOT/f'{name}_train.sqlite'
    if not p.exists():return 0
    try:
        db=sqlite3.connect(f'file:{p.as_posix()}?mode=ro',uri=True);n=db.execute('SELECT count(*) FROM results').fetchone()[0];db.close();return n
    except sqlite3.Error:return 0

def error_count(name):
    p=ROOT/f'{name}_train.sqlite'
    if not p.exists():return 0
    try:
        db=sqlite3.connect(f'file:{p.as_posix()}?mode=ro',uri=True);n=db.execute("SELECT count(*) FROM results WHERE status!='complete'").fetchone()[0];db.close();return n
    except sqlite3.Error:return -1

def main():
    env=os.environ.copy();env['TEMP']=env['TMP']=r'E:\hand_recog\vision-runtime\tmp';env['PYTHONIOENCODING']='utf-8'
    children={};logs=[]
    for model,device in [('mediapipe',0),('wilor',1),('hamer',0)]:
        stdout=(ROOT/f'{model}-train.stdout.log').open('a',encoding='utf8');stderr=(ROOT/f'{model}-train.stderr.log').open('a',encoding='utf8');logs.extend([stdout,stderr])
        p=subprocess.Popen([str(PYTHON),'-B','-u',str(HERE/'run.py'),'--model',model,'--phase','train','--device',str(device)],env=env,stdout=stdout,stderr=stderr,creationflags=FLAGS)
        children[model]=p
    (ROOT/'full_pass_jobs.json').write_text(json.dumps({k:dict(pid=p.pid) for k,p in children.items()},indent=2),encoding='utf8')
    while True:
        states={k:dict(processed=count(k),total=171498,pid=p.pid,returncode=p.poll()) for k,p in children.items()}
        message='로컬 학습 분할 171,498장 전체 추론 진행 · '+' · '.join(f'{k}: {s["processed"]:,}/{s["total"]:,}' for k,s in states.items())
        status=dict(updated_at=datetime.now(timezone.utc).isoformat(),message=message,models=states)
        write_progress(ROOT/'report/batch_status.json',status)
        if all(p.poll() is not None for p in children.values()):break
        time.sleep(20)
    for f in logs:f.close()
    for model in ['wilor','hamer']:
        subprocess.run([str(PYTHON),'-B',str(HERE/'audit_regions.py'),'--model',model,'--phase','train'],env=env,creationflags=FLAGS,check=True)
    subprocess.run([str(PYTHON),'-B',str(HERE/'report.py'),'--phase','train'],env=env,creationflags=FLAGS,check=True)
    states={k:dict(processed=count(k),errors=error_count(k),total=171498,pid=p.pid,returncode=p.returncode) for k,p in children.items()}
    complete=all(s['processed']==171498 and s['returncode']==0 and s['errors']==0 for s in states.values())
    write_progress(ROOT/'report/batch_status.json',dict(updated_at=datetime.now(timezone.utc).isoformat(),message='학습 분할 전체 추론 완료 · 별도 보고서: report_train/hand_pose_review.html' if complete else '학습 분할 일부 미완료 · 저장된 입력부터 재개 가능',models=states))

if __name__=='__main__':main()
