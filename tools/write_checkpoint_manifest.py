"""Write the current checkpoint's content manifest without modifying frozen CAD evidence."""
import argparse,hashlib,json,subprocess,datetime
from pathlib import Path
P=Path(__file__).resolve().parents[1];ap=argparse.ArgumentParser();ap.add_argument('checkpoint_id');a=ap.parse_args()
assert a.checkpoint_id.startswith('CP') and a.checkpoint_id[2:].isdigit()
current=json.loads((P/'CHECKPOINT.json').read_text(encoding='utf-8'))
assert current['id']==a.checkpoint_id,'Only the current uncommitted checkpoint may be written'
manifest='checkpoints/'+a.checkpoint_id+'.manifest.json'
exists_in_head=subprocess.run(['git','cat-file','-e','HEAD:'+manifest],cwd=P,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode==0
assert not exists_in_head,'Committed checkpoint manifests are immutable; create the next checkpoint'
# Existing snapshots remain immutable. Only the requested current manifest is regenerated.
paths=subprocess.check_output(['git','ls-files','--cached','--others','--exclude-standard','-z'],cwd=P).decode('utf-8').split('\0');files={}
for relative in sorted(set(paths)-{'',manifest,'CHECKPOINT.json','PROGRESS.md'}):
 f=(P/relative).resolve();assert f.is_relative_to(P) and f.is_file(),relative
 b=f.read_bytes();files[relative]={'bytes':len(b),'sha256':hashlib.sha256(b).hexdigest()}
out={'id':a.checkpoint_id,'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'scope':'All tracked and non-ignored candidate files, excluding this manifest, CHECKPOINT.json and the append-only PM journal PROGRESS.md. Historical snapshots remain included. Review commits after this checkpoint are separate evidence.','files':files}
(P/manifest).write_text(json.dumps(out,indent=2),encoding='utf-8');print(json.dumps({'manifest':manifest,'files':len(files)}))
