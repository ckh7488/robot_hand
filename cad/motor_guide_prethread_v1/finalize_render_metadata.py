"""Remove the previous report's self-hash entry, preserving images and execution."""
import json,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
def sha(f):return hashlib.sha256(f.read_bytes()).hexdigest()
def main():
    path=HERE/'images/report.json';raw=path.read_bytes();report=json.loads(raw.decode('utf-8-sig'));assert report['completed'] and 'report.json' in report['outputs'];history=HERE/'history/render2_initial_report.json';assert not history.exists();history.write_bytes(raw);report['outputs'].pop('report.json')
    for name,h in report['outputs'].items():assert sha(HERE/'images'/name)==h
    report['source_sha256'][Path(__file__).relative_to(ROOT).as_posix()]=sha(Path(__file__));report['metadata_only_correction']='Removed prior report self-hash from outputs after rerender. Original report kept in history/render2_initial_report.json; PNG/native/geometry unchanged.';path.write_bytes((json.dumps(report,ensure_ascii=False,indent=2)+'\n').encode('utf-8'))
if __name__=='__main__':main()
