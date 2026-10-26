"""Supplement native core's160geometry hashes with all frozen code/reports."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,required=True);ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    if a.output.exists() or a.output.resolve().is_relative_to(a.root.resolve()):raise ValueError('New external output required')
    def git(*args):return subprocess.check_output(['git',*args],cwd=a.root,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    if git('rev-parse','HEAD')!=a.commit or git('status','--porcelain=v1'):raise ValueError('Wrong/dirty review checkout')
    index=json.loads((a.root/'AUDIT_EVIDENCE_INDEX.json').read_text(encoding='utf-8-sig'));groups={};failures=[]
    for group in ['geometry_sha256','code_sha256','report_sha256']:
        hashes=index[group];groups[group]=len(hashes)
        for rel,expected in hashes.items():
            path=(a.root/rel).resolve()
            if not path.is_relative_to(a.root.resolve()):raise ValueError('Escaping path')
            actual=hashlib.sha256(path.read_bytes()).hexdigest()
            if actual!=expected:failures.append(dict(path=rel,expected=expected,actual=actual))
    clean=not git('status','--porcelain=v1')
    result=dict(target_commit=a.commit,passed=not failures and clean,group_counts=groups,total_hashes=sum(groups.values()),failures=failures,clean_checkout=clean,
        purpose='Supplement geometry-only160hash count in core.json. Includes frozen38code and24report files; no extra native collision claims.')
    a.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    if not result['passed']:raise ValueError('Frozen evidence changed')


if __name__=='__main__':main()
