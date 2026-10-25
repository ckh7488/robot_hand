"""Explicit thumb_wrist_v1 save; run only when authorized. CLI: metadata.json [--state NEW.json].
Metadata: expected_head, commit_message, progress_lines (three single-line bullets).
Preserves CHECKPOINT and unrelated files; no cleanup, reset, force push or retry.
"""
from pathlib import Path
import argparse, hashlib, json, os, re, subprocess, sys, traceback

ROOT=Path('D:/robot_hand').resolve(); WORK=Path(__file__).resolve().parent
BASE='fae9514622079e5349b4b429553202e32cb60c2b'; CAD='cad/thumb_wrist_v1'; MANIFEST=CAD+'/manifest.json'
GIT='C:/Program Files/Git/cmd/git.exe'; BATCH=WORK.parent/'gesture002/git_blob_batch.py'
BATCH_SHA='28e37b501bac04630ffb953bfe6ccc1cb81674830e475834c539b0c8abd5f386'
REMOTE={'https://github.com/ckh7488/robot_hand','https://github.com/ckh7488/robot_hand.git','git@github.com:ckh7488/robot_hand.git'}
FILES={'wrist_profile.py','profile.json','audit.py','render.py','README.md','reports/motion_audit.json',
       'generated/comparison.png','generated/candidate.png','generated/motion.gif','generated/render_report.json'}
DOCS={'README.md','AGENTS.md'}; SNAPSHOT=WORK/'preparation_git_snapshot.json'
def digest(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for chunk in iter(lambda:f.read(1048576),b''):h.update(chunk)
    return h.hexdigest()
def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def require(ok,message):
    if not ok:raise ValueError(message)
def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('metadata',type=Path)
    ap.add_argument('--state',type=Path,default=WORK/'publish_state.json');a=ap.parse_args()
    statepath=a.state.resolve();require(statepath.is_relative_to(WORK) and statepath.suffix=='.json' and not statepath.exists(),'New work state JSON required')
    require(a.metadata.resolve().is_relative_to(WORK) and a.metadata.suffix=='.json','Metadata JSON must be in this D: work directory')
    for key in ('GIT_DIR','GIT_WORK_TREE','GIT_INDEX_FILE'):require(not os.environ.get(key),'Refuse inherited '+key)
    require(digest(BATCH)==BATCH_SHA,'Batch helper changed');sys.dont_write_bytecode=True;sys.path.insert(0,str(BATCH.parent))
    from git_blob_batch import ImmutableGitBlobs, repository_path
    env=dict(os.environ,GIT_NO_REPLACE_OBJECTS='1',GIT_OPTIONAL_LOCKS='0',GIT_TERMINAL_PROMPT='0',GCM_INTERACTIVE='never',
             OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',TEMP=str(WORK),TMP=str(WORK))
    state=dict(phase='preflight',pushed=False,candidate_sha=None,errors=[],writer_sha256=digest(__file__),batch_helper_sha256=BATCH_SHA,
               metadata=str(a.metadata.resolve()),metadata_sha256=digest(a.metadata),preservation_snapshot_sha256=digest(SNAPSHOT))
    blobs=ImmutableGitBlobs(GIT,ROOT,env,WORK);bound={};own={CAD+'/'+p for p in FILES};stage=own|DOCS|{MANIFEST}
    def save():statepath.write_text(json.dumps(state,indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf-8')
    def git(*args,data=None):
        opts=dict(cwd=ROOT,env=env,stdout=subprocess.PIPE,stderr=subprocess.PIPE,shell=False,creationflags=subprocess.CREATE_NO_WINDOW)
        opts.update(stdin=subprocess.DEVNULL) if data is None else opts.update(input=data)
        p=subprocess.run([GIT,*args],**opts);require(p.returncode==0,'Git '+args[0]+': '+p.stderr.decode('utf-8','replace'));return p.stdout
    def text(*args):return git(*args).decode('utf-8').strip()
    def names(*args):return {p.decode('utf-8') for p in git(*args).split(b'\0') if p}
    def path(name):
        name=repository_path(name);p=ROOT/name;require(p.resolve()==p and p.is_file(),'Missing/aliased file: '+name);return p
    def bind(name,expected=None):
        p=Path(name)
        if p.is_absolute():require(p.resolve().is_relative_to(ROOT),'Input outside repository');name=p.resolve().relative_to(ROOT).as_posix()
        p=path(name);actual=digest(p);require(expected is None or actual==expected,'Stale input: '+name)
        require(name not in bound or bound[name]==actual,'Input changed: '+name)
        if name not in own:require(blobs.signature(BASE,name)['sha256']==actual,'Non-candidate input differs from baseline: '+name)
        bound[name]=actual;return name
    def guard(report):
        g=report.get('input_hash_guard',{});require(report.get('completed') is True and not report.get('errors') and g.get('passed') is True and bool(g.get('start')) and g['start']==g.get('end'),'Incomplete audit or failed input guard')
        return {bind(n,h) for n,h in g['start'].items()}
    def preserve():
        for row in snapshot['dirty']:
            p=ROOT/row['path'];require(p.is_file() and digest(p)==row['sha256'],'Preserved dirty file changed: '+row['path'])
        require(digest(ROOT/'CHECKPOINT.json')==pointer_sha,'CHECKPOINT changed')
    def frozen():
        for n,h in frozen_files.items():require(digest(path(n))==h,'Snapshot changed: '+n)
        require(digest(__file__)==state['writer_sha256'] and digest(BATCH)==BATCH_SHA,'Writer/helper source changed')
        require(digest(a.metadata)==state['metadata_sha256'] and digest(SNAPSHOT)==state['preservation_snapshot_sha256'],'Metadata/preservation evidence changed')
    try:
        meta=read(a.metadata);require(meta.get('expected_head')==BASE,'Expected the fixed full baseline HEAD')
        require(text('rev-parse','HEAD')==BASE and text('symbolic-ref','--short','HEAD')=='main','Expected unchanged main HEAD')
        require(not names('diff','--cached','--name-only','-z') and not git('ls-files','-u','-z'),'Preserve nonempty/conflicted index')
        urls=text('remote','get-url','--push','--all','origin').splitlines();require(len(urls)==1 and urls[0] in REMOTE and text('remote','get-url','origin') in REMOTE,'Unexpected origin URL')
        require(text('ls-remote','origin','refs/heads/main').split()==[BASE,'refs/heads/main'],'Remote main advanced')
        snapshot=read(SNAPSHOT);require(snapshot['head']==BASE and len(snapshot['dirty'])==187 and snapshot['all_file_stats_stable'],'Preservation snapshot differs')
        require(all(r['classification'] in ('unrelated_preserve','shared_progress') for r in snapshot['dirty']),'Snapshot contains owned mutable files')
        pointer_sha=digest(ROOT/'CHECKPOINT.json');require(pointer_sha==blobs.signature(BASE,'CHECKPOINT.json')['sha256'],'CHECKPOINT already changed');preserve()
        lines=meta['progress_lines'];require(isinstance(lines,list) and len(lines)==3 and all(isinstance(x,str) and x.startswith('- ') and '\n' not in x and '\r' not in x for x in lines),'Three single-line progress bullets required')
        subject=meta['commit_message'];require(isinstance(subject,str) and bool(subject.strip()) and '\n' not in subject and '\r' not in subject,'One-line commit message required')
        require(not (ROOT/MANIFEST).exists(),'Preserve existing manifest')
        require(not any(p.startswith(CAD+'/') for p in names('ls-tree','-r','--name-only','-z',BASE)),'Candidate already exists in baseline')
        for n in own|DOCS:require(path(n).stat().st_size<100*1024*1024,'Oversized candidate file: '+n)
        audit=read(ROOT/(CAD+'/reports/motion_audit.json'));inputs=guard(audit)
        require({CAD+'/'+n for n in ('audit.py','wrist_profile.py','profile.json')}<=inputs,'Audit omitted portable candidate sources')
        summary=audit.get('summary',{})
        require(audit.get('mode')=='full' and summary.get('full_sampled_path_executed') is True and summary.get('cases_with_material_collision')==0 and summary.get('operation_failure_count')==0,'Full sampled path must complete without reported collision or operation failure')
        require(bool(audit.get('cases')) and all(c.get('collision_count')==0 and not c.get('collision_details') and not c.get('operation_failures') for c in audit['cases'].values()),'Raw case outcomes disagree with the zero-collision summary')
        render=read(ROOT/(CAD+'/generated/render_report.json'))
        require(render.get('status')=='completed' and render.get('completed') is True and render.get('inputs_unchanged') is True and not render.get('errors'),'Incomplete render')
        render_inputs={bind(n,h) for n,h in render.get('source_sha256',{}).items()}
        require({CAD+'/'+n for n in ('render.py','wrist_profile.py','profile.json')}<=render_inputs,'Render omitted portable candidate sources')
        require(set(render.get('output_sha256',{}))=={'comparison.png','candidate.png','motion.gif'},'Unexpected render outputs')
        for n,h in render['output_sha256'].items():bind(CAD+'/generated/'+n,h)
        for n in own:bind(n)
        manifest=dict(id='thumb_wrist_v1',baseline_commit=BASE,scope='Isolated pose/range study; unchanged GESTURE002 geometry. No manufacturing, continuous collision or latency approval.',
            writer_sha256=state['writer_sha256'],batch_helper_sha256=BATCH_SHA,candidate_files=sorted(own),
            files={n:dict(bytes=path(n).stat().st_size,sha256=h) for n,h in sorted(bound.items())})
        with (ROOT/MANIFEST).open('x',encoding='utf-8',newline='\n') as f:f.write(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n')
        progress=blobs.read(BASE,'PROGRESS.md').rstrip(b'\r\n')+b'\n\n'+('\n'.join(lines)+'\n').encode('utf-8')
        frozen_files={n:digest(path(n)) for n in stage|set(bound)};preserve();frozen()
        state.update(phase='snapshot_verified',parent_sha=BASE,stage_files=sorted(stage),manifest_sha256=digest(ROOT/MANIFEST),preservation_snapshot_sha256=digest(SNAPSHOT));save()
        require(text('rev-parse','HEAD')==BASE and not names('diff','--cached','--name-only','-z'),'HEAD/index changed before staging')
        pathfile=WORK/'publish_paths.nul'
        with pathfile.open('xb') as f:f.write(b''.join(n.encode('utf-8')+b'\0' for n in sorted(stage)))
        git('-c','core.autocrlf=false','--literal-pathspecs','add','--pathspec-from-file='+str(pathfile),'--pathspec-file-nul')
        oid=git('hash-object','-w','--stdin',data=progress).decode().strip();git('update-index','--add','--cacheinfo','100644',oid,'PROGRESS.md')
        staged=names('diff','--cached','--name-only','-z');require(staged and staged<=stage|{'PROGRESS.md'} and 'PROGRESS.md' in staged,'Unexpected staged paths')
        tree=text('write-tree')
        for n,h in frozen_files.items():require(blobs.signature(tree,n)['sha256']==h,'Staged bytes differ: '+n)
        require(blobs.read(tree,'PROGRESS.md')==progress,'Staged progress differs');preserve();frozen()
        git('-c','core.whitespace=blank-at-eol,blank-at-eof,space-before-tab,cr-at-eol','diff','--cached','--check')
        state.update(phase='staged_verified',verified_tree=tree,staged_names=sorted(staged));save()
        require(text('rev-parse','HEAD')==BASE and text('symbolic-ref','--short','HEAD')=='main' and names('diff','--cached','--name-only','-z')==staged and text('write-tree')==tree,'HEAD/index changed before commit')
        git('commit','-m',subject);sha=text('rev-parse','HEAD');state.update(phase='committed_not_pushed',candidate_sha=sha);save()
        require(text('rev-parse',sha+'^')==BASE and text('rev-parse',sha+'^{tree}')==tree and names('diff-tree','--no-commit-id','--name-only','-r','-z',sha)==staged,'Committed scope/tree differs; do not push')
        preserve();frozen();require(text('remote','get-url','--push','--all','origin').splitlines()==urls,'Push destination changed')
        git('push','origin',sha+':refs/heads/main');require(text('ls-remote','origin','refs/heads/main').split()==[sha,'refs/heads/main'],'Remote SHA differs after push')
        state.update(phase='pushed',pushed=True,origin_main=sha,checkpoint_unchanged=True,preserved_dirty_files=187)
    except Exception as e:state['errors'].append(dict(error=repr(e),traceback=traceback.format_exc()))
    finally:
        blobs.close(abort=True);save()
    print(json.dumps(state,ensure_ascii=False));return 0 if state['pushed'] else 1
if __name__=='__main__':raise SystemExit(main())
