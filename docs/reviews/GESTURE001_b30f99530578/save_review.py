"""Record a completed GESTURE001 clean-copy reproduction; never run on import.

CLI: python save_review.py metadata.json [--state new_review_state.json]
Required metadata: id, expected_head, candidate_sha, review_directory,
review_markdown, progress_lines (one to three single-line bullets).
Optional: commit_message, evidence_files (individual relative files inside the
review directory; review.json is always copied). The executed review_commit.py
and this writer are copied with their hashes. This tool does not rerun CAD.

Only a new docs/reviews/GESTURE001_<sha12> folder, CHECKPOINT.json and a PROGRESS
index blob are committed. The existing PROGRESS working copy is never written.
Known geometric failures may reproduce; this is not manufacturing approval.
No reset, cleanup, amend, force push or automatic recovery is performed.
"""
from pathlib import Path, PurePosixPath
from datetime import datetime, timezone
import argparse
import hashlib
import json
import os
import re
import subprocess
import traceback

ROOT=Path('D:/robot_hand').resolve()
WORK=Path(__file__).resolve().parent
REVIEWS=(WORK.parent/'reviews').resolve()
RUNNER=WORK/'review_commit.py'
GIT='C:/Program Files/Git/cmd/git.exe'
ID='GESTURE001'
MANIFEST='checkpoints/GESTURE001.manifest.json'
POINTER='CHECKPOINT.json'
PROGRESS='PROGRESS.md'
CAD='cad/gesture_hand_v3'
SHA1=re.compile(r'^[0-9a-f]{40}$')
SHA256=re.compile(r'^[0-9a-f]{64}$')
ALLOWED_REMOTE={'https://github.com/ckh7488/robot_hand','https://github.com/ckh7488/robot_hand.git','git@github.com:ckh7488/robot_hand.git'}
SCOPE=('Exact committed archive and manifest/frozen evidence; all CAD and mass tensors rebuilt; '
       'open, fist and thumb-up exact collision results and individual nominal fabrication interfaces rerun. '
       'Other six endpoints, transitions, renderer and dynamics not rerun. Existing failure outcomes are retained; '
       'this is same-task reproduction, not external independent audit or manufacturing/performance approval.')


def hbytes(data):return hashlib.sha256(data).hexdigest()
def hfile(path):return hbytes(Path(path).read_bytes())
def read(path):return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def relative(value):
    if not isinstance(value,str) or not value or '\\' in value or ':' in value or any(ord(c)<32 for c in value):
        raise ValueError('Expected normalized relative path')
    p=PurePosixPath(value)
    if p.is_absolute() or str(p)!=value or any(x in ('..','.git') for x in p.parts):
        raise ValueError('Unsafe relative path: '+value)
    return value


def inside(base,name):
    relative(name);lexical=base/name;path=lexical.resolve()
    if not path.is_relative_to(base) or path!=lexical:
        raise ValueError('Escaping/aliased path: '+name)
    return path


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('metadata',type=Path)
    ap.add_argument('--state',type=Path,default=WORK/'review_state.json')
    args=ap.parse_args();metadata_path=args.metadata.resolve();state_path=args.state.resolve()
    if not state_path.is_relative_to(WORK) or state_path==metadata_path or state_path.exists() or state_path.suffix.lower()!='.json':
        raise ValueError('Use a new state JSON inside this task work directory')
    for key in ('GIT_DIR','GIT_WORK_TREE','GIT_INDEX_FILE'):
        if os.environ.get(key):raise ValueError('Refuse inherited '+key)
    env=os.environ.copy();env.update(GIT_TERMINAL_PROMPT='0',GCM_INTERACTIVE='never',
        OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',TEMP=str(WORK),TMP=str(WORK))
    state=dict(id=ID,phase='preflight',review_sha=None,pushed=False,errors=[],
        metadata=str(metadata_path),metadata_sha256=hfile(metadata_path),writer_sha256=hfile(__file__))

    def save():
        state_path.parent.mkdir(parents=True,exist_ok=True)
        state_path.write_text(json.dumps(state,indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf-8')

    def git(*arguments,data=None,ok=(0,)):
        options=dict(cwd=ROOT,env=env,stdout=subprocess.PIPE,stderr=subprocess.PIPE,
            creationflags=subprocess.CREATE_NO_WINDOW,shell=False)
        if data is None:options['stdin']=subprocess.DEVNULL
        else:options['input']=data
        p=subprocess.run([GIT,*arguments],**options)
        if p.returncode not in ok:
            raise RuntimeError('Git '+arguments[0]+' failed: '+p.stderr.decode('utf-8','replace'))
        return p

    def text(*arguments):return git(*arguments).stdout.decode('utf-8').strip()
    def names(*arguments):return [x.decode('utf-8') for x in git(*arguments).stdout.split(b'\0') if x]
    def committed(sha,name):return git('show',sha+':'+relative(name)).stdout

    try:
        metadata=read(metadata_path);expected=metadata['expected_head'];candidate=metadata['candidate_sha']
        if metadata.get('id')!=ID or not isinstance(expected,str) or not SHA1.fullmatch(expected) or not isinstance(candidate,str) or not SHA1.fullmatch(candidate):
            raise ValueError('Expected GESTURE001 and full expected_head/candidate_sha')
        if text('symbolic-ref','--quiet','HEAD')!='refs/heads/main' or text('rev-parse','HEAD')!=expected:
            raise ValueError('Expected unchanged main HEAD')
        if text('rev-parse',candidate+'^{commit}')!=candidate:raise ValueError('Candidate is not the exact commit')
        git('merge-base','--is-ancestor',candidate,expected)
        if names('diff','--cached','--name-only','-z') or git('ls-files','--unmerged','-z').stdout:
            raise ValueError('Preserve another task staged/unmerged index')
        if text('remote','get-url','origin') not in ALLOWED_REMOTE:raise ValueError('Unexpected origin remote')
        push_urls=text('remote','get-url','--push','--all','origin').splitlines()
        if len(push_urls)!=1 or push_urls[0] not in ALLOWED_REMOTE:raise ValueError('Expected one approved origin push URL')
        state.update(candidate_sha=candidate,parent_sha=expected,origin_push_url=push_urls[0])
        source=Path(metadata['review_directory']).resolve()
        if not source.is_relative_to(REVIEWS) or source==REVIEWS or not source.is_dir():
            raise ValueError('Expected an existing individual clean-copy review directory on D:')
        review_path=inside(source,'review.json');review_bytes=review_path.read_bytes();review=json.loads(review_bytes)
        if review.get('completed') is not True or review.get('reproduction_passed') is not True or review.get('errors'):
            raise ValueError('Review is incomplete/failed; preserve its evidence without marking reproduced')
        if review.get('target_sha')!=candidate or review.get('resolved_commit_sha')!=candidate:
            raise ValueError('Review does not identify the exact candidate commit')
        if review.get('manufacturing_approved') is not False:raise ValueError('Review must explicitly withhold manufacturing approval')
        runner_sha=hfile(RUNNER)
        if review.get('reviewer_sha256')!=runner_sha or review.get('reviewer_source_unchanged') is not True:
            raise ValueError('Executed review runner does not match the source being preserved')
        guard=review.get('source_hash_guard',{})
        if guard.get('passed') is not True or not guard.get('start') or guard.get('start')!=guard.get('end'):
            raise ValueError('Reproduction source guard incomplete')
        for name,digest in guard['start'].items():
            if not isinstance(digest,str) or not SHA256.fullmatch(digest) or hbytes(committed(candidate,name))!=digest:
                raise ValueError('Reproduction source differs from candidate: '+name)
        labels=[r['label'] for r in review['commands']]
        if labels!=['archive','rebuild','core_collisions','interfaces'] or any(r.get('exit_code')!=0 for r in review['commands']):
            raise ValueError('Expected four completed bounded reproduction commands')
        old_pointer=json.loads(committed(candidate,POINTER));pointer_bytes=committed(expected,POINTER)
        pointer=json.loads(pointer_bytes)
        for label,value in [('candidate',old_pointer),('current',pointer)]:
            if value.get('id')!=ID or value.get('manifest')!=MANIFEST or value.get('candidate')!=CAD:
                raise ValueError(label+' checkpoint pointer identifies another candidate')
        if pointer.get('post_commit_results_reproduced') is True or pointer.get('post_commit_review_report'):
            raise ValueError('Preserve the already-recorded checkpoint review')
        if inside(ROOT,POINTER).read_bytes()!=pointer_bytes:raise ValueError('Preserve dirty CHECKPOINT working copy')
        manifest_bytes=committed(candidate,MANIFEST)
        if committed(expected,MANIFEST)!=manifest_bytes:raise ValueError('Checkpoint manifest changed after candidate commit')
        manifest=json.loads(manifest_bytes)
        if manifest.get('id')!=ID or review.get('manifest_files_checked')!=len(manifest['files']) or review.get('frozen_files_checked')!=222:
            raise ValueError('Review manifest/frozen scope differs from checkpoint')
        # The runner already rehashed all manifest files in its clean archive.
        # Bind the preserved detailed comparisons to the candidate's part set.
        candidate_scene=json.loads(committed(candidate,CAD+'/generated/scene.json'))
        part_names=[p['name'] for p in candidate_scene['hand']]
        if len(part_names)!=len(set(part_names)):raise ValueError('Duplicate candidate part names')
        for key in ('geometry_reproduction','mass_tensor_reproduction'):
            rows=review[key];row_names=[r['name'] for r in rows]
            if len(row_names)!=len(set(row_names)) or set(row_names)!=set(part_names) or any(r.get('passed') is not True for r in rows):
                raise ValueError('Incomplete or failed '+key)
        core=review['core_collision_reproduction']
        if len(core)!=3 or {r['gesture'] for r in core}!={'open','fist','thumb_up'} or any(r.get('passed') is not True or r.get('missing_pairs') or r.get('extra_pairs') for r in core):
            raise ValueError('Core collision outcomes were not reproduced')
        if review.get('interface_summary_reproduced') is not True or review.get('interface_changed_checks') or not isinstance(review.get('interface_individual_checks_compared'),int) or review['interface_individual_checks_compared']<=0:
            raise ValueError('Individual interface outcomes were not reproduced')
        if review.get('interface_original_summary')!=review.get('interface_recomputed_summary'):
            raise ValueError('Interface summary mismatch')
        lines=metadata['progress_lines']
        if not isinstance(lines,list) or not 1<=len(lines)<=3 or any(not isinstance(x,str) or not x.startswith('- ') or '\r' in x or '\n' in x for x in lines):
            raise ValueError('One to three single-line progress bullets required')
        addition='\n'.join(lines)+'\n'
        if ID not in addition or candidate[:12] not in addition:
            raise ValueError('Progress must identify GESTURE001 and the candidate SHA prefix')
        progress_head=committed(expected,PROGRESS)
        folder_name='docs/reviews/'+ID+'_'+candidate[:12]
        if folder_name.encode() in progress_head:raise ValueError('Progress already contains this review')
        progress_path=inside(ROOT,PROGRESS)
        progress_work_before=hfile(progress_path) if progress_path.exists() else None
        progress_bytes=progress_head.rstrip(b'\r\n')+b'\n\n'+addition.encode('utf-8')
        markdown=metadata['review_markdown']
        if not isinstance(markdown,str) or ID not in markdown or candidate not in markdown:
            raise ValueError('Review Markdown must name GESTURE001 and the full candidate SHA')
        subject=metadata.get('commit_message','review(GESTURE001): reproduce native CAD and bounded gesture audits at '+candidate[:12])
        if not isinstance(subject,str) or not subject.startswith('review(GESTURE001): ') or '\n' in subject or '\r' in subject:
            raise ValueError('Expected single-line review(GESTURE001) commit subject')
        folder=inside(ROOT,folder_name)
        if folder.exists():raise FileExistsError('Preserve prior review folder')
        extra=metadata.get('evidence_files',[])
        if not isinstance(extra,list) or len(extra)!=len(set(extra)):raise ValueError('Unique individual evidence_files required')
        payload={'review.json':review_bytes,'README.md':(markdown.rstrip()+'\n').encode('utf-8'),
            'review_commit.py':RUNNER.read_bytes(),'save_review.py':Path(__file__).read_bytes()}
        source_hashes={'review.json':hbytes(review_bytes)}
        reserved=set(payload)|{'evidence.json'}
        for name in extra:
            relative(name)
            if name in reserved or name.startswith('repository/') or name in ('repository','committed.tar'):
                raise ValueError('Reserved or regenerated-repository evidence path: '+name)
            path=inside(source,name)
            if not path.is_file() or path.suffix.lower() in ('.log','.err','.pyc','.pyo') or '__pycache__' in path.parts or path.name.startswith('.env'):
                raise ValueError('Only individual durable evidence files may be copied: '+name)
            if path.stat().st_size>=100*1024*1024:raise ValueError('Evidence file reaches GitHub single-file limit')
            payload[name]=path.read_bytes();source_hashes[name]=hbytes(payload[name])
        provenance=dict(id=ID,target_sha=candidate,recorded_utc=datetime.now(timezone.utc).isoformat(),
            scope=SCOPE,manufacturing_approved=False,motor_performance_validated=False,
            checkpoint_manifest_sha256=hbytes(manifest_bytes),reviewer_sha256=runner_sha,
            reviewer_source_unchanged=True,writer_sha256=hfile(__file__),
            commands_reexecuted=labels,part_count=len(part_names),core_gestures=['open','fist','thumb_up'],
            interface_checks=review['interface_individual_checks_compared'],
            frozen_files_checked=222,manifest_files_checked=len(manifest['files']),
            files={name:dict(bytes=len(raw),sha256=hbytes(raw)) for name,raw in sorted(payload.items())})
        payload['evidence.json']=(json.dumps(provenance,indent=2,ensure_ascii=False,allow_nan=False)+'\n').encode('utf-8')
        pointer.update(post_commit_results_reproduced=True,post_commit_review_target=candidate,
            post_commit_review_report=folder_name+'/review.json',post_commit_reproduction_scope=SCOPE)
        updated_pointer=(json.dumps(pointer,indent=2,ensure_ascii=False,allow_nan=False)+'\n').encode('utf-8')
        # No external computation is launched. Check all inputs again before
        # the first repository write, then create this new owned evidence set.
        if text('rev-parse','HEAD')!=expected or names('diff','--cached','--name-only','-z'):
            raise ValueError('HEAD/index changed before writing review')
        if inside(ROOT,POINTER).read_bytes()!=pointer_bytes or hfile(RUNNER)!=runner_sha:
            raise ValueError('Pointer or review runner changed during preparation')
        for name,digest in source_hashes.items():
            if hfile(inside(source,name))!=digest:raise ValueError('Review evidence changed: '+name)
        folder.mkdir(parents=True,exist_ok=False)
        for name,raw in payload.items():
            path=inside(folder,name);path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(raw)
        inside(ROOT,POINTER).write_bytes(updated_pointer)
        expected_bytes={folder_name+'/'+name:raw for name,raw in payload.items()}
        expected_bytes[POINTER]=updated_pointer
        stage_files=set(expected_bytes)
        state.update(phase='evidence_written',review_directory=folder_name,scope=SCOPE,
            staged_files=sorted(stage_files|{PROGRESS}),progress_worktree_sha256_before=progress_work_before,
            progress_index_sha256=hbytes(progress_bytes),saved_report_sha256=hbytes(review_bytes))
        save()
        path_list=WORK/('gesture001_review_'+candidate[:12]+'_paths.nul')
        if path_list.exists():raise FileExistsError('Preserve previous review staging path list')
        path_list.write_bytes(b''.join(name.encode('utf-8')+b'\0' for name in sorted(stage_files)))
        if text('rev-parse','HEAD')!=expected or names('diff','--cached','--name-only','-z'):
            raise ValueError('HEAD/index changed before staging; preserve files for inspection')
        git('-c','core.autocrlf=false','--literal-pathspecs','add','--pathspec-from-file='+str(path_list),'--pathspec-file-nul')
        blob=git('hash-object','-w','--stdin',data=progress_bytes).stdout.decode().strip()
        git('update-index','--add','--cacheinfo','100644',blob,PROGRESS)
        staged=set(names('diff','--cached','--name-only','-z'))
        if staged!=stage_files|{PROGRESS}:raise ValueError('Unexpected staged set; do not commit')
        staged_tree=text('write-tree')
        for name,raw in expected_bytes.items():
            if inside(ROOT,name).read_bytes()!=raw or git('show',':'+name).stdout!=raw:
                raise ValueError('Review staged bytes changed: '+name)
        if git('show',':'+PROGRESS).stdout!=progress_bytes:raise ValueError('Staged progress bytes changed')
        if (hfile(progress_path) if progress_path.exists() else None)!=progress_work_before:
            raise ValueError('Progress working copy changed concurrently; do not commit')
        git('-c','core.whitespace=blank-at-eol,blank-at-eof,space-before-tab,cr-at-eol','diff','--cached','--check')
        state['phase']='staged_verified';save()
        if text('rev-parse','HEAD')!=expected or text('symbolic-ref','--quiet','HEAD')!='refs/heads/main' or set(names('diff','--cached','--name-only','-z'))!=staged or text('write-tree')!=staged_tree:
            raise ValueError('Main/HEAD/index changed before commit')
        git('commit','-m',subject)
        review_sha=text('rev-parse','HEAD');state.update(phase='committed_not_pushed',review_sha=review_sha);save()
        if text('rev-parse',review_sha+'^')!=expected or set(names('diff-tree','--no-commit-id','--name-only','-r','-z',review_sha))!=staged:
            raise ValueError('Committed parent/paths differ; do not push')
        for name,raw in expected_bytes.items():
            if committed(review_sha,name)!=raw:raise ValueError('Committed review bytes differ; do not push')
        if committed(review_sha,PROGRESS)!=progress_bytes:raise ValueError('Committed progress differs; do not push')
        if (hfile(progress_path) if progress_path.exists() else None)!=progress_work_before:
            raise ValueError('Progress working copy changed during commit; do not push')
        if text('remote','get-url','--push','--all','origin').splitlines()!=push_urls:
            raise ValueError('Origin push destination changed; do not push')
        git('push','origin',review_sha+':refs/heads/main')
        remote=text('ls-remote','--heads','origin','refs/heads/main').split()
        if not remote or remote[0]!=review_sha:raise ValueError('Remote main does not match review commit')
        state.update(phase='pushed',pushed=True,origin_main=remote[0],progress_worktree_unchanged=True)
    except Exception as exc:
        state['errors'].append(dict(error=repr(exc),traceback=traceback.format_exc()))
        state['failed_phase']=state['phase'];state['phase']='failed'
    save();print(json.dumps(state,ensure_ascii=False,allow_nan=False),flush=True)
    return 0 if state['pushed'] else 1


if __name__=='__main__':raise SystemExit(main())
