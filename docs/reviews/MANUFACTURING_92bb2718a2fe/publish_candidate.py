"""Publish only the explicit manufacturing_v1 manifest. DO NOT run without root authorization.

CLI metadata.json [--state NEW.json]
Metadata: expected_head, commit_message, progress_lines (three single-line bullets),
stage_paths (exact candidate_files + three root docs + manifest, not PROGRESS).
No cleanup/reset/force push. Existing dirty PROGRESS bytes stay in the worktree.
"""
from pathlib import Path
import argparse, hashlib, json, os, subprocess, sys, traceback
from checkpoint_contract import (BASE, CAD, MANIFEST, AUDIT, RENDER, DOCS, GIT, BATCH_SHA, REMOTE,
    require, read, digest, safe_file, manifest_contract, package_contract, frozen_evidence)

ROOT = Path('D:/robot_hand').resolve()
WORK = Path(__file__).resolve().parent
BATCH = WORK.parent / 'gesture002/git_blob_batch.py'
COMMON = WORK / 'checkpoint_contract.py'
SNAPSHOT = WORK / 'git_state.json'

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('metadata', type=Path)
    parser.add_argument('--state', type=Path, default=WORK/'publish_state.json')
    args = parser.parse_args(); statepath = args.state.resolve(); metapath = args.metadata.resolve()
    require(WORK.drive.upper() == 'D:', 'D: work location required')
    require(statepath.is_relative_to(WORK) and statepath.suffix == '.json' and not statepath.exists(), 'Use a new work state JSON')
    require(metapath.is_relative_to(WORK) and metapath.is_file(), 'Metadata must be in this work folder')
    for key in ('GIT_DIR', 'GIT_WORK_TREE', 'GIT_INDEX_FILE'): require(not os.environ.get(key), 'Refuse inherited '+key)
    require(digest(BATCH) == BATCH_SHA, 'Batch reader source changed')
    sys.dont_write_bytecode = True; sys.path.insert(0, str(BATCH.parent))
    from git_blob_batch import ImmutableGitBlobs, repository_path
    env = dict(os.environ, GIT_NO_REPLACE_OBJECTS='1', GIT_OPTIONAL_LOCKS='0', GIT_TERMINAL_PROMPT='0',
        GCM_INTERACTIVE='never', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1',
        NUMEXPR_NUM_THREADS='1', PYTHONDONTWRITEBYTECODE='1', TEMP=str(WORK), TMP=str(WORK))
    state = dict(phase='preflight', pushed=False, candidate_sha=None, errors=[],
        writer_sha256=digest(__file__), contract_helper_sha256=digest(COMMON), batch_helper_sha256=BATCH_SHA,
        metadata_sha256=digest(metapath), preservation_snapshot_sha256=digest(SNAPSHOT))
    blobs = ImmutableGitBlobs(GIT, ROOT, env, WORK)
    def save(): statepath.write_text(json.dumps(state, indent=2, ensure_ascii=False, allow_nan=False)+'\n', encoding='utf-8')
    def git(*argv, data=None):
        options = dict(cwd=ROOT, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=False,
                       creationflags=subprocess.CREATE_NO_WINDOW)
        options.update(stdin=subprocess.DEVNULL) if data is None else options.update(input=data)
        result = subprocess.run([GIT, *argv], **options)
        require(result.returncode == 0, 'Git '+argv[0]+': '+result.stderr.decode('utf-8', 'replace'))
        return result.stdout
    def text(*args): return git(*args).decode('utf-8').strip()
    def names(*args): return {p.decode('utf-8') for p in git(*args).split(b'\0') if p}
    def path(name): return safe_file(ROOT, name, repository_path)
    def signature(name):
        p = ROOT/repository_path(name)
        if not p.exists(): return dict(exists=False)
        require(p.resolve() == p and p.is_file(), 'Aliased/non-file preservation target: '+name)
        return dict(exists=True, bytes=p.stat().st_size, sha256=digest(p))
    def preserve():
        for name, expected in preserved.items(): require(signature(name) == expected, 'Preserved dirty file changed: '+name)
        require(digest(path('CHECKPOINT.json')) == pointer_sha, 'CHECKPOINT changed')
    def frozen():
        for name, expected in bound.items(): require(signature(name) == expected, 'Manifest/source snapshot changed: '+name)
        require(digest(__file__) == state['writer_sha256'] and digest(COMMON) == state['contract_helper_sha256'] and digest(BATCH) == BATCH_SHA, 'Writer/helper changed')
        require(digest(metapath) == state['metadata_sha256'] and digest(SNAPSHOT) == state['preservation_snapshot_sha256'], 'Metadata/preservation snapshot changed')
    try:
        meta = read(metapath)
        require(meta.get('expected_head') == BASE, 'Expected fixed dcdd20c baseline')
        require(text('rev-parse', 'HEAD') == BASE and text('symbolic-ref', '--short', 'HEAD') == 'main', 'Expected unchanged main')
        require(not names('diff', '--cached', '--name-only', '-z') and not git('ls-files', '-u', '-z'), 'Index must be empty and unconflicted')
        urls = text('remote', 'get-url', '--push', '--all', 'origin').splitlines()
        require(len(urls) == 1 and urls[0] in REMOTE and text('remote', 'get-url', 'origin') in REMOTE, 'Unexpected origin')
        require(text('ls-remote', 'origin', 'refs/heads/main').split() == [BASE, 'refs/heads/main'], 'Remote main changed')
        snapshot = read(SNAPSHOT)
        require(snapshot.get('completed') is True and not snapshot.get('errors') and snapshot['head'] == BASE and snapshot['head_after'] == BASE and
                snapshot['porcelain_unchanged_during_read'] is True and snapshot['counts'] == dict(prior_shared_progress=1, prior_unrelated=186) and
                len(snapshot['dirty']) == 187 and not snapshot['new_activity'] and not snapshot['prior_changed_bytes'], 'Unexpected original dirty snapshot')
        require(all(r['stable_during_hash'] and r['same_bytes_as_prior'] for r in snapshot['dirty']), 'Unstable original byte snapshot')
        manifest = read(path(MANIFEST)); own = manifest_contract(manifest, repository_path)
        supplied = meta.get('stage_paths')
        require(isinstance(supplied, list) and len(supplied) == len(set(supplied)), 'Explicit unique stage_paths required')
        stage = set(supplied)
        require(stage == own | DOCS | {MANIFEST}, 'Stage list differs from exactly declared owned paths')
        for name in stage: repository_path(name)
        require(not any(n.startswith(CAD+'/') for n in names('ls-tree', '-r', '--name-only', '-z', BASE)), 'Candidate already present in baseline')
        preserved = {r['path']: dict(exists=True, bytes=r['bytes'], sha256=r['sha256']) for r in snapshot['dirty']}
        require(not (set(preserved) & stage), 'Stage list overlaps unrelated dirty work')
        current_dirty = names('ls-files', '--modified', '--deleted', '--others', '--exclude-standard', '-z')
        for name in current_dirty-stage:
            actual = signature(name)
            if name in preserved: require(preserved[name] == actual, 'Original dirty bytes changed: '+name)
            else: preserved[name] = actual
        pointer_sha = digest(path('CHECKPOINT.json'))
        require(pointer_sha == blobs.signature(BASE, 'CHECKPOINT.json')['sha256'], 'CHECKPOINT already changed')
        preserve()
        for name, item in manifest['files'].items():
            require(signature(name) == dict(exists=True, **item), 'Manifest byte/length mismatch: '+name)
            if name not in own | DOCS: require(blobs.signature(BASE, name)['sha256'] == item['sha256'], 'Non-owned dependency changed from baseline: '+name)
        for name in stage: require(path(name).stat().st_size < 100*1024*1024, 'GitHub oversized file: '+name)
        audit, source = package_contract(ROOT, manifest, repository_path, ROOT)
        evidence_index = 'AUDIT_EVIDENCE_INDEX.json'
        require(digest(path(evidence_index)) == blobs.signature(BASE, evidence_index)['sha256'], 'Frozen evidence index changed')
        evidence = frozen_evidence(read(path(evidence_index)))
        for name, expected in evidence.items():
            require(digest(path(name)) == expected == blobs.signature(BASE, name)['sha256'], 'Frozen baseline evidence changed: '+name)
        lines = meta.get('progress_lines'); message = meta.get('commit_message')
        require(isinstance(lines, list) and len(lines) == 3 and all(isinstance(s, str) and s.startswith('- ') and '\n' not in s and '\r' not in s for s in lines), 'Exactly three single-line progress bullets required')
        require(isinstance(message, str) and message.strip() and '\n' not in message and '\r' not in message, 'One-line commit message required')
        progress = blobs.read(BASE, 'PROGRESS.md').rstrip(b'\r\n') + b'\n\n' + ('\n'.join(lines)+'\n').encode('utf-8')
        bound = {name: signature(name) for name in set(manifest['files']) | stage | set(evidence) | {evidence_index}}
        preserve(); frozen()
        state.update(phase='snapshot_verified', stage_files=sorted(stage), manifest_sha256=digest(path(MANIFEST)),
            original_dirty_preserved=187, extra_dirty_preserved=sorted(set(preserved)-{r['path'] for r in snapshot['dirty']}),
            preserved_files=preserved, frozen_evidence_files_checked=222, audit_summary=audit['summary'], remaining_blockers=manifest['remaining_blockers'])
        save()
        require(text('rev-parse', 'HEAD') == BASE and not names('diff', '--cached', '--name-only', '-z'), 'HEAD/index changed before staging')
        pathfile = statepath.with_suffix('.paths.nul')
        with pathfile.open('xb') as stream: stream.write(b''.join(n.encode('utf-8')+b'\0' for n in sorted(stage)))
        git('-c', 'core.autocrlf=false', '--literal-pathspecs', 'add', '--pathspec-from-file='+str(pathfile), '--pathspec-file-nul')
        oid = git('hash-object', '-w', '--stdin', data=progress).decode().strip()
        git('update-index', '--add', '--cacheinfo', '100644', oid, 'PROGRESS.md')
        staged = names('diff', '--cached', '--name-only', '-z')
        require(staged and staged <= stage | {'PROGRESS.md'} and 'PROGRESS.md' in staged and own | {MANIFEST} <= staged, 'Unexpected/missing staged paths')
        tree = text('write-tree')
        for name, expected in bound.items():
            sig = blobs.signature(tree, name)
            require(sig['bytes'] == expected['bytes'] and sig['sha256'] == expected['sha256'], 'Staged tree bytes differ: '+name)
        require(blobs.read(tree, 'PROGRESS.md') == progress, 'Staged progress differs')
        require(blobs.signature(tree, 'CHECKPOINT.json')['sha256'] == pointer_sha, 'Staged CHECKPOINT changed')
        preserve(); frozen()
        git('-c', 'core.whitespace=blank-at-eol,blank-at-eof,space-before-tab,cr-at-eol', 'diff', '--cached', '--check')
        state.update(phase='staged_verified', verified_tree=tree, staged_names=sorted(staged)); save()
        require(text('rev-parse', 'HEAD') == BASE and text('symbolic-ref', '--short', 'HEAD') == 'main' and
                names('diff', '--cached', '--name-only', '-z') == staged and text('write-tree') == tree, 'HEAD/index changed before commit')
        git('commit', '-m', message)
        sha = text('rev-parse', 'HEAD'); state.update(phase='committed_not_pushed', candidate_sha=sha); save()
        require(text('rev-parse', sha+'^') == BASE and text('rev-parse', sha+'^{tree}') == tree and
                names('diff-tree', '--no-commit-id', '--name-only', '-r', '-z', sha) == staged, 'Committed scope/tree differs; do not push')
        preserve(); frozen()
        require(text('remote', 'get-url', '--push', '--all', 'origin').splitlines() == urls, 'Push destination changed')
        git('push', 'origin', sha+':refs/heads/main')
        require(text('ls-remote', 'origin', 'refs/heads/main').split() == [sha, 'refs/heads/main'], 'Remote SHA differs after push')
        state.update(phase='pushed', pushed=True, origin_main=sha, checkpoint_unchanged=True)
    except Exception as error:
        state['errors'].append(dict(error=repr(error), traceback=traceback.format_exc()))
    finally:
        blobs.close(abort=True); save()
    print(json.dumps(state, ensure_ascii=False)); return 0 if state['pushed'] else 1

if __name__ == '__main__': raise SystemExit(main())
