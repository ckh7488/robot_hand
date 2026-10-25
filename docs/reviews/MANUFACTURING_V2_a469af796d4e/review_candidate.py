"""Exact-SHA saved-CAD audit replay. Prepare only; root authorizes execution later.

CLI --sha FULL_COMMIT --output NEW_D_WORK_REVIEWS_DIRECTORY
Materializes the manifest input closure from immutable Git bytes. Replays the full
assembly audit into an external report. Does not rebuild CAD or rerender images.
"""
from pathlib import Path
import argparse, importlib.metadata, json, os, platform, re, shutil, subprocess, sys, time, traceback
from checkpoint_contract import (BASE, CAD, MANIFEST, AUDIT, DOCS, GIT, BATCH_SHA, SCOPE,
    require, read, digest, safe_file, manifest_contract, package_contract, report_guard,
    audit_contract, frozen_evidence, compare_audits, native_part_comparisons)

ROOT = Path('D:/robot_hand').resolve()
WORK = Path(__file__).resolve().parent
REVIEWS = WORK.parent/'reviews'
BATCH = WORK.parent/'gesture002/git_blob_batch.py'
COMMON = WORK/'checkpoint_contract.py'

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sha', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(); destination = args.output.resolve()
    require(re.fullmatch('[0-9a-f]{40}', args.sha), 'Full commit SHA required')
    require(destination.drive.upper() == 'D:' and destination.is_relative_to(REVIEWS.resolve()) and
            destination != REVIEWS.resolve() and not destination.exists(), 'Use a new D: reviews directory')
    for key in ('GIT_DIR', 'GIT_WORK_TREE', 'GIT_INDEX_FILE'): require(not os.environ.get(key), 'Refuse inherited '+key)
    require(digest(BATCH) == BATCH_SHA, 'Batch reader changed')
    sys.dont_write_bytecode = True; sys.path.insert(0, str(BATCH.parent))
    from git_blob_batch import ImmutableGitBlobs, repository_path
    destination.mkdir(parents=True); repo = destination/'repository'; repo.mkdir(); tmp = destination/'tmp'; tmp.mkdir()
    env = dict(os.environ, GIT_NO_REPLACE_OBJECTS='1', GIT_OPTIONAL_LOCKS='0', GIT_TERMINAL_PROMPT='0',
        GCM_INTERACTIVE='never', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1',
        NUMEXPR_NUM_THREADS='1', PYTHONDONTWRITEBYTECODE='1', TEMP=str(tmp), TMP=str(tmp))
    versions = {}
    for package in ('cadquery', 'cadquery-ocp', 'numpy'):
        try: versions[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError: versions[package] = None
    state = dict(target_sha=args.sha, baseline_sha=BASE, completed=False, reproduction_passed=False,
        commands=[], errors=[], reviewer_sha256=digest(__file__), contract_helper_sha256=digest(COMMON),
        batch_helper_sha256=BATCH_SHA, scope=SCOPE, cad_rebuilt=False, render_replayed=False,
        environment=dict(python=sys.executable, version=sys.version, platform=platform.platform(),
                         pythonpath=env.get('PYTHONPATH'), package_versions=versions, blas_threads=1, temporary_directory=str(tmp)))
    def save(): (destination/'review.json').write_text(json.dumps(state, indent=2, ensure_ascii=False, allow_nan=False)+'\n', encoding='utf-8')
    def run(label, argv, cwd):
        row = dict(label=label, argv=list(map(str, argv)), cwd=str(cwd), completed=False)
        state['commands'].append(row); save(); started = time.monotonic()
        with (destination/(label+'.log')).open('xb') as out, (destination/(label+'.err')).open('xb') as err:
            result = subprocess.run(row['argv'], cwd=cwd, env=env, stdin=subprocess.DEVNULL, stdout=out,
                                    stderr=err, shell=False, creationflags=subprocess.CREATE_NO_WINDOW)
        row.update(completed=True, exit_code=result.returncode, elapsed_s=time.monotonic()-started); save()
        require(result.returncode == 0, 'Failed subprocess: '+label)
    def inside(name): return safe_file(repo, name, repository_path)
    blobs = ImmutableGitBlobs(GIT, ROOT, env, tmp)
    try:
        run('resolve_commit', [GIT, 'rev-parse', args.sha+'^{commit}'], ROOT)
        require((destination/'resolve_commit.log').read_text().strip() == args.sha, 'SHA must identify an exact commit')
        raw = blobs.read(args.sha, MANIFEST); manifest = json.loads(raw); own = manifest_contract(manifest, repository_path)
        require(shutil.disk_usage(destination).free > sum(r['bytes'] for r in manifest['files'].values()) + 256*1024*1024, 'Insufficient D: space for the exact input closure and audit output')
        for count, (name, item) in enumerate(manifest['files'].items(), 1):
            signature = blobs.signature(args.sha, name)
            require(signature['bytes'] == item['bytes'] and signature['sha256'] == item['sha256'], 'Committed manifest mismatch: '+name)
            if name not in own | DOCS:
                require(blobs.signature(BASE, name)['sha256'] == item['sha256'], 'Non-owned dependency changed from baseline: '+name)
            output = repo/repository_path(name); output.parent.mkdir(parents=True, exist_ok=True)
            with output.open('xb') as stream: stream.write(blobs.read(args.sha, name))
            require(digest(output) == item['sha256'], 'Extracted bytes differ: '+name)
            if count % 100 == 0: print(json.dumps(dict(progress='manifest_extraction', checked=count, total=len(manifest['files']))), flush=True)
        (repo/MANIFEST).write_bytes(raw)
        index_name = 'AUDIT_EVIDENCE_INDEX.json'
        require(blobs.signature(args.sha, index_name)['sha256'] == blobs.signature(BASE, index_name)['sha256'], 'Frozen evidence index changed')
        evidence = frozen_evidence(json.loads(blobs.read(args.sha, index_name)))
        for name, expected in evidence.items():
            require(blobs.signature(args.sha, name)['sha256'] == expected == blobs.signature(BASE, name)['sha256'], 'Frozen 222 evidence changed: '+name)
        require(blobs.signature(args.sha, 'CHECKPOINT.json')['sha256'] == blobs.signature(BASE, 'CHECKPOINT.json')['sha256'], 'Existing checkpoint pointer changed')
        state.update(manifest_sha256=digest(repo/MANIFEST), manifest_files_checked=len(manifest['files']),
            frozen_evidence_files_checked=222, frozen_evidence_materialized=False, checkpoint_unchanged=True,
            immutable_blob_reads=blobs.stats(), remaining_blockers=manifest['remaining_blockers'])
        frozen = {n: digest(inside(n)) for n in set(manifest['files']) | {MANIFEST}}
        old, oldguard = package_contract(repo, manifest, repository_path, ROOT)
        state['committed_summary'] = old['summary']; save()
        audit_output = destination/'assembly_audit.json'
        run('assembly_audit', [sys.executable, '-B', str(repo/CAD/'audit_assembly.py'), '--repo', str(repo), '--output', str(audit_output)], repo)
        new = read(audit_output); newguard = report_guard(new, repo, manifest, repo, repository_path)
        audit_contract(new)
        require(newguard == oldguard, 'Replayed input hash coverage differs')
        comparisons = compare_audits(old, new)
        parts = native_part_comparisons(old, new)
        after = {n: digest(inside(n)) for n in frozen}
        source_unchanged = digest(__file__) == state['reviewer_sha256'] and digest(COMMON) == state['contract_helper_sha256'] and digest(BATCH) == BATCH_SHA
        state.update(completed=True, replayed_summary=new['summary'], comparisons=comparisons,
            part_comparisons=parts, source_hash_guard=dict(start=frozen, end=after, passed=frozen == after),
            reviewer_source_unchanged=source_unchanged, assembly_audit_sha256=digest(audit_output),
            manufacturing_approved=False, complete_audit_replayed=True, sampled_endpoint_count=new['summary']['static_pose_count'],
            sampled_path_observation_count=new['summary']['sampled_hand_path_count'], observation_count=len(new['observations']),
            collision_entity_count=new['summary']['collision_entities'], pilot_only_blanks_checked=10,
            blocked_whole_hand_access=[dict(id=p['id'],approach=k,blocked_parts=s['whole_hand_neutral']['blocked_parts'])
                for p in new['retention'] for k,s in p['access'].items() if not s['whole_hand_neutral']['passed']])
        state['reproduction_passed'] = frozen == after and source_unchanged and all(r['passed'] for r in comparisons + parts)
    except Exception as error:
        state['errors'].append(dict(error=repr(error), traceback=traceback.format_exc()))
    finally:
        blobs.close(abort=True); save()
    print(json.dumps({key: state[key] for key in ('completed', 'reproduction_passed', 'errors')}, ensure_ascii=False))
    return 0 if state['reproduction_passed'] else 1

if __name__ == '__main__': raise SystemExit(main())
