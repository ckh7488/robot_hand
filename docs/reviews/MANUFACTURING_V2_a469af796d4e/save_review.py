"""Preserve a successful manufacturing exact-SHA replay in one follow-up commit.

CLI [--candidate FULL_SHA] [--review D_WORK_REVIEWS/MANUFACTURING_V2_<sha12>]
    [--state NEW_WORK_JSON]
Defaults resolve the pushed candidate from this folder's publish_state.json.
No execution on import. Only the new named docs/reviews folder is staged.
"""
from pathlib import Path
import argparse, hashlib, json, os, re, subprocess, sys, traceback
from checkpoint_contract import (BASE, CAD, MANIFEST, AUDIT, GIT, BATCH_SHA, REMOTE, SCOPE,
    require, read, digest, manifest_contract, report_guard, audit_contract, compare_audits, native_part_comparisons)

ROOT = Path('D:/robot_hand').resolve(); WORK = Path(__file__).resolve().parent
REVIEWS = WORK.parent/'reviews'; PUBLISHED = WORK/'publish_state.json'; SNAPSHOT = WORK/'git_state.json'
RUNNER = WORK/'review_candidate.py'; PUBLISHER = WORK/'publish_candidate.py'; COMMON = WORK/'checkpoint_contract.py'
BATCH = WORK.parent/'gesture002/git_blob_batch.py'

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate'); parser.add_argument('--review', type=Path)
    parser.add_argument('--state', type=Path, default=WORK/'review_publish_state.json')
    args = parser.parse_args(); published = read(PUBLISHED)
    candidate = args.candidate or published.get('candidate_sha')
    require(isinstance(candidate, str) and re.fullmatch('[0-9a-f]{40}', candidate), 'Full candidate SHA required')
    directory = (args.review or REVIEWS/('MANUFACTURING_V2_'+candidate[:12])).resolve()
    require(directory == REVIEWS.resolve()/('MANUFACTURING_V2_'+candidate[:12]) and directory.is_dir(), 'Wrong exact-SHA review directory')
    statepath = args.state.resolve()
    require(statepath.is_relative_to(WORK) and statepath.suffix == '.json' and not statepath.exists(), 'Use a new work state JSON')
    for key in ('GIT_DIR', 'GIT_WORK_TREE', 'GIT_INDEX_FILE'): require(not os.environ.get(key), 'Refuse inherited '+key)
    require(digest(BATCH) == BATCH_SHA, 'Batch helper changed')
    sys.dont_write_bytecode = True; sys.path.insert(0, str(BATCH.parent))
    from git_blob_batch import ImmutableGitBlobs, repository_path
    env = dict(os.environ, GIT_NO_REPLACE_OBJECTS='1', GIT_OPTIONAL_LOCKS='0', GIT_TERMINAL_PROMPT='0',
        GCM_INTERACTIVE='never', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1',
        NUMEXPR_NUM_THREADS='1', PYTHONDONTWRITEBYTECODE='1', TEMP=str(WORK), TMP=str(WORK))
    state = dict(candidate_sha=candidate, phase='preflight', review_commit=None, pushed=False, errors=[], writer_sha256=digest(__file__))
    blobs = ImmutableGitBlobs(GIT, ROOT, env, WORK)
    def save(): statepath.write_text(json.dumps(state, indent=2, ensure_ascii=False, allow_nan=False)+'\n', encoding='utf-8')
    def git(*argv):
        result = subprocess.run([GIT, *argv], cwd=ROOT, env=env, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, shell=False, creationflags=subprocess.CREATE_NO_WINDOW)
        require(result.returncode == 0, 'Git '+argv[0]+': '+result.stderr.decode('utf-8', 'replace')); return result.stdout
    def text(*args): return git(*args).decode('utf-8').strip()
    def names(*args): return {p.decode('utf-8') for p in git(*args).split(b'\0') if p}
    def file_signature(name):
        p = ROOT/repository_path(name)
        if not p.exists(): return dict(exists=False)
        require(p.resolve() == p and p.is_file(), 'Aliased/non-file preservation target: '+name)
        return dict(exists=True, bytes=p.stat().st_size, sha256=digest(p))
    def preserve():
        for name, expected in preserved.items(): require(file_signature(name) == expected, 'Preserved file changed: '+name)
        require(digest(ROOT/'CHECKPOINT.json') == pointer_sha, 'CHECKPOINT changed')
        for name, expected in candidate_hashes.items(): require(digest(ROOT/name) == expected, 'Committed candidate/input changed: '+name)
        for path, expected in source_hashes.items(): require(digest(path) == expected, 'Evidence/helper source changed: '+str(path))
    try:
        require(published.get('pushed') is True and published.get('phase') == 'pushed' and not published.get('errors') and
                published.get('candidate_sha') == candidate == published.get('origin_main'), 'Candidate publication is incomplete')
        require(text('rev-parse', 'HEAD') == candidate and text('symbolic-ref', '--short', 'HEAD') == 'main', 'Expected main at candidate SHA')
        require(not names('diff', '--cached', '--name-only', '-z') and not git('ls-files', '-u', '-z'), 'Index must be empty and unconflicted')
        urls = text('remote', 'get-url', '--push', '--all', 'origin').splitlines()
        require(len(urls) == 1 and urls[0] in REMOTE and text('remote', 'get-url', 'origin') in REMOTE, 'Unexpected origin URL')
        require(text('ls-remote', 'origin', 'refs/heads/main').split() == [candidate, 'refs/heads/main'], 'Remote main differs')
        review_path = directory/'review.json'; audit_path = directory/'assembly_audit.json'
        log_names = ('resolve_commit.log', 'resolve_commit.err', 'assembly_audit.log', 'assembly_audit.err')
        source_hashes = {p: digest(p) for p in (review_path, audit_path, RUNNER, PUBLISHER, COMMON, BATCH,
                                               PUBLISHED, SNAPSHOT, Path(__file__), *(directory/n for n in log_names))}
        review = read(review_path); audit = read(audit_path); manifest = json.loads(blobs.read(candidate, MANIFEST))
        manifest_contract(manifest, repository_path)
        require(review.get('target_sha') == candidate and review.get('baseline_sha') == BASE and review.get('completed') is True and
                review.get('reproduction_passed') is True and not review.get('errors'), 'Wrong-target/incomplete/failed replay')
        require(review.get('reviewer_sha256') == source_hashes[RUNNER] and review.get('contract_helper_sha256') == source_hashes[COMMON] and
                review.get('batch_helper_sha256') == BATCH_SHA and review.get('reviewer_source_unchanged') is True, 'Replayed helper identity differs')
        require(published['writer_sha256'] == source_hashes[PUBLISHER] and published['contract_helper_sha256'] == source_hashes[COMMON] and
                published['batch_helper_sha256'] == BATCH_SHA and published['preservation_snapshot_sha256'] == source_hashes[SNAPSHOT], 'Publisher/helper preservation identity differs')
        manifest_sha = blobs.signature(candidate, MANIFEST)['sha256']
        require(review['manifest_sha256'] == published['manifest_sha256'] == manifest_sha and review['assembly_audit_sha256'] == source_hashes[audit_path], 'Manifest/audit output identity differs')
        commands = review.get('commands', [])
        require([c['label'] for c in commands] == ['resolve_commit', 'assembly_audit'] and
                all(c.get('completed') is True and c.get('exit_code') == 0 for c in commands), 'Missing successful full saved-CAD audit replay')
        require(review.get('scope') == SCOPE and review.get('cad_rebuilt') is False and review.get('render_replayed') is False and
                review.get('complete_audit_replayed') is True and review.get('sampled_endpoint_count') == 10 and
                review.get('sampled_path_observation_count') == 48 and review.get('observation_count') == 58 and
                review.get('collision_entity_count') == 319 and review.get('pilot_only_blanks_checked') == 10 and
                review.get('manufacturing_approved') is False and review.get('frozen_evidence_files_checked') == 222, 'Reproduction scope differs')
        expected = {n: item['sha256'] for n, item in manifest['files'].items()}; expected[MANIFEST] = manifest_sha
        guard = review.get('source_hash_guard', {})
        require(guard.get('passed') is True and guard.get('start') == expected == guard.get('end'), 'Replay immutable closure guard differs')
        fresh = directory/'repository'
        for name, sha in expected.items():
            repository_path(name)
            require(blobs.signature(candidate, name)['sha256'] == sha == digest(fresh/name), 'Committed/replayed input differs: '+name)
        old = json.loads(blobs.read(candidate, AUDIT))
        audit_contract(old); audit_contract(audit)
        require(report_guard(old, ROOT, manifest, fresh, repository_path) == report_guard(audit, fresh, manifest, fresh, repository_path), 'Replay input coverage differs')
        recalculated = compare_audits(old, audit)
        require(json.loads(json.dumps(recalculated, allow_nan=False)) == review['comparisons'] and all(r['passed'] for r in recalculated), 'Reported collision/insertion comparisons differ')
        require(review['committed_summary'] == old['summary'] == review['replayed_summary'] == audit['summary'] and
                review['remaining_blockers'] == manifest['remaining_blockers'], 'Summary/remaining blockers differ')
        part_comparisons=native_part_comparisons(old,audit)
        require(len(part_comparisons)==359 and part_comparisons==review['part_comparisons'] and all(p['passed'] for p in part_comparisons), 'Incomplete/incorrect native-part comparison')
        blocked=[dict(id=p['id'],approach=k,blocked_parts=s['whole_hand_neutral']['blocked_parts'])
            for p in audit['retention'] for k,s in p['access'].items() if not s['whole_hand_neutral']['passed']]
        require(review['blocked_whole_hand_access']==blocked, 'Whole-hand blockers omitted/changed')
        snapshot = read(SNAPSHOT)
        require(snapshot['head'] == BASE and len(snapshot['dirty']) == 187 and not snapshot['prior_changed_bytes'], 'Original preservation snapshot differs')
        preserved = dict(published['preserved_files'])
        for row in snapshot['dirty']:
            require(preserved.get(row['path']) == dict(exists=True, bytes=row['bytes'], sha256=row['sha256']), 'Original dirty bytes missing from publisher state')
        relative = 'docs/reviews/MANUFACTURING_V2_'+candidate[:12]; destination = ROOT/relative
        require(not destination.exists() and not any(n.startswith(relative+'/') for n in names('ls-tree', '-r', '--name-only', '-z', candidate)), 'Preserve existing review directory')
        for name in names('ls-files', '--modified', '--deleted', '--others', '--exclude-standard', '-z'):
            require(not name.startswith(relative+'/'), 'Review output already has local work')
            actual = file_signature(name)
            if name in preserved: require(preserved[name] == actual, 'Preserved dirty work changed: '+name)
            else: preserved[name] = actual
        pointer_sha = blobs.signature(candidate, 'CHECKPOINT.json')['sha256']
        require(pointer_sha == blobs.signature(BASE, 'CHECKPOINT.json')['sha256'], 'Candidate changed existing checkpoint')
        candidate_hashes = expected; preserve()
        summary = audit['summary']; blockers = '\n'.join('- '+text for text in manifest['remaining_blockers'])
        body = f'''# 제작 후보 — 정확 SHA 저장 CAD 재검사

대상 커밋: `{candidate}`

커밋 manifest의 전체 입력을 새 D: 사본에 정확한 바이트로 읽고, 저장된 native STEP 부품359개와 별도 드릴 가공용 블랭크10개를 다시 검사했다. 블랭크는 완성 부품이나 조립 질량에 포함하지 않았다. 원본 자료222개의 해시도 대상 커밋과 기존 기준에서 확인했다. CAD 재생성이나 이미지 재렌더는 수행하지 않았다.

원위 핀10세트마다 핀·받침 와셔·나사의 공칭 접근과 육각/일자 공구 접근 다섯 종류를 재계산했다. 자기 손가락 부분조립의 막힘은 {summary['finger_module_blocked']}이며, 완성 손 중립자세의 막힘은 {summary['whole_hand_neutral_blocked']}로 원보고서와 일치했다. 공구를 크게 만든 양성대조20건도 재현했다. 완성 손 상태의 막힘은 해소된 것으로 표시하지 않으며 부분조립 순서의 추가 검증 대상으로 남긴다.

핀·받침·나사·기존 양쪽 와셔의 다섯 부품을 포함한 회전/축방향 자유도 포락체10개와 나머지309부품을 합쳐319개 충돌 대상으로 검사했다. 세트 내부 명목10쌍과 상관된 축방향 이동의 네 꼭짓점도 검사한다. 열 가지 정적 자세와 정방향 경로48표본(관절 최대간격5°)에서 보고 기준 이상 겹침 {summary['pose_collision_count']}건, 연산 실패 {summary['operation_failure_count']}건이 재현됐다. 이는 연속 손 동작이나 최소 유격, 편심·기울기·공차, 실제 나사산의 체결, 조임력·풀림·강도, 구동 및100ms 성능의 승인이 아니다. 같은 작업의 새 사본 재현이며 외부 독립 감사나 제조 승인이 아니다.

남은 제작 문제:

{blockers}

- [정확 SHA 재현 보고서](review.json) · [전체 저장 CAD 조립 검사](assembly_audit.json)
- [실행 로그](assembly_audit.log) · [후보 원본](https://github.com/ckh7488/robot_hand/tree/{candidate}/cad/manufacturing_v2)
- 검토 실행기 SHA256: `{source_hashes[RUNNER]}`
- 공용 검증 계약 SHA256: `{source_hashes[COMMON]}`
- 이 후속 저장기 SHA256: `{state['writer_sha256']}`

기존 CHECKPOINT와 PROGRESS 작업 사본, 무관한 기존 변경 파일은 보존했다. 이 커밋에는 이 검토 폴더만 추가한다.
'''
        contents = {'README.md': body.encode('utf-8'), 'review.json': review_path.read_bytes(), 'assembly_audit.json': audit_path.read_bytes(),
            'publish_state.json': PUBLISHED.read_bytes(), 'git_state.json': SNAPSHOT.read_bytes(),
            'review_candidate.py': RUNNER.read_bytes(), 'publish_candidate.py': PUBLISHER.read_bytes(),
            'checkpoint_contract.py': COMMON.read_bytes(), 'save_review.py': Path(__file__).read_bytes(), 'git_blob_batch.py': BATCH.read_bytes()}
        contents.update({name: (directory/name).read_bytes() for name in log_names})
        require(all(len(raw) < 100*1024*1024 for raw in contents.values()), 'Oversized review evidence')
        preserve()
        require(text('rev-parse', 'HEAD') == candidate and not names('diff', '--cached', '--name-only', '-z'), 'HEAD/index changed before writing')
        destination.mkdir(parents=True)
        for name, raw in contents.items():
            with (destination/name).open('xb') as stream: stream.write(raw)
        staged = {relative+'/'+name for name in contents}
        copied = {relative+'/'+name: hashlib.sha256(raw).hexdigest() for name, raw in contents.items()}
        state.update(phase='written_not_staged', files=copied, preserved_original_dirty_files=187,
                     preserved_total_dirty_files=len(preserved), original_candidate_summary=summary); save(); preserve()
        require(text('rev-parse', 'HEAD') == candidate and not names('diff', '--cached', '--name-only', '-z'), 'HEAD/index changed before staging')
        # These exact audit evidence files intentionally include two .log files.
        # Force applies only to this explicit reviewed list, never a directory or wildcard.
        git('-c', 'core.autocrlf=false', '--literal-pathspecs', 'add', '-f', '--', *sorted(staged))
        require(names('diff', '--cached', '--name-only', '-z') == staged, 'Unexpected staged file')
        tree = text('write-tree')
        for name, sha in copied.items(): require(digest(ROOT/name) == sha == blobs.signature(tree, name)['sha256'], 'Copied/staged evidence differs: '+name)
        require(blobs.signature(tree, MANIFEST)['sha256'] == expected[MANIFEST] and
                blobs.signature(tree, 'CHECKPOINT.json')['sha256'] == pointer_sha and
                blobs.signature(tree, 'PROGRESS.md')['sha256'] == blobs.signature(candidate, 'PROGRESS.md')['sha256'], 'Unrelated staged pointer/progress/manifest change')
        git('-c', 'core.whitespace=blank-at-eol,blank-at-eof,space-before-tab,cr-at-eol', 'diff', '--cached', '--check'); preserve()
        require(text('rev-parse', 'HEAD') == candidate and text('symbolic-ref', '--short', 'HEAD') == 'main' and
                names('diff', '--cached', '--name-only', '-z') == staged and text('write-tree') == tree, 'HEAD/index changed before commit')
        git('commit', '-m', 'review(manufacturing): replay saved-CAD assembly checks at '+candidate[:12])
        sha = text('rev-parse', 'HEAD'); state.update(phase='committed_not_pushed', review_commit=sha); save()
        require(text('rev-parse', sha+'^') == candidate and text('rev-parse', sha+'^{tree}') == tree and
                names('diff-tree', '--no-commit-id', '--name-only', '-r', '-z', sha) == staged, 'Committed scope/tree differs; do not push')
        preserve()
        require(text('rev-parse', 'HEAD') == sha and not names('diff', '--cached', '--name-only', '-z') and
                text('remote', 'get-url', '--push', '--all', 'origin').splitlines() == urls, 'HEAD/index/destination changed before push')
        git('push', 'origin', sha+':refs/heads/main')
        require(text('ls-remote', 'origin', 'refs/heads/main').split() == [sha, 'refs/heads/main'], 'Remote SHA differs after push')
        state.update(phase='pushed', pushed=True, origin_main=sha, checkpoint_unchanged=True, progress_workcopy_unchanged=True)
    except Exception as error:
        state['errors'].append(dict(error=repr(error), traceback=traceback.format_exc()))
    finally:
        blobs.close(abort=True); save()
    print(json.dumps(state, ensure_ascii=False)); return 0 if state['pushed'] else 1

if __name__ == '__main__': raise SystemExit(main())
