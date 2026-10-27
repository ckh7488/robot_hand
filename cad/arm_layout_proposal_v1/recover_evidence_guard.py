"""Preserve false exit guard and verify restored exact renderer and saved images."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent


def main():
    out = HERE/'reviews/evidence_guard_recovery.json'
    assert not out.exists()
    def sha(file):
        return hashlib.sha256(file.read_bytes()).hexdigest()
    raw_path = HERE/'reviews/evidence.json'
    raw = json.loads(raw_path.read_text(encoding='utf-8-sig'))
    assert raw['completed'] and raw['input_hash_guard'] is False
    sources_match = all(sha(ROOT/f) == h for f, h in raw['source_sha256'].items())
    images_match = all(sha(ROOT/f) == h for f, h in raw['outputs'].items())
    assert sources_match and images_match and len(raw['outputs']) == 3
    renderer = HERE/'render_proposal.py'
    expected = [h for f, h in raw['source_sha256'].items() if f.replace('\\', '/').endswith('/render_proposal.py')]
    assert expected == [sha(renderer)]
    history = HERE/'history/render_executed_exact.py.txt'
    assert not history.exists()
    history.parent.mkdir(exist_ok=True)
    history.write_bytes(renderer.read_bytes())
    source = dict(raw['source_sha256'])
    source[raw_path.relative_to(ROOT).as_posix()] = sha(raw_path)
    source[history.relative_to(ROOT).as_posix()] = sha(history)
    source[Path(__file__).relative_to(ROOT).as_posix()] = sha(Path(__file__))
    result = dict(completed=True, source_sha256=source, input_hash_guard=True,
        initial_renderer_exit_guard=False, original_report_modified=False,
        restored_renderer_matches_original_recorded_sha256=True,
        all_original_three_image_hashes_match=True, new_render_or_native_or_exports=0,
        explanation='An attempted extra conflict-image edit changed render_proposal.py during its original run and made the exit guard false. Its original executed code was restored byte-for-byte to the SHA already recorded in the raw report. Original three outputs and every source hash now match. The added conflict picture was instead made by separate render_conflict.py. This is a metadata continuity recovery, not an initial true guard or image/pixel replay.')
    out.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print('restored exact renderer and original three outputs', flush=True)


if __name__ == '__main__':
    main()
