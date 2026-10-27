"""Correct aggregate bounds from saved per-part bounds, without CAD replay."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent


def main():
    raw_path = HERE/'reviews/module_survey.json'
    out = HERE/'reviews/module_survey_corrected.json'
    assert not out.exists()
    raw = json.loads(raw_path.read_text(encoding='utf-8-sig'))
    assert raw['completed'] and raw['input_hash_guard']
    corrected = []
    for group in raw['bundles']:
        boxes = [z['bbox_mm'] for z in group['rows']]
        assert all(len(b) == 2 and all(len(v) == 3 for v in b) for b in boxes)
        bounds = [[min(b[0][i] for b in boxes) for i in range(3)],
                  [max(b[1][i] for b in boxes) for i in range(3)]]
        corrected.append(dict(group, canonical_bbox_mm=bounds))
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    result = dict(raw, bundles=corrected,
        initial_aggregate_bbox_valid=False,
        recovery='Initial ndarray aggregate used Nx2x3 as Nx6. Per-part exact CAD bounds were valid and saved. Reaggregate only these409 stored boxes; no CAD load/native/bbox/export rerun.',
        initial_report_sha256=digest(raw_path),
        new_CAD_or_native_executions=0)
    result['source_sha256'] = dict(raw['source_sha256'])
    result['source_sha256'][raw_path.relative_to(ROOT).as_posix()] = digest(raw_path)
    result['source_sha256'][Path(__file__).relative_to(ROOT).as_posix()] = digest(Path(__file__))
    assert all(digest(ROOT/f) == h for f, h in result['source_sha256'].items())
    out.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print([(z['name'], z['canonical_bbox_mm']) for z in corrected], flush=True)


if __name__ == '__main__':
    main()
