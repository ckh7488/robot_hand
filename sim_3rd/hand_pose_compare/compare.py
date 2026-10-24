"""Evaluate independent WiLoR inference against already captured MediaPipe frames.

Private photos and model outputs remain in the explicit output directory.
No video is captured, uploaded, or sent to a robot by this program.
"""
from pathlib import Path
import argparse
import hashlib
import json
import sys
import time


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def serializable(value):
    import numpy as np
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {k:serializable(v) for k,v in value.items()}
    if isinstance(value, (tuple,list)):
        return [serializable(v) for v in value]
    return value


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--upstream', type=Path, required=True)
    parser.add_argument('--chumpy', type=Path, required=True)
    parser.add_argument('--weights', type=Path, required=True)
    parser.add_argument('--cases', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--device', default='cuda:0')
    parser.add_argument('--repeats', type=int, default=3)
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error('--repeats must be at least 1')
    sys.path[:0] = [str(args.upstream), str(args.chumpy)]
    import cv2
    import numpy as np
    import torch
    from wilor_adapter import LocalWiLoR
    from render_comparison import render_case
    from render_mesh import render_mesh_case
    torch.set_num_threads(1)
    args.output.mkdir(parents=True, exist_ok=True)
    cases = sorted(args.cases.glob('case_*/case.json'))
    if not cases:
        raise ValueError('No saved cases')
    start = time.perf_counter()
    model = LocalWiLoR(weights=args.weights, device=args.device, verbose=False)
    print('MODEL LOADED', time.perf_counter()-start, 's', flush=True)
    summary = dict(schema=1, model='WiLoR-mini', dtype='float32', input='unmodified saved BGR camera.png',
                   selection='largest independently detected hand; confidence >= 0.3',
                   device=str(model.device), gpu=torch.cuda.get_device_name() if torch.cuda.is_available() else None,
                   torch=torch.__version__, checkpoint_keys=model.checkpoint_keys,
                   comparison='saved MediaPipe predictions from same image; no 3D ground truth',
                   timing='synchronized detector+pose including pre/postprocess; excludes disk IO/model loading',
                   cases=[])
    first = cv2.imread(str(cases[0].parent/'camera.png'))
    if first is None:
        raise ValueError('Cannot load warmup image')
    warmup = model.infer_one(first)
    summary['cold_first_inference_ms'] = warmup['elapsed_ms']
    print('WARMUP', warmup['status'], warmup['elapsed_ms'], flush=True)
    for case_path in cases:
        case = json.loads(case_path.read_text(encoding='utf-8'))
        frame_path = case_path.parent/'camera.png'
        frame = cv2.imread(str(frame_path))
        if frame is None:
            raise ValueError(f'Cannot read {frame_path}')
        out = args.output/case_path.parent.name
        out.mkdir(exist_ok=True)
        attempts = [model.infer_one(frame) for _ in range(args.repeats)]
        result = attempts[0]
        result['repeated_elapsed_ms'] = [r['elapsed_ms'] for r in attempts]
        result['median_elapsed_ms'] = float(np.median(result['repeated_elapsed_ms']))
        result['source_image_sha256'] = digest(frame_path)
        result['source_case_sha256'] = digest(case_path)
        result['source_case'] = str(case_path)
        result['repeats_status'] = [r['status'] for r in attempts]
        if result['status'] == 'tracked':
            initial = result['result']['wilor_preds']['pred_keypoints_3d']
            result['repeat_max_joint_difference_m'] = max(float(np.max(np.abs(r['result']['wilor_preds']['pred_keypoints_3d']-initial))) for r in attempts if r['status']=='tracked')
        (out/'prediction.json').write_text(json.dumps(serializable(result), indent=2), encoding='utf-8')
        render_case(case, frame, result, out)
        render_mesh_case(case, frame, result, model.wilor_model.mano.faces, out)
        row = {k:v for k,v in result.items() if k not in ('result','source_case')}
        row['case_id'] = case_path.parent.name
        summary['cases'].append(row)
        (args.output/'summary.json').write_text(json.dumps(serializable(summary), indent=2), encoding='utf-8')
        print('CASE COMPLETE', row['case_id'], result['status'], round(result['median_elapsed_ms'],1), 'ms', flush=True)
    if torch.cuda.is_available():
        summary['cuda_peak_allocated_bytes'] = torch.cuda.max_memory_allocated()
        summary['cuda_peak_reserved_bytes'] = torch.cuda.max_memory_reserved()
    (args.output/'summary.json').write_text(json.dumps(serializable(summary), indent=2), encoding='utf-8')
    print('COMPLETE', flush=True)


if __name__ == '__main__':
    main()
