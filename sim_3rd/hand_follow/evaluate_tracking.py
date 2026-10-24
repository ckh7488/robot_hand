"""Replay a private numeric session through both mappings, using identical times.

Outputs contain aggregate evidence and CAD renders only by default. Calibration
profiles and optional replay traces stay in the caller's private output folder.
Run with --session SESSION --output PRIVATE_OUTPUT [--example-time SECONDS].
"""
import argparse
import hashlib
import json
from pathlib import Path
import time
import numpy as np
from hand_follow.fingers import observations, geometry_signature
from hand_follow.pipeline import PosePipeline
from hand_follow.retarget import Calibration, NAMES


def plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def calibration_from_session(rows, events, design):
    """Use the most recent completed C then V; never infer endpoints from replay."""
    windows, pending = {}, None
    for event in events:
        if event['event'] == 'calibration_reset':
            windows, pending = {}, None
        elif event['event'] == 'calibration_requested':
            pending = event if event.get('kind') in ('open', 'fist') else None
        elif event['event'] == 'calibration_finished' and pending:
            # Historical log schema has a message instead of a success flag.
            if '완료' in event.get('message', ''):
                if pending['kind'] == 'open':
                    windows = {}
                windows[pending['kind']] = (pending, event)
            pending = None
    if set(windows) != {'open', 'fist'}:
        raise ValueError('Session needs a completed C followed by V calibration')
    last = windows['fist'][1]
    calibration = Calibration(np.array(last['offset_deg']), np.array(last['gain']),
        last.get('estimated_dip_indices', []),
        {int(k): v for k, v in last.get('estimated_dip_ratios', {}).items()})
    profile = dict(schema=1, cad_signature=geometry_signature(design))
    evidence = {}
    for name, (request, finished) in windows.items():
        low = max(request['elapsed_s'] + .5, finished['elapsed_s'] - 1.4)
        high = finished['elapsed_s'] + .05  # completion is logged just before its frame
        samples = [r for r in rows if low <= r['elapsed_s'] <= high and
                   r['mode'] == 'live' and r['pose']['tracked']]
        if len(samples) < 5:
            raise ValueError(f'Too few logged {name} calibration samples')
        profile[name+'_tips'] = np.median([observations(
            r['detection']['world'], r['detection']['handedness'])['tips'] for r in samples], axis=0).tolist()
        if name == 'fist':
            profile['fist_flex_deg'] = np.median([r['pose']['raw_deg'][:16] for r in samples],
                axis=0).reshape(4,4)[:,1:].tolist()
        evidence[name] = dict(window_s=[low,high], logged_samples=len(samples))
    return calibration, profile, evidence, last['elapsed_s']+.05


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--example-time', type=float)
    parser.add_argument('--save-trace', action='store_true')
    args = parser.parse_args()
    out = args.output; out.mkdir(parents=True, exist_ok=True)
    rows = [json.loads(line) for line in (args.session/'landmarks.jsonl').read_text(encoding='utf-8').splitlines()]
    events = [json.loads(line) for line in (args.session/'events.jsonl').read_text(encoding='utf-8').splitlines()]
    design_path = args.repo/'cad/webcam_articulation_v1/design.json'
    design = json.loads(design_path.read_text(encoding='utf-8'))
    metadata = json.loads((args.session/'session.json').read_text(encoding='utf-8'))
    for relative, expected in metadata['cad_hashes'].items():
        if digest(args.repo/relative.replace('\\','/')) != expected:
            raise ValueError(f'Replay CAD differs from recorded CAD: {relative}')
    ranges = np.array([design['joint_ranges_deg'][n if n.startswith('thumb') else n.split('_',1)[1]] for n in NAMES])
    cal, profile, windows, start = calibration_from_session(rows, events, design)
    cal.save(out/'calibration_angles.json')
    cal.position_profile = profile; cal.save(out/'calibration_pose_v2.json')
    before = PosePipeline(design,ranges,out/'calibration_angles.json',retarget_mode='angles')
    after = PosePipeline(design,ranges,out/'calibration_pose_v2.json',retarget_mode='position')
    replay, durations = [], []
    for row in rows:
        if row['elapsed_s'] < start:
            continue
        if row['mode'] != 'live':
            # Test-mode frames are not measurements of webcam-driven motion.
            before.update(None,row['elapsed_s'],row['elapsed_s'])
            after.update(None,row['elapsed_s'],row['elapsed_s'])
            continue
        old = before.update(row['detection'],row['elapsed_s'],row['elapsed_s'])
        begin = time.perf_counter()
        pose = after.update(row['detection'],row['elapsed_s'],row['elapsed_s'])
        if pose['tracked']:
            durations.append((time.perf_counter()-begin)*1000)
        replay.append(dict(time=row['elapsed_s'],before=old['command_deg'],
                           after=pose['command_deg'],pose=pose))
    closed = [r for r in replay if r['pose']['tracked'] and
              min(f['progress'] for f in r['pose']['finger_fit']['fingers']) > .90]
    if len(closed) < 2:
        raise ValueError('Not enough closed-hand replay frames to compare')
    summary = dict(schema=1, frames=len(replay), closed_frames=len(closed),
        calibration_samples=windows, joint_names=NAMES,
        scope='Both modes use identical logged detections/timestamps and filters. Sampling <=10 Hz; no camera pixels. '
              'Closed selection: all four position-progress values >0.90. Adjacent change includes real motion and joint saturation; not a stationary jitter/accuracy measurement.',
        baseline='Legacy angle mapping in current pipeline; analytic thumb solver uses the same objective as prior revision.',
        limitations=['Point fingertip palm guard only; no mesh collision/contact guarantee.',
                     'Closed workspace is anchored to the CAD authored fist; not anatomical ground truth.',
                     'Calibration reconstructed from decimated numeric records; not exactly the original full-rate C/V samples.'],
        input_sha256={name:digest(args.session/name) for name in ('landmarks.jsonl','events.jsonl','session.json')},
        source_sha256={str(p.relative_to(args.repo)).replace('\\','/'):digest(p)
                       for p in sorted((Path(__file__).parent/'hand_follow').glob('*.py'))},
        evaluator_sha256=digest(Path(__file__)), design_sha256=digest(design_path),
        mapping_ms_p50_p90_p99=np.percentile(durations,[50,90,99]),
        closed_before_median=np.median([r['before'] for r in closed],axis=0),
        closed_after_median=np.median([r['after'] for r in closed],axis=0),
        finger_tip_error_mm_p50_p90=np.percentile([f['tip_error_mm'] for r in closed
            for f in r['pose']['finger_fit']['fingers']],[50,90]),
        closed_active_limit_fraction={name:float(np.mean([j in r['pose']['finger_fit']['active_limits']
            for r in closed])) for j,name in enumerate(NAMES[:16]) if j%4})
    for label in ('before','after'):
        steps = [np.abs(np.array(a[label])-b[label]) for a,b in zip(closed,closed[1:])
                 if 0 < b['time']-a['time'] < .25]
        summary['adjacent_closed_pairs'] = len(steps)
        summary[label+'_closed_step_p95_deg'] = np.percentile(steps,95,axis=0) if steps else None
    example = min(closed,key=lambda r:abs(r['time']-args.example_time)) if args.example_time is not None else closed[len(closed)//2]
    summary['example'] = dict(time=example['time'],before=example['before'],after=example['after'])
    (out/'summary.json').write_text(json.dumps(summary,indent=2,default=plain),encoding='utf-8')
    if args.save_trace:
        (out/'replay.json').write_text(json.dumps(replay,default=plain),encoding='utf-8')
    from hand_follow.cad_hand import CadHand
    from PIL import Image, ImageDraw, ImageFont
    robot = CadHand(args.repo,cad='webcam_articulation_v1')
    font_path = Path('C:/Windows/Fonts/malgun.ttf')
    font = ImageFont.truetype(str(font_path),26) if font_path.exists() else ImageFont.load_default()
    small = ImageFont.truetype(str(font_path),20) if font_path.exists() else ImageFont.load_default()
    try:
        for view,name in ((1,'fist'),(2,'side')):
            board = Image.new('RGB',(1760,860),'#141a24'); draw = ImageDraw.Draw(board)
            for i,label in enumerate(('before','after')):
                robot.set(example[label])
                board.paste(Image.fromarray(robot.render(view,880,760,focus=[.012,.078,.04],distance=.36)),(i*880,60))
                draw.text((24+i*880,16),['이전 각도 확대','손끝 위치 + 굽힘 맞춤'][i],font=font,fill='#edf5ff')
            draw.text((24,826),f"같은 입력 {example['time']:.2f}초 · 같은 CAD와 카메라 · 기록 재생 비교 · 실시간 체감 확인 필요",font=small,fill='#a8c4d5')
            board.save(out/f'before_after_{name}.png')
    finally:
        robot.close()
    print(json.dumps(summary,default=plain),flush=True)


if __name__ == '__main__':
    main()
