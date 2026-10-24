"""Capture, mapping, CAD and UI orchestration."""
import argparse
from pathlib import Path
import sys
import time
import traceback
import numpy as np
from .cad_hand import CadHand
from .diagnostics import SessionLog
from .pipeline import PosePipeline
from .sources import LatestFrameCamera, VideoFileSource, SyntheticSource
PROJECT = Path(__file__).resolve().parents[1]

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--repo', type=Path, default=PROJECT.parents[1])
    ap.add_argument('--source', default='0')
    ap.add_argument('--cad', default='webcam_articulation_v1',
                    choices=('webcam_articulation_v1', 'form_candidate_v1'))
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--model', type=Path, default=PROJECT / 'models' / 'hand_landmarker.task')
    ap.add_argument('--log-dir', type=Path, default=Path('D:/Codex-work/robot-hand-workspace/work/hand_follow_sessions'))
    ap.add_argument('--calibration', type=Path,
                    help='optional profile; the new CAD uses a separate default profile')
    ap.add_argument('--no-gui', action='store_true')
    ap.add_argument('--whole', action='store_true', help='include reference forearm')
    ap.add_argument('--seconds', type=float, default=0)
    ap.add_argument('--beta', type=float, default=0.025)
    ap.add_argument('--min-cutoff', type=float, default=2.0)
    ap.add_argument('--swap-handedness', action='store_true', help='diagnostic override only')
    ap.add_argument('--retarget', choices=('position', 'angles'), default='position')
    ap.add_argument('--full-quality', action='store_true', help='full-size CAD render with shadows')
    a = ap.parse_args(argv)
    if a.calibration is None:
        a.calibration = PROJECT / (f'calibration_pose_v2_{a.cad}.json' if a.retarget == 'position' else
            'calibration_articulation_v1.json' if a.cad == 'webcam_articulation_v1' else 'calibration_v3.json')
    robot = tracker = source = ui = log = None
    error = None
    try:
        robot = CadHand(a.repo, whole=a.whole, cad=a.cad)
        pipeline = PosePipeline(robot.design, robot.ranges, a.calibration, a.beta, a.min_cutoff, a.retarget)
        log = SessionLog(a.log_dir, dict(
            source=a.source, requested_fps=a.fps, cad_source=str(robot.source),
            cad_hashes=robot.hashes, preview='kinematic', calibration=str(a.calibration),
            calibration_snapshot=dict(offset_deg=pipeline.calibration.offset,
                                      gain=pipeline.calibration.gain,
                                      estimated_dip_indices=pipeline.calibration.coupled_dips,
                                      estimated_dip_ratios=pipeline.calibration.dip_ratios,
                                      position_profile=pipeline.calibration.position_profile),
            finger_mapping=a.retarget, render_quality='full' if a.full_quality else 'fast',
            thumb_mapping='position_fit_with_design_joint_frames',
            filter=dict(beta=a.beta, min_cutoff=a.min_cutoff)))
        if a.source == 'synthetic':
            source = SyntheticSource(fps=a.fps, noise_m=0)
        else:
            import cv2
            cv2.setNumThreads(1)
            from .tracker import HandTracker
            tracker = HandTracker(a.model)
            source = LatestFrameCamera(int(a.source), a.width, a.height, a.fps, backend=cv2.CAP_DSHOW if sys.platform == 'win32' else None) if a.source.isdigit() else VideoFileSource(a.source)
        if not a.no_gui:
            from .ui import PreviewUI
            ui = PreviewUI()
        seq = -1
        started = last_frame = time.perf_counter()
        previous_loop = started
        processing_fps, view, debug, testing = (float(a.fps), 1, False, False)
        while not a.seconds or time.perf_counter() - started < a.seconds:
            if (log.directory / 'stop.request').exists():
                break
            seq, captured, frame = source.read(seq, timeout=0.2)
            now = time.perf_counter()
            if frame is None:
                if a.source != 'synthetic' and (not a.source.isdigit()):
                    break
                timeout = 15 if seq < 0 else 4
                if now - last_frame > timeout:
                    raise RuntimeError(f'Webcam delivered no frames for {timeout} seconds. Check camera access and other camera apps.')
                if ui:
                    import cv2
                    if cv2.waitKey(1) & 255 in (ord('q'), 27):
                        break
                continue
            last_frame = now
            tracking_start = time.perf_counter()
            if tracker is None:
                detection, image = (frame, None)
            else:
                image = cv2.flip(frame, 1)
                detection = tracker.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), max(0, (captured - started) * 1000))
            tracking_end = time.perf_counter()
            if detection is not None and a.swap_handedness:
                detection['handedness'] = 'Left' if detection['handedness'] == 'Right' else 'Right'
            capturing = pipeline.capture_kind
            pose = pipeline.update(detection, captured)
            if capturing and pipeline.capture_kind is None:
                log.event('calibration_finished', message=pipeline.message,
                          offset_deg=pipeline.calibration.offset, gain=pipeline.calibration.gain,
                          estimated_dip_indices=pipeline.calibration.coupled_dips,
                          estimated_dip_ratios=pipeline.calibration.dip_ratios,
                          position_profile=pipeline.calibration.position_profile)
            if testing:
                pose['command_deg'] = pipeline.fist.copy()
            robot.set(pose['command_deg'])
            command_time = time.perf_counter()
            processing_fps = 0.9 * processing_fps + 0.1 / max(command_time - previous_loop, 0.001)
            previous_loop = command_time
            timing = dict(camera_fps=source.measured_fps(), processing_fps=processing_fps, track_ms=(tracking_end - tracking_start) * 1000, age_ms=(command_time - captured) * 1000)
            mode = 'test' if testing else 'live'
            render_start = time.perf_counter()
            rendered = robot.render(view, width=880 if a.full_quality else 660,
                                    height=760 if a.full_quality else 570, fast=not a.full_quality) if ui else None
            render_end = time.perf_counter()
            key = ui.show(image, detection, rendered, pose, pipeline, timing, mode, view, debug) if ui else 255
            timing.update(map_ms=(command_time-tracking_end)*1000,
                          render_ms=(render_end-render_start)*1000,
                          ui_ms=(time.perf_counter()-render_end)*1000)
            log.frame(seq, captured, command_time, detection, pose, robot.angles, timing, mode)
            if key in (ord('q'), 27):
                break
            elif key in (ord('1'), ord('2'), ord('3')):
                view = key - ord('1')
            elif key in (ord('c'), ord('v')) and (not testing):
                pipeline.start_calibration('open' if key == ord('c') else 'fist')
                log.event('calibration_requested', kind=pipeline.capture_kind)
            elif key == ord('x'):
                pipeline.clear_calibration()
                log.event('calibration_reset')
            elif key == ord('f'):
                pipeline.filter_on = not pipeline.filter_on
                pipeline.filter.reset()
            elif key == ord('h') and tracker:
                tracker.reset_selection()
                pipeline.filter.reset()
            elif key == ord('d'):
                debug = not debug
            elif key == ord('t'):
                testing = not testing
                log.event('test_pose' if testing else 'live_pose')
        if log.frames == 0:
            raise RuntimeError('No camera frames received during this run. Allow at least 15 seconds for camera startup.')
    except BaseException:
        error = traceback.format_exc()
        raise
    finally:
        for resource in (source, tracker, ui, robot):
            if resource is not None:
                resource.close()
        if log:
            log.close(error)
            print(f'Session: {log.directory}', flush=True)
