"""Capture, mapping, CAD and UI orchestration."""
import argparse
import hashlib
from pathlib import Path
import sys
import time
import traceback
import numpy as np
from .cad_hand import CadHand
from .diagnostics import SessionLog
from .captures import PoseCaseRecorder, capture_displayed_frame, calibration_snapshot
from .pipeline import PosePipeline
from .sources import LatestFrameCamera, VideoFileSource, SyntheticSource
PROJECT = Path(__file__).resolve().parents[1]

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--repo', type=Path, default=PROJECT.parents[1])
    ap.add_argument('--source', default='0')
    ap.add_argument('--cad', default='accepted_form_v1',
                    choices=('accepted_form_v1', 'webcam_articulation_v1', 'form_candidate_v1'))
    ap.add_argument('--form-preview', type=Path,
                    help='prepared proportion-study directory with design.json and preview.xml')
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
    ap.add_argument('--beta', type=float)
    ap.add_argument('--min-cutoff', type=float)
    ap.add_argument('--swap-handedness', action='store_true', help='diagnostic override only')
    ap.add_argument('--retarget', choices=('shape', 'position', 'angles'), default='shape',
                    help='shape is an experimental joint-posture comparison, not an accepted replacement')
    ap.add_argument('--full-quality', action='store_true', help='full-size CAD render with shadows')
    ap.add_argument('--stabilize-motion', action='store_true',
                    help='experimental motion smoothing; reduces jumps but adds response delay')
    ap.add_argument('--legacy-tracking', action='store_true',
                    help='diagnostic comparison without consistency checks or continuity handling')
    a = ap.parse_args(argv)
    a.beta = (.025 if a.legacy_tracking else .012) if a.beta is None else a.beta
    a.min_cutoff = (2. if a.legacy_tracking else 2.5) if a.min_cutoff is None else a.min_cutoff
    if a.form_preview is None and a.cad == 'accepted_form_v1':
        a.form_preview = a.repo/'cad/accepted_form_v1'
    if a.form_preview and a.whole:
        ap.error('--form-preview is a hand-only shape study')
    if a.calibration is None:
        if a.form_preview:
            signature = hashlib.sha256((a.form_preview / 'design.json').read_bytes()).hexdigest()[:16]
            a.calibration = PROJECT / f'calibration_preview_{a.retarget}_{signature}.json'
        else:
            a.calibration = PROJECT / (f'calibration_shape_{a.cad}.json' if a.retarget == 'shape' else
                f'calibration_pose_v2_{a.cad}.json' if a.retarget == 'position' else
                'calibration_articulation_v1.json' if a.cad == 'webcam_articulation_v1' else 'calibration_v3.json')
    robot = tracker = source = ui = log = None
    error = None
    try:
        if a.form_preview:
            from .form_preview import FormPreviewHand
            robot = FormPreviewHand(a.form_preview)
        else:
            robot = CadHand(a.repo, whole=a.whole, cad=a.cad)
        pipeline = PosePipeline(robot.design, robot.ranges, a.calibration, a.beta, a.min_cutoff, a.retarget,
                                stabilize=a.stabilize_motion, robust=not a.legacy_tracking)
        log = SessionLog(a.log_dir, dict(
            source=a.source, requested_fps=a.fps, cad_source=str(robot.source),
            code_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in Path(__file__).parent.glob('*.py')},
            cad_hashes=robot.hashes, preview='proportion_mesh' if a.form_preview else 'kinematic',
            calibration=str(a.calibration),
            calibration_snapshot=calibration_snapshot(pipeline.calibration),
            finger_mapping=a.retarget, render_quality='full' if a.full_quality else 'fast',
            thumb_mapping='calibrated_posture_and_independent_features' if a.retarget=='shape' else 'position_fit_with_design_joint_frames',
            motion_stabilization=a.stabilize_motion,
            tracking_revision='legacy' if a.legacy_tracking else 'geometric_consistency_and_continuity_v1',
            filter=dict(beta=a.beta, min_cutoff=a.min_cutoff)))
        cases = PoseCaseRecorder(log.directory)
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
        processing_fps, view, debug, testing = (float(a.fps), 0, False, False)
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
            rendered = robot.render(view, width=532 if a.full_quality else 426,
                                    height=620 if a.full_quality else 496, fast=not a.full_quality) if ui else None
            render_end = time.perf_counter()
            key = ui.show(image, detection, rendered, pose, pipeline, timing, mode, view, debug) if ui else 255
            timing.update(map_ms=(command_time-tracking_end)*1000,
                          render_ms=(render_end-render_start)*1000,
                          ui_ms=(time.perf_counter()-render_end)*1000)
            log.frame(seq, captured, command_time, detection, pose, robot.angles, timing, mode)
            if key in (ord('q'), 27):
                break
            elif key in (ord('s'), ord('S')) and ui:
                capture_displayed_frame(cases, log, ui, pipeline, seq=seq,
                    captured=captured, command_time=command_time, detection=detection,
                    pose=pose, actual=robot.angles, timing=timing, mode=mode, view=view,
                    camera_bgr=image, robot_rgb=rendered)
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
                pipeline.reselect_hand()
            elif key == ord('d'):
                debug = not debug
            elif key == ord('t'):
                testing = not testing
                if not testing and pipeline.continuity:
                    pipeline.command = robot.angles.copy()
                    pipeline.continuity.position = robot.angles.copy()
                    pipeline.continuity.hold(captured)
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
