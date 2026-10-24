"""User-marked failure cases. Pixels are written only on an explicit capture.

All arrays come from one displayed frame, before any next-frame processing.
The raw detector landmarks remain separate from mapped/filtered robot angles.
"""
import datetime
import json
from pathlib import Path
import numpy as np
from PIL import Image


def exact_json(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


def calibration_snapshot(calibration):
    return dict(offset_deg=calibration.offset, gain=calibration.gain,
                estimated_dip_indices=calibration.coupled_dips,
                estimated_dip_ratios=calibration.dip_ratios,
                position_profile=calibration.position_profile)


class PoseCaseRecorder:
    def __init__(self, session_directory):
        self.root = Path(session_directory) / 'cases'
        # Do not create any picture directory until the user presses S.
        existing = list(self.root.glob('case_*')) if self.root.exists() else []
        numbers = [int(p.name[5:9]) for p in existing if p.name[5:9].isdigit()]
        self.next_number = max(numbers, default=0) + 1
        self.saved = len([p for p in existing if (p/'case.json').exists() and not p.name.endswith('.partial')])

    def save(self, *, seq, capture_elapsed_s, command_elapsed_s, detection,
             pose, actual, timing, mode, view, pipeline, camera_bgr,
             screen_rgb, robot_rgb):
        number = self.next_number
        name = f'case_{number:04d}'
        staging = self.root / (name + '.partial')
        destination = self.root / name
        staging.mkdir(parents=True, exist_ok=False)
        self.next_number += 1
        files = {}
        for label, pixels in [('camera',camera_bgr), ('screen',screen_rgb), ('robot',robot_rgb)]:
            if pixels is None:
                continue
            pixels = np.asarray(pixels)
            if pixels.dtype != np.uint8 or pixels.ndim != 3 or pixels.shape[2] != 3:
                raise ValueError(f'Invalid {label} image')
            if label == 'camera':
                pixels = pixels[:, :, ::-1]  # The mirrored camera buffer shown by the app.
            filename = label + '.png'
            Image.fromarray(pixels).save(staging / filename)
            files[label] = filename
        payload = dict(schema=1, case_id=name, number=number,
            saved_at=datetime.datetime.now().astimezone().isoformat(),
            seq=seq, capture_elapsed_s=capture_elapsed_s, command_elapsed_s=command_elapsed_s,
            mode=mode, view=['front','oblique','side'][view%3], retarget_mode=pipeline.retarget_mode,
            filter_on=pipeline.filter_on, detection=detection, pose=pose, actual_deg=actual,
            timing=timing, calibration_snapshot=calibration_snapshot(pipeline.calibration),
            files=files, session_metadata='../../session.json',
            coordinate_notes=dict(world='Untouched detector world landmarks in meters.',
                image='Normalized image x/y used for the overlay.',
                normalized='Detector normalized x/y/z, when available; z is not metric world depth.',
                camera='Mirrored BGR input saved as RGB PNG, without the overlay.',
                screen='The displayed RGB canvas for this same seq.',
                robot='Rendered preview, not a physical robot sensor reading.'))
        (staging/'case.json').write_text(json.dumps(payload,default=exact_json,ensure_ascii=False,indent=2),encoding='utf-8')
        staging.rename(destination)
        self.saved += 1
        return destination


def capture_displayed_frame(recorder, log, ui, pipeline, *, seq, captured,
                            command_time, detection, pose, actual, timing,
                            mode, view, camera_bgr, robot_rgb):
    """S and the button share this path; a disk failure must not end tracking."""
    try:
        directory = recorder.save(seq=seq, capture_elapsed_s=captured-log.started,
            command_elapsed_s=command_time-log.started, detection=detection,
            pose=pose, actual=actual, timing=timing, mode=mode, view=view,
            pipeline=pipeline, camera_bgr=camera_bgr, screen_rgb=ui.last_canvas,
            robot_rgb=robot_rgb)
    except (OSError,ValueError) as exc:
        log.event('pose_case_failed', seq=seq, error=str(exc))
        ui.notify('저장 실패 · 저장 공간과 로그를 확인해 주세요')
        return None
    log.event('pose_case_saved', case_id=directory.name, seq=seq,
              detection_present=detection is not None, mode=mode,
              directory=str(directory))
    ui.capture_count = recorder.saved
    suffix = ' · 인식 없음도 기록' if detection is None else ''
    ui.notify(f'오류 자세 #{int(directory.name[5:]):03d} 저장 완료{suffix}')
    return directory
