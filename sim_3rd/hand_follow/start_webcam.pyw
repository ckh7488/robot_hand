"""Double-click launcher: a windowless interpreter and persistent error log."""
import os
from pathlib import Path
import subprocess
import sys
import traceback

PROJECT = Path(__file__).resolve().parent
WORK = Path(os.environ.get('HAND_FOLLOW_WORK', 'D:/Codex-work/robot-hand-workspace/work'))
RUNTIME = Path(os.environ.get('USERPROFILE', '')) / '.cache/codex-runtimes/codex-primary-runtime/dependencies/python/pythonw.exe'
DEPS = Path(os.environ.get('HAND_FOLLOW_DEPS', str(WORK / 'hand_follow_deps')))

if sys.version_info[:2] != (3, 12) and DEPS.exists() and RUNTIME.exists():
    subprocess.Popen([str(RUNTIME), str(__file__), *sys.argv[1:]], cwd=PROJECT,
                     creationflags=subprocess.CREATE_NO_WINDOW)
    raise SystemExit(0)

os.chdir(PROJECT)
for name in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[name] = '1'
(WORK / 'tmp').mkdir(parents=True, exist_ok=True)
os.environ['TEMP'] = os.environ['TMP'] = str(WORK / 'tmp')
os.environ['MPLCONFIGDIR'] = str(WORK / 'hand_follow_webcam/matplotlib')
if DEPS.exists():
    sys.path.insert(0, str(DEPS))
sys.path.insert(0, str(PROJECT))
log_directory = WORK / 'hand_follow_sessions'
log_directory.mkdir(parents=True, exist_ok=True)
with (log_directory / 'launcher.log').open('a', encoding='utf-8', buffering=1) as log:
    sys.stdout = sys.stderr = log
    try:
        from hand_follow.app import main
        main()
    except Exception:
        traceback.print_exc()
        if '--no-gui' not in sys.argv:
            import ctypes
            ctypes.windll.user32.MessageBoxW(0, f'실행 중 오류가 발생했습니다.\n{log_directory / "launcher.log"}', 'Robot Hand', 0x10)
        raise SystemExit(1)
