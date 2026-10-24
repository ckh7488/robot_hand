"""Windows launcher: pythonw run_hidden.pyw <job.json>; logs to output/run.log."""
from pathlib import Path
import json
import os
import sys
import traceback

job = json.loads(Path(sys.argv[1]).read_text(encoding='utf-8'))
output = Path(job['output']).resolve()
output.mkdir(parents=True, exist_ok=True)
cache = Path(job['cache']).resolve()
for directory in [cache/'tmp',cache/'hf',cache/'torch',cache/'yolo']:
    directory.mkdir(parents=True,exist_ok=True)
os.environ.update(TEMP=str(cache/'tmp'),TMP=str(cache/'tmp'),OMP_NUM_THREADS='1',
                  OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',HF_HOME=str(cache/'hf'),
                  TORCH_HOME=str(cache/'torch'),YOLO_CONFIG_DIR=str(cache/'yolo'),HF_HUB_OFFLINE='1')
sys.stdout = sys.stderr = (output/'run.log').open('w',encoding='utf-8',buffering=1)
try:
    from compare import main
    sys.argv = ['compare.py']
    for key in ['upstream','chumpy','weights','cases','output','device','repeats']:
        if key in job:
            sys.argv += ['--'+key,str(job[key])]
    main()
    (output/'complete.json').write_text(json.dumps({'completed':True}),encoding='utf-8')
except BaseException:
    traceback.print_exc()
    (output/'failed.txt').write_text(traceback.format_exc(),encoding='utf-8')
    raise SystemExit(1)
