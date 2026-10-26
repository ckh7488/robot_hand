"""Run a CAD entry point with the OCC pool limited as well as BLAS/OpenMP."""
import os
from pathlib import Path
import runpy
import sys

for key in ['OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS']:
    os.environ[key]='1'
from cadquery.occ_impl.shapes import setThreads
from OCP.OSD import OSD_ThreadPool
setThreads(1)
pool=OSD_ThreadPool.DefaultPool_s()
print('OCC thread pool initialized to1',flush=True)
print('pool getters',[(name,getattr(pool,name)()) for name in ['NbThreads','NbDefaultThreadsToLaunch'] if hasattr(pool,name)],flush=True)
target=Path(sys.argv[1]).resolve()
sys.argv=[str(target),*sys.argv[2:]]
sys.dont_write_bytecode=True
runpy.run_path(str(target),run_name='__main__')
