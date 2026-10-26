"""Keep the first study and evaluate one geometrically informed alternative."""
import importlib.util
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_palm_second_placement',HERE/'check.py')
c=importlib.util.module_from_spec(spec);spec.loader.exec_module(c)
c.HERE=HERE/'selected'
sys.argv=[str(__file__),'--plan',str(HERE/'placement2.json'),'--output',str(HERE/'reviews/check2.json')]
c.main()
