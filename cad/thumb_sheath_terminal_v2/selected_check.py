"""Same complete static checks, with new rib and corrected local bore."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import selected_model as m
e=m.drive.module('_offset024_selected_engine',m.ROOT/'cad/thumb_sheath_terminal_v2/check_offset.py')
def inputs(design):
    data,pack=e.prior.inputs(design)
    for n in ['offset_model.py','check_offset.py','selected_model.py','selected_check.py']:
        data[0]['cad/thumb_sheath_terminal_v2/'+n]=m.drive.sha(Path(__file__).resolve().parent/n)
    return data,pack
e.m=m;e.HERE=m.HERE;e.inputs=inputs
if __name__=='__main__':e.main()
