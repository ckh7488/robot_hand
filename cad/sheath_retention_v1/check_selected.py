import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import selected_retention as m
e=m.drive.module('_retain026_selected_check',m.HERE/'check.py')
raw_inputs=e.inputs
def inputs(design):
    folder=e.HERE;e.HERE=m.HERE
    try:data,pack,guide,housing=raw_inputs(design)
    finally:e.HERE=folder
    for n in ['selected_retention.py','check_selected.py','roll_probe.py']:data[0]['cad/sheath_retention_v1/'+n]=m.drive.sha(m.HERE/n)
    data[0]['cad/sheath_retention_v1/reviews/roll_probe.json']=m.drive.sha(m.HERE/'reviews/roll_probe.json')
    return data,pack,guide,housing
e.HERE=m.HERE/'selected';e.inputs=inputs
e.m=m
if __name__=='__main__':e.main()
