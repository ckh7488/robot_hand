"""Original module geometry and explicit temporary assembly placements."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('_arm068_prior',ROOT/'cad/oriented_fastener_entry_v1/model.py')
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
n,np,d=a.n,a.np,a.d
translate,moved,bbox=a.translate,a.moved,a.bbox


def context():
    rows,membership,source=a.context()
    for f in [Path(__file__),ROOT/'cad/oriented_fastener_entry_v1/reviews/check.json',ROOT/'cad/oriented_fastener_entry_v1/reviews/evidence_final.json']:
        source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    return rows,membership,source


def packets(rows):return a.a.packets(rows)
