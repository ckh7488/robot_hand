"""Saved ARM065 geometry only; assembly poses never change final placement."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('_arm066_prior',ROOT/'cad/shared_roll_root_v1/model.py')
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
n,np,d=a.n,a.np,a.d


def context():
    before,membership,source=a.context()
    raw=n.p.read(ROOT/'cad/shared_roll_root_v1/reviews/check.json')
    saved=n.c.load(n.p.read(ROOT/raw['scene'])['parts'],source)
    rows=[z for z in before if z['name'] not in a.OLD]+saved
    for z in saved:membership[z['name']]='forearm'
    for f in [Path(__file__),ROOT/'cad/shared_roll_root_v1/reviews/check.json',ROOT/'cad/shared_roll_root_v1/reviews/evidence_final.json',ROOT/'docs/reviews/ARM065/native_difference.json']:
        source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    assert len(rows)==886 and len({z['name'] for z in rows})==886
    return rows,membership,source


def packets(rows):
    names={z['name'] for z in rows};raw=n.p.read(ROOT/'cad/eight_bank_layout_v1/reviews/check.json')
    out=[]
    for p in raw['placements']:
        present=set(p['rows'])&names
        if p['module']=='thumb_ip_actuator':
            present|={'arm059_integrated_electrical_R5_base','arm059_straight_jaw_0','arm059_straight_jaw_1'}&names
        mounting={k for k in present if '_rack_nut_' in k or '_rack_screw_' in k}
        assert len(mounting)==4
        out.append(dict(module=p['module'],row=p['row'],bank=p['bank'],azimuth_deg=p['azimuth_deg'],
            names=sorted(present),mounting_names=sorted(mounting),body_names=sorted(present-mounting),
            approach_normal=[0.,-1. if p['module']=='thumb_ip_actuator' else 1.,0.],
            original_module_interface_reference='ARM054 saved placements;all actual parts use ARM065 final world geometry,including ARM059 replacements.'))
    assert len(out)==16 and sum(map(lambda z:len(z['names']),out))==len(set(k for p in out for k in p['names']))
    return out


def translation(v):
    M=np.eye(4);M[:3,3]=v;return M


def moved(rows,v):
    M=translation(v)
    return [dict(z,shape=n.g.original.transform(z['shape'],M)) for z in rows]


def bbox(rows):
    boxes=np.array([n.c.m.pack._bbox(z['shape']) for z in rows])
    return np.stack([boxes[:,0].min(axis=0),boxes[:,1].max(axis=0)])
