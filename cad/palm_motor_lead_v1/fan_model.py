"""Allow the third wire to fan away from the rotor without moving its port."""
import importlib.util
from pathlib import Path

spec=importlib.util.spec_from_file_location('_palm_fan_source',Path(__file__).resolve().parent/'zero_model.py')
z=importlib.util.module_from_spec(spec);spec.loader.exec_module(z)
c,b,m,p,plugs,pairs,clear=z.c,z.b,z.m,z.p,z.plugs,z.pairs,z.clear


def wires(motor,angles=None,**parameters):
    if angles is None:return z.wires(motor,**parameters)
    assert len(angles)==3
    return [z.wires(motor,angle=angle,**parameters)[i] for i,angle in enumerate(angles)]
