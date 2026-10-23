import sys
from OCP.STEPControl import STEPControl_Reader
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
def load(p):
    r=STEPControl_Reader(); r.ReadFile(p); r.TransferRoots(); return r.OneShape()
def vol(s):
    g=GProp_GProps(); BRepGProp.VolumeProperties_s(s,g); return g.Mass()
C=sys.argv[1]
for a in ['Root_cartridge','Compact_root','Thumb_internal_cradle']:
    new=load(f'{C}/modular_roots_v9/parts/{a}.step'); old=load(f'{C}/modular_roots_v8/parts/{a}.step')
    c1=BRepAlgoAPI_Cut(new,old); c2=BRepAlgoAPI_Cut(old,new)
    print(a,'V9 vol',round(vol(new),3),'V8 vol',round(vol(old),3),'V9-V8 (new material)',round(vol(c1.Shape()),6),'V8-V9 (removed)',round(vol(c2.Shape()),3))
