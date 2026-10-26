"""Replay the exact checkout without its unused ambiguous service import."""
import sys
from pathlib import Path
clone=Path(sys.argv[1]);original=clone/'cad/wrist_housing_guide_v1/review_core.py'
source=original.read_text(encoding='utf-8-sig');unused='from guide_service import sequence\n'
assert source.count(unused)==1
# sequence is never referenced by this review; all sample transforms are read
# from its saved, hashed service report. No geometry/test logic is changed.
assert source.replace(unused,'').count('sequence')==0
sys.path.insert(0,str(original.parent));sys.argv=[str(original),*sys.argv[2:]]
exec(compile(source.replace(unused,''),str(original),'exec'),{'__file__':str(original),'__name__':'__main__'})
