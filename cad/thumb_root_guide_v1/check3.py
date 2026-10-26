import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import build_check as check
import guide_candidate3 as selected
check.g=selected
check.HERE=selected.HERE
if __name__=='__main__':check.main()
