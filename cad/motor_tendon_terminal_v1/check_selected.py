import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import terminal_selected as selected
check=selected.drive.module('_terminal022_selected_check',Path(__file__).resolve().parent/'build_check.py')
check.model=selected;check.HERE=selected.HERE
check.main()
