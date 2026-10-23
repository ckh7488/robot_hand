import pymupdf as f
from pathlib import Path
r=Path(r'E:/hand_recog/mujoco-direct/cad/index_joint_v1/sources/cable_fastener');d=f.open(r/'EH_catalog.pdf');p=d[5];p.get_pixmap(matrix=f.Matrix(2,2)).save(str(r/'EH_page_6.png'));print(p.get_text())
