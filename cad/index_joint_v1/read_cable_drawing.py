import pymupdf as fitz,sys
from pathlib import Path
sys.stdout.reconfigure(encoding='utf-8')
r=Path(r'E:/hand_recog/mujoco-direct/cad/index_joint_v1/sources/cable_fastener');d=fitz.open(r/'EH_catalog.pdf')
for i in [2,3,4]:
 p=d[i];p.get_pixmap(matrix=fitz.Matrix(2,2)).save(str(r/('EH_page_'+str(i+1)+'.png')))
 print('PAGE',i+1,p.get_text()[:12000])
