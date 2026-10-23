import fitz,sys
sys.stdout.reconfigure(encoding="utf-8")
from pathlib import Path
r=Path(r'E:/hand_recog/mujoco-direct');out=r/'cad/index_joint_v1'
for name,path in [('motor',r/'sources/XL330.pdf'),('hinge',out/'sources/FPX330-H101.pdf'),('fixed',out/'sources/FPX330-S101.pdf')]:
 d=fitz.open(path)
 for i,p in enumerate(d):
  print(name,i,p.get_text())
  p.get_pixmap(matrix=fitz.Matrix(2,2)).save(str(out/(name+'_drawing.png')))
