import sys,json,copy
from pathlib import Path
D=Path(__file__).resolve().parent;B=D.parent/'modular_roots_v9';sys.path.insert(0,str(B));from kinematics_numpy import C,META,forward
cases=[]
for n,p in C['poses'].items():cases.append({'name':n,'frames':forward(p).tolist()})
for n in ['Open','Close']:
 for spread in [-8,3]:
  for mcp in [-20,60]:
   p=copy.deepcopy(C['poses'][n]);p['little'][0]=spread;p['little'][1]=mcp;cases.append({'name':n+'_s'+str(spread)+'_m'+str(mcp),'frames':forward(p).tolist()})
(D/'reviews'/'guide_exact_cases.json').write_text(json.dumps({'meta':META,'cases':cases}),encoding='utf-8')
