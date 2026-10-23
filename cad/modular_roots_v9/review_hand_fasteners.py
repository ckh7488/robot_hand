import FreeCAD as A
import Part,json,sys,time,hashlib
from pathlib import Path
sys.stdout.reconfigure(encoding='utf-8')
D=Path(__file__).resolve().parent
S=json.loads((D/'scene.json').read_text(encoding='utf-8'));M=json.loads((D/'cad_manifest.json').read_text(encoding='utf-8'))
assets={n:Part.read(str(D/'parts'/(n+'.step'))) for n in M['assets']}
def placed(s,it):
 m=A.Matrix()
 for i in range(3):
  for j in range(3):setattr(m,'A%d%d'%(i+1,j+1),it['R'][i][j])
  setattr(m,'A%d4'%(i+1),it['pos_mm'][i])
 o=s.copy();o.Placement=A.Placement(m).multiply(o.Placement);return o
cache={};exact_calls=0
result={'scope':'Modeled M2 machine and TAP screws, M3 screws and captive nuts versus CAD assembly parts. Explicit fastener/part volume intersections; no thread helix, assembly preload or strength claim.','poses':{},'bolt_length_mm':6,'frame_mm':3,'printed_wall_mm':5,'nut_height_mm':1.6,'pocket_depth_mm':2.2,'calculated_tip_beyond_nut_mm':.4,'procurement_exact_low_head_part_verified':False,'reference_screw':{'part':'NBK SNZS-M2-6','source':'https://www.nbk1560.com/en/products/specialscrew/nedzicom/miniaturescrew/SNZS-M/SNZS-M2/','manufacturer_dimensions_verified':True,'physical_fit_and_preload_verified':False},'factory_S101_head_seat_z_mm':-11.8,'factory_H101_head_seat_y_mm':15.8}
for pn in sys.argv[1:] or ['Open']:
 items=list(S[pn]);shapes=[placed(assets[it['asset']],it) for it in items];bad=[];calls=0;start=time.monotonic()
 for i in range(len(items)):
  for j in range(i+1,len(items)):
   if items[i]['role']!='fastener' and items[j]['role']!='fastener':continue
   s,t=shapes[i],shapes[j]
   if not s.BoundBox.intersect(t.BoundBox):continue
   calls+=1
   aa,bb=items[i],items[j]
   if bb['name'] in aa.get('threaded_engagement_with',[]) or aa['name'] in bb.get('threaded_engagement_with',[]):continue
   import numpy as np
   ar=np.array(aa['R']);br=np.array(bb['R']);at=np.array(aa['pos_mm']);bt=np.array(bb['pos_mm']);sig=np.round(np.r_[(ar.T@br).ravel(),ar.T@(bt-at)],7);sig[sig==0]=0.
   key=(aa['asset'],bb['asset'],sig.tobytes())
   if key not in cache:cache[key]=s.common(t).Volume;exact_calls+=1
   vol=cache[key]
   if vol>.01:bad.append({'pair':[items[i]['name'],items[j]['name']],'intersection_mm3':round(vol,5)})
 result['poses'][pn]={'checked_overlapping_bounds':calls,'interference':sorted(bad,key=lambda x:-x['intersection_mm3']),'seconds':round(time.monotonic()-start,2),'unique_exact_booleans_total':exact_calls}
 print(pn,'pairs',calls,'bad',len(bad),'largest',result['poses'][pn]['interference'][:10],flush=True)
 (D/'reviews/hand_fasteners.json').write_text(json.dumps(result,indent=2))
result['modeled_thread_engagement_note']='TAP-to-designated motor thread volume is intentional and omitted from collision failure; frame heads, neighboring parts and non-designated motor pieces remain checked. No helix, driver bit or clamping force model.'
result['passed']=not any(r['interference'] for r in result['poses'].values())
(D/'reviews/hand_fasteners.json').write_text(json.dumps(result,indent=2));print('PASSED',result['passed'],flush=True)
