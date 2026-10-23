"""Compare saved FCStd BRep payloads without expensive intersections of identical vendor CAD.
Only OCCT's checked flag is ignored. All topology, token order, and other flags remain.
Numerical serialization roundoff is bounded by an absolute 1e-10 token tolerance.
This is a serialized-model equivalence check, not a geometric Hausdorff certificate.
"""
import re, zipfile, xml.etree.ElementTree as ET
NUM=re.compile(r'[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?')
FLAGS=re.compile(r'(?m)^([01]{2})[01]([01]{4})[ \t]*$')
def compare_objects(old_path,new_path,exclude=()):
 rows=[]
 with zipfile.ZipFile(old_path) as old,zipfile.ZipFile(new_path) as new:
  def objects(z):
   out={}
   for o in ET.fromstring(z.read('Document.xml')).find('ObjectData'):
    s=o.find("./Properties/Property[@name='Shape']/Part")
    if s is None:continue
    p=o.find("./Properties/Property[@name='Placement']/PropertyPlacement")
    out[o.attrib['name']]=(s.attrib['file'],p.attrib if p is not None else {})
   return out
  a,b=objects(old),objects(new)
  assert set(a)==set(b),'Object set changed'
  for name in sorted(set(b)-set(exclude)):
   fa,pa=a[name];fb,pb=b[name]
   assert set(pa)==set(pb),(name,'Placement fields changed')
   assert all(abs(float(pa[k])-float(pb[k]))<=1e-10 for k in pa),(name,'Placement changed')
   raw_a,raw_b=old.read(fa),new.read(fb)
   if raw_a==raw_b:rows.append({'object':name,'byte_identical':True,'max_numeric_roundoff':0.});continue
   x=FLAGS.sub(r'\g<1>0\g<2>',raw_a.decode('ascii'));y=FLAGS.sub(r'\g<1>0\g<2>',raw_b.decode('ascii'))
   assert NUM.sub('#',x)==NUM.sub('#',y),(name,'BRep structure/order changed')
   xx=NUM.findall(x);yy=NUM.findall(y);assert len(xx)==len(yy),name
   diff=max((abs(float(v)-float(w)) for v,w in zip(xx,yy)),default=0.)
   assert diff<=1e-10,(name,'Numerical geometry changed',diff)
   rows.append({'object':name,'byte_identical':False,'max_numeric_roundoff':diff})
 return {'unchanged_objects':len(rows),'byte_identical':sum(r['byte_identical'] for r in rows),'max_numeric_roundoff':max((r['max_numeric_roundoff'] for r in rows),default=0.),'ignored_flag':'OCCT Checked state only, third of seven bits','numeric_token_tolerance':1e-10,'rows':rows}
