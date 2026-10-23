"""Build an illustrative linear elastic palm load screen from actual CAD mesh.
N, mm, MPa, tonne units. Not anisotropic FDM certification or bolted contact FEA.
"""
import sys,json,math,re,hashlib,numpy as np
from pathlib import Path
D=Path(__file__).resolve().parent.parent.parent;F=Path(__file__).resolve().parent
sys.stdout.reconfigure(encoding='utf-8')
mesh=F/'palm_mesh_linear.inp';nodes={};elements={};mode='';etypes={}
for line in mesh.read_text(encoding='utf-8').splitlines():
 line=line.strip()
 if not line or line.startswith('**'):continue
 if line.startswith('*'):
  if line.upper().startswith('*NODE'):mode='node'
  elif line.upper().startswith('*ELEMENT'):
   typ=re.search(r'TYPE=([^,]+)',line.upper()).group(1);mode='element' if typ=='C3D10' else '';etypes[typ]=etypes.get(typ,0)
  else:mode=''
  continue
 fields=[x.strip() for x in line.split(',') if x.strip()]
 if mode=='node':nodes[int(fields[0])]=list(map(float,fields[1:4]))
 elif mode=='element':elements[int(fields[0])]=list(map(int,fields[1:]));etypes['C3D10']+=1
assert elements and all(len(v)==10 for v in elements.values()),etypes
ids=np.array(sorted(nodes));xyz=np.array([nodes[k] for k in ids]);imap={n:i for i,n in enumerate(ids)}
# Check corner tetrahedra and straight second-order nodes before solving.
conn=np.array([[imap[n] for n in row] for row in elements.values()]);p=xyz[conn[:,:4]];J=np.stack([p[:,1]-p[:,0],p[:,2]-p[:,0],p[:,3]-p[:,0]],axis=2);vol=np.linalg.det(J)/6
assert np.all(vol>1e-10),(vol.min(),np.sum(vol<=0))
matmass=json.loads((D/'mass_screen.json').read_text(encoding='utf-8'));C=json.loads((D/'design.json').read_text(encoding='utf-8'));scene=json.loads((D/'scene.json').read_text(encoding='utf-8'))['Open'];byname={r['name']:r for r in matmass['items']};sceneidx={r['name']:r for r in scene}
# Clamp cylindrical bore nodes at four wrist saddle holes; idealized rigid bolted support.
support=np.zeros(len(ids),bool)
for x,y in [(x,y) for x in [-32,8] for y in [8,28]]:
 rr=np.linalg.norm(xyz[:,:2]-[x,y],axis=1);support|=(abs(rr-1.7)<.3)&(xyz[:,2]>=-63.01)&(xyz[:,2]<=-57.99)
assert support.sum()>=12,support.sum()
patches=[]
for f in C['hand']['fingers']:
 n=f['name'];rows=[r for r in matmass['items'] if r['name'].startswith(n+'_')]
 if n=='thumb':
  rows+=[byname['Thumb_internal_cradle'],byname['Palm_thumb_mount_screws'],byname['Palm_thumb_mount_nuts']]
  aa=np.deg2rad(f['base_rpy'][2]);R=np.array([[np.cos(aa),-np.sin(aa),0],[np.sin(aa),np.cos(aa),0],[0,0,1.]])
  centers=np.array([np.array([*f['root'][:2],0.])+R@np.array([24,y,-60.5]) for y in [-14,8]])
 else:
  rows+=[byname[n.capitalize()+'_root_cartridge'],byname[n.capitalize()+'_root_mount_screws'],byname[n.capitalize()+'_root_mount_nuts']]
  centers=np.array([[f['root'][0]+25,f['root'][1]+y,-60.5] for y in [-24,4]])
 mask=np.zeros(len(ids),bool)
 for c in centers:mask|=np.linalg.norm(xyz-c,axis=1)<6.0
 assert mask.sum()>12,(n,mask.sum())
 patches.append((n,rows,mask,centers.mean(0)))
# Non-structural cover weight delivered through its five top lands.
covermask=np.zeros(len(ids),bool)
for x,y in [(-39,15),(-62,48),(-86,98),(-40,106),(2,121)]:covermask|=(np.linalg.norm(xyz[:,:2]-[x,y],axis=1)<4)&(xyz[:,2]>19)
patches.append(('cover',[byname['Palm_front_cover'],byname['Palm_cover_screws'],byname['Palm_cover_nuts']],covermask,np.array([-40,75,20.])))
report={'material':{'E_MPa':1500.,'nu':.35,'density_tonne_mm3':1.25e-9,'status':'Illustrative isotropic polymer, NOT measured printed anisotropic material'},'mesh':{'source_step_sha256':hashlib.sha256((D/'parts/Palm_back_chassis.step').read_bytes()).hexdigest(),'mesh_sha256':hashlib.sha256(mesh.read_bytes()).hexdigest(),'nodes':len(nodes),'C3D10_elements':len(elements),'straight_midside_nodes':True,'minimum_corner_tet_volume_mm3':float(vol.min()),'total_corner_tet_volume_mm3':float(vol.sum()),'negative_corner_tets':int(np.sum(vol<=0))},'support_nodes':int(support.sum()),'support_model':'Zero displacement on M3 cylindrical bore nodes in four saddle holes; idealized rigid clamps, no preload/contact','cases':[],'limitations':['No FDM layer anisotropy, creep, fatigue, impact, buckling or measured modulus.','Cartridge load resultants distributed to two M3 bearing-zone node groups; cartridges, bolt contact, slip and preload are not modeled.','Palm cover stiffness ignored; its mass is included.','Wiring, missing arm flange hardware and motor inertia omitted.','Quadratic displacement interpolation on straight-sided CAD tetrahedra; no mesh-convergence proof yet.']}
lines=['*HEADING','CAD palm structural SCREEN ONLY','*NODE']+[str(n)+','+','.join('%.12g'%v for v in nodes[n]) for n in sorted(nodes)]
lines+=['*ELEMENT,TYPE=C3D10,ELSET=EALL']+[str(n)+','+','.join(map(str,elements[n])) for n in sorted(elements)]
lines+=['*NSET,NSET=SUPPORT']
sids=ids[support].tolist()
for i in range(0,len(sids),16):lines.append(','.join(map(str,sids[i:i+16])))
lines+=['*MATERIAL,NAME=POLYMER_SCREEN','*ELASTIC','1500.,0.35','*DENSITY','1.25e-9','*SOLID SECTION,ELSET=EALL,MATERIAL=POLYMER_SCREEN','*BOUNDARY','SUPPORT,1,3']
allpatch=np.zeros(len(ids),bool)
for case,unit in [('palm_normal',[0,0,-1]),('finger_direction',[0,-1,0]),('sideways',[-1,0,0])]:
 unit=np.array(unit,float);loads=np.zeros_like(xyz);wrench_records=[]
 for name,rows,mask,ref in patches:
  allpatch|=mask;Fv=sum((r['mass_kg']*9.81*unit for r in rows),np.zeros(3));Mv=sum((np.cross(np.array(r['com_mm'])-ref,r['mass_kg']*9.81*unit) for r in rows),np.zeros(3));wanted=np.r_[Fv,Mv];p=xyz[mask]-ref;A=np.zeros((6,len(p)*3))
  for k,rr in enumerate(p):
   A[:3,k*3:k*3+3]=np.eye(3);x,y,z=rr;A[3:,k*3:k*3+3]=[[0,-z,y],[z,0,-x],[-y,x,0]]
  vals=(A.T@np.linalg.solve(A@A.T,wanted)).reshape(-1,3);loads[mask]+=vals;err=np.max(np.abs(A@vals.ravel()-wanted));assert err<1e-7
  wrench_records.append({'name':name,'nodes':int(mask.sum()),'mass_kg':sum(r['mass_kg'] for r in rows),'force_N':Fv.tolist(),'moment_Nmm':Mv.tolist(),'maximum_resultant_error':float(err)})
 lines+=['*STEP','*STATIC','*CLOAD,OP=NEW']
 for nid,force in zip(ids,loads):
  for j,v in enumerate(force):
   if abs(v)>1e-12:lines.append('%d,%d,%.12g'%(nid,j+1,v))
 lines+=['*DLOAD,OP=NEW','EALL,GRAV,9810.,'+','.join(map(str,unit)),'*NODE FILE','U','*EL FILE','S','*NODE PRINT,NSET=SUPPORT','RF','*END STEP']
 report['cases'].append({'name':case,'gravity_direction':unit.tolist(),'patches':wrench_records,'point_load_resultant_N':loads.sum(0).tolist(),'palm_self_weight_N':float(vol.sum()*1.25e-6*9.81)})
(F/'palm_static.inp').write_text('\n'.join(lines)+'\n');(F/'setup.json').write_text(json.dumps(report,indent=2));np.savez_compressed(F/'mesh_index.npz',ids=ids,xyz=xyz,conn=conn,support=support,load_patch=allpatch)
print('PREPARED',len(nodes),'nodes',len(elements),'C3D10 elements','support',support.sum(),flush=True)
print(json.dumps(report['cases'],indent=2),flush=True)
