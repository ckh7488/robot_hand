import json,re,numpy as np
from pathlib import Path
F=Path(__file__).resolve().parent;meta=json.loads((F/'setup.json').read_text(encoding='utf-8'));I=np.load(F/'mesh_index.npz');ids=I['ids'];xyz=I['xyz'];lookup={n:i for i,n in enumerate(ids)};disp=[];stress=[];data=None;kind=None;count=0
for line in (F/'palm_static.frd').open():
 if line.startswith(' -4'):
  kind=line.split()[1];data=np.full((len(ids),3 if kind=='DISP' else 6 if kind=='STRESS' else 1),np.nan);count=0
 elif line.startswith(' -1') and data is not None:
  nid=int(line[3:13]);nums=[float(v) for v in re.findall(r'[-+]?\d+\.\d+E[-+]\d+',line[13:])];data[lookup[nid],:len(nums)]=nums;count+=1
 elif line.startswith(' -3') and data is not None:
  assert count==len(ids),(kind,count,len(ids))
  if kind=='DISP':disp.append(data)
  elif kind=='STRESS':stress.append(data)
  data=None
assert len(disp)==len(stress)==3,(len(disp),len(stress))
# Rigid support and external wrench checks.
blocks=[];current=None
for line in (F/'palm_static.dat').read_text(encoding='utf-8').splitlines():
 if 'forces' in line.lower() and 'SUPPORT' in line:current=[];blocks.append(current)
 elif current is not None and re.match(r'^\s+\d+\s+[-+]?\d',line):
  a=line.split();current.append([int(a[0]),*map(float,a[1:4])])
 elif current is not None and current and not line.strip():current=None
report={'status':'Illustrative static stiffness screen; NOT strength release','source_setup':meta,'cases':[]};arr={}
# Exclude a10mm neighbourhood of applied loads and fixed nodes for a second stress statistic.
from scipy.spatial import cKDTree
patch=I['support']|I['load_patch'];dist=cKDTree(xyz[patch]).query(xyz,k=1)[0];away=dist>10
for k,(u,s,case) in enumerate(zip(disp,stress,meta['cases'])):
 assert np.isfinite(u).all() and np.isfinite(s).all()
 mag=np.linalg.norm(u,axis=1);xx,yy,zz,xy,yz,zx=s.T;vm=np.sqrt(.5*((xx-yy)**2+(yy-zz)**2+(zz-xx)**2)+3*(xy**2+yz**2+zx**2));imax=int(np.argmax(mag));stressidx=int(np.argmax(vm));rf=np.array([r[1:] for r in blocks[k]]).sum(0);expected=-(np.array(case['point_load_resultant_N'])+case['palm_self_weight_N']*np.array(case['gravity_direction']));eq=float(np.linalg.norm(rf-expected));row={'name':case['name'],'maximum_displacement_mm':float(mag[imax]),'displacement_max_at_mm':xyz[imax].tolist(),'maximum_vm_MPa_raw':float(vm.max()),'raw_stress_at_mm':xyz[stressidx].tolist(),'maximum_vm_MPa_excluding_10mm_fixture_and_load_neighbourhoods':float(vm[away].max()),'support_max_displacement_mm':float(mag[I['support']].max()),'summed_reaction_N':rf.tolist(),'expected_reaction_N':expected.tolist(),'reaction_equilibrium_error_N':eq,'max_displacement_if_E800_MPa_mm':float(mag.max()*1500/800),'max_displacement_if_E2500_MPa_mm':float(mag.max()*1500/2500)};report['cases'].append(row);arr['u_'+str(k)]=u;arr['vm_'+str(k)]=vm
report['numerical_checks_passed']=all(r['support_max_displacement_mm']<1e-8 and r['reaction_equilibrium_error_N']<.02 for r in report['cases']);report['mesh_converged']=False;report['FDM_strength_certified']=False
(F/'results.json').write_text(json.dumps(report,indent=2));np.savez_compressed(F/'fields.npz',**arr);print(json.dumps(report['cases'],indent=2));print('Numerical equilibrium',report['numerical_checks_passed'])
