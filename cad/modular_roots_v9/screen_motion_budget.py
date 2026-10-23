"""Optimistic single-axis torque-speed feasibility bound from CAD point masses.
Not a servo dynamics simulation or a continuous torque rating.
"""
import os,json,numpy as np
from pathlib import Path
D=Path(__file__).resolve().parent;os.environ['MPLCONFIGDIR']=str(D/'mpl_config')
from kinematics_numpy import C,META,forward,rigid_body
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams['font.family']='Malgun Gothic';plt.rcParams['axes.unicode_minus']=False
M=json.loads((D/'cad_manifest.json').read_text(encoding='utf-8'));f=next(f for f in C['hand']['fingers'] if f['name']=='middle');mat=forward(C['poses']['Open']);idx={x['name']:i for i,x in enumerate(META)};A=mat[idx['middle_mcp_motor']];axis=A[:3,0];origin=A[:3,3];rows=[]
for it,t in zip(META,mat):
 if rigid_body(it['name']) not in ['middle_mcp','middle_pip','middle_dip']:continue
 a=M['assets'][it['asset']]
 if it['role']=='vendor_motor':mass=.018;com=np.array([-15,-7.5,0.])
 elif it['role']=='vendor_rotor':continue
 else:mass=a['volume_mm3']*(7.85e-6 if it['role']=='fastener' else 1.25e-6);com=np.array(a['com_mm'])
 rv=(t[:3,:3]@com+t[:3,3]-origin)/1000;rows.append({'name':it['name'],'mass_kg':mass,'first_moment_kgm':(mass*rv).tolist(),'point_inertia_kg_m2':float(mass*np.linalg.norm(np.cross(axis,rv))**2)})
# Best-case distal motor deletion omits all its motor/frame/output hardware and ignores added coupling mass.
def retained(r):return not r['name'].startswith('middle_dip_') or r['name']=='middle_dip_tip'
scenarios={'current':rows,'ideal_passive_DIP':[r for r in rows if retained(r)]};u=np.linspace(0,1,1001);h=10*u**3-15*u**4+6*u**5;hd=30*u**2-60*u**3+30*u**4;hdd=60*u-180*u**2+120*u**3
out={'scope':'Analytical optimistic lower bounds, single MCP axis, other joints held straight. No control loop, gear losses, rotor/reflected inertia or structural flex.','reference_url':'https://emanual.robotis.com/docs/en/dxl/x/xl330-m077/','density_assumption_g_cm3':1.25,'rows':[],'scenarios':{}}
for key,rr in scenarios.items():
 I=sum(r['point_inertia_kg_m2'] for r in rr);moment=sum((np.array(r['first_moment_kgm']) for r in rr),np.zeros(3));G=9.81*np.linalg.norm(np.cross(axis,moment));eqr=I/np.linalg.norm(np.cross(axis,moment));eqm=np.linalg.norm(np.cross(axis,moment))/eqr
 out['scenarios'][key]={'downstream_mass_kg':sum(r['mass_kg'] for r in rr),'point_mass_inertia_lower_bound_kg_m2':I,'worst_static_gravity_Nm':float(G),'equivalent_single_point_mass_kg':float(eqm),'equivalent_point_radius_m':float(eqr)}
 for angle in [60,90]:
  delta=np.deg2rad(angle)
  for label,stall,rpm in [('5V nominal',.215,383),('6V nominal',.228,456)]:
   omega=rpm*2*np.pi/60
   for gravity in ['zero','all_hand_orientations']:
    # Worst orientation at each instant gives G; one global orientation attaining the peak violation exists.
    # Thus |I*qdd|+G is the all-orientations envelope.
    g=0 if gravity=='zero' else G
    def feasible(duration):
     v=delta*hd/duration;acc=delta*hdd/duration**2;available=stall*np.maximum(0,1-abs(v)/omega);demand=abs(I*acc)+g
     return bool(np.all(demand<=available))
    lo=.02;hi=2.
    for _ in range(40):
     mid=(lo+hi)/2
     if feasible(mid):hi=mid
     else:lo=mid
    out['rows'].append({'scenario':key,'angle_deg':angle,'supply':label,'gravity':gravity,'optimistic_minimum_minjerk_duration_s':hi,'speed_only_minimum_s':float(1.875*delta/omega)})
out['limitations']=['Point masses omit each part intrinsic inertia, so physical inertia is higher.','Linear stall-to-no-load curve is an illustrative DC motor approximation, not a measured XL330 response.','Stall torque is not continuous rated torque; current limiting, bus voltage sag and controller delay can worsen performance.','Ideal passive DIP deletes parts without adding a real linkage; it is a bound, not a proposed build or a changed20axis design.','An all-orientations envelope is used; a particular hand orientation may need less gravity torque.','Full-solid CAD volume assumptions require weighed parts.','This screen does not establish a safe speed or a motor selection.']
(D/'reviews/motion_budget.json').write_text(json.dumps(out,indent=2));print(json.dumps(out['scenarios'],indent=2));print(json.dumps(out['rows'],indent=2))
fig,ax=plt.subplots(figsize=(10,5),facecolor='#eff4fa');ax.set_facecolor('#eff4fa');labels=['현재 CAD 질량','DIP 구동부 제거 가정'];x=np.arange(2)
for j,(g,label,col) in enumerate([('zero','중력 제외','#407f9e'),('all_hand_orientations','모든 손 방향 중력 포함','#dd9565')]):
 vals=[next(r['optimistic_minimum_minjerk_duration_s']*1000 for r in out['rows'] if r['scenario']==k and r['angle_deg']==90 and r['supply']=='5V nominal' and r['gravity']==g) for k in scenarios]
 bars=ax.bar(x+(j-.5)*.3,vals,.3,label=label,color=col)
 for b,v in zip(bars,vals):ax.text(b.get_x()+b.get_width()/2,v+5,f'{v:.0f}ms',ha='center')
ax.set_xticks(x,labels);ax.set_ylabel('90° 최소 저크 궤적의 낙관적인 시간 하한(ms)');ax.set_ylim(0,max(v['optimistic_minimum_minjerk_duration_s']*1000 for v in out['rows'])*1.18);ax.legend();ax.set_title('모터의 무부하 최고 속도만으로 빠른 추종을 판정할 수 없음',pad=15)
fig.text(.08,.02,'5V 공칭값과 점질량 근사 / 서보 응답 실측 아님 · 마찰·고유 관성·전류 제한을 넣으면 더 느려질 수 있음',fontsize=10);fig.tight_layout(rect=[0,.06,1,1]);fig.savefig(D/'reviews/motion_budget.png',dpi=160);plt.close(fig)
