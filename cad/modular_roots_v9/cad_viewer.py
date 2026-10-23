"""Actual CAD mesh forward-kinematics viewer. No motor response/dynamics simulation."""
import json,time,copy,threading
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from kinematics_numpy import D,C,META,forward,verify

def display_model():
 assets=''.join('<mesh name="%s" file="%s"/>'%(n,(D/'meshes'/(n+'.stl')).as_posix()) for n in sorted(set(it['asset'] for it in META)))
 geoms=[]
 for it in META:
  q=Rotation.from_matrix(it['R']).as_quat();vals=lambda x:' '.join(map(str,x))
  geoms.append('<geom name="%s" type="mesh" mesh="%s" pos="%s" quat="%s" rgba="%s" contype="0" conaffinity="0"/>'%(it['name'],it['asset'],vals(np.array(it['pos_mm'])/1000),vals([q[3],*q[:3]]),vals([*it['color'],1])))
 xml='<mujoco model="CAD kinematics only"><option gravity="0 0 0"/><visual><headlight ambient=".65 .65 .65" diffuse=".5 .5 .5"/></visual><asset><texture type="skybox" builtin="flat" rgb1=".94 .96 .98" rgb2=".94 .96 .98" width="256" height="1536"/>'+assets+'</asset><worldbody><light pos=".4 .2 .8"/>'+''.join(geoms)+'</worldbody></mujoco>'
 m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m);mujoco.mj_forward(m,d);raw=forward(C['poses']['Open']);inner=[]
 for k,g in enumerate(range(m.ngeom)):
  a=raw[k].copy();a[:3,3]/=1000;b=np.eye(4);b[:3,:3]=d.geom_xmat[g].reshape(3,3);b[:3,3]=d.geom_xpos[g];inner.append(np.linalg.inv(a)@b)
 return m,d,np.array(inner)

def apply_pose(m,d,inner,pose,arm=(0,0,0,0),display_R=None):
 raw=forward(pose,arm).copy();raw[:,:3,3]/=1000;full=raw@inner
 if display_R is not None:
  full[:,:3,:3]=display_R@full[:,:3,:3];full[:,:3,3]=full[:,:3,3]@display_R.T
 m.geom_pos[:]=full[:,:3,3];q=Rotation.from_matrix(full[:,:3,:3]).as_quat();m.geom_quat[:]=q[:,[3,0,1,2]];mujoco.mj_forward(m,d)

def main():
 import tkinter as tk
 from tkinter import ttk
 from concurrent.futures import ThreadPoolExecutor
 import mujoco.viewer
 from mesh_collision import Checker
 verify();m,d,inner=display_model();ui=tk.Tk();ui.title('실제 CAD 관절 검토 · 모터 응답 시뮬레이션 아님');ui.geometry('690x800');state={'dirty':True,'version':0,'checked_version':-1};variables={};armvars=[]
 ttk.Label(ui,text='CAD 형상을 직접 움직이는 뷰어',font=('Malgun Gothic',15,'bold')).pack(pady=(9,3));ttk.Label(ui,text='모터 응답·변형을 계산하지 않음 / 임의 자세는 충돌할 수 있음',foreground='#9b401b').pack()
 buttonrow=ttk.Frame(ui);buttonrow.pack(pady=8)
 def changed(*args):state['dirty']=True;state['version']+=1
 def setpose(n):
  for f,q in C['poses'][n].items():
   for j,v in enumerate(q):variables[f][j].set(v)
  changed()
 for n in C['poses']:ttk.Button(buttonrow,text=n,command=lambda name=n:setpose(name),width=10).pack(side='left')
 def slider(parent,label,lo,hi,initial=0):
  row=ttk.Frame(parent);row.pack(fill='x');ttk.Label(row,text=label,width=14).pack(side='left');v=tk.DoubleVar(value=initial);ttk.Scale(row,from_=lo,to=hi,variable=v,command=changed).pack(side='left',fill='x',expand=True);ttk.Label(row,textvariable=v,width=9).pack(side='left');return v
 af=ttk.LabelFrame(ui,text='팔 · 공칭 모터 외곽 배치 참고');af.pack(fill='x',padx=10,pady=4)
 for label,lo,hi in [('팔꿈치',-35,60),('전완 회전',-60,60),('손목 굽힘',-35,55),('손목 좌우',-20,20)]:armvars.append(slider(af,label,lo,hi))
 outer=ttk.Frame(ui);outer.pack(fill='both',expand=True,padx=10)
 ranges={'index':[(-4,8),(-20,60),(0,60),(0,45)],'middle':[(-3,3),(-20,60),(0,60),(0,45)],'ring':[(-4,3),(-20,60),(0,60),(0,45)],'little':[(-8,3),(-20,60),(0,60),(0,45)],'thumb':[(-20,20),(0,35),(0,40),(0,30)]}
 labels={'index':'검지','middle':'중지','ring':'약지','little':'소지','thumb':'엄지'}
 for ix,f in enumerate(C['hand']['fingers']):
  n=f['name'];fr=ttk.LabelFrame(outer,text=labels[n]);fr.grid(row=ix//2,column=ix%2,sticky='nsew',padx=4,pady=3);variables[n]=[slider(fr,['벌림/회전','첫 굽힘','둘째 굽힘','끝 굽힘'][j],*ab,C['poses']['Open'][n][j]) for j,ab in enumerate(ranges[n])]
 outer.columnconfigure(0,weight=1);outer.columnconfigure(1,weight=1)
 message=tk.StringVar(value='실제 삼각형 CAD 거리 검사 준비 중');ttk.Label(ui,textvariable=message,wraplength=660,foreground='#214a64').pack(padx=10,pady=8)
 ttk.Label(ui,text='팔 체결·배선·재료 강도·모터 응답은 별도 검증 대상입니다.',foreground='#666').pack(pady=(0,8))
 closed=[False];ui.protocol('WM_DELETE_WINDOW',lambda:closed.__setitem__(0,True));pool=ThreadPoolExecutor(max_workers=1);future=None;request_version=None;last=0.;checker=Checker('hand');base_rgba=m.geom_rgba.copy();pose=copy.deepcopy(C['poses']['Open']);arm=(0,0,0,0)
 try:
  with mujoco.viewer.launch_passive(m,d,show_left_ui=False,show_right_ui=False) as viewer:
   viewer.cam.lookat[:]=[0,.01,-.05];viewer.cam.distance=1.2;viewer.cam.azimuth=-65;viewer.cam.elevation=15
   while viewer.is_running() and not closed[0]:
    ui.update()
    if state['dirty']:
     pose={f:[v.get() for v in vs] for f,vs in variables.items()};arm=tuple(v.get() for v in armvars);apply_pose(m,d,inner,pose,arm,Rotation.from_euler('x',90,degrees=True).as_matrix());state['dirty']=False;m.geom_rgba[:]=base_rgba;message.set('자세 검사 중…')
    now=time.monotonic()
    if future is None and now-last>.35 and state['checked_version']!=state['version']:
     request_version=state['version'];state['checked_version']=request_version;future=pool.submit(checker.evaluate,copy.deepcopy(pose),arm,3.5,1.1);last=now
    if future is not None and future.done():
     r=future.result();future=None
     if request_version==state['version']:
      for hit in r['below_threshold']:
       for name in hit['pair']:
        g=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,name)
        if g>=0:m.geom_rgba[g,:3]=[.9,.16,.10] if hit['surface_collision_or_containment'] else [1.,.65,.12]
      row=r['minimum'];pair=' / '.join(row['pair'] or []);message.set(('간섭/근접 발견: ' if r['below_threshold'] else '표본 자세 거리: ')+('≥' if row['lower_bound_only'] else '')+format(row['distance_mm'],'.2f')+'mm '+pair+' · 손 부분만 검사')
    viewer.sync();time.sleep(.02)
 finally:pool.shutdown(wait=False,cancel_futures=True);ui.destroy()
if __name__=='__main__':main()
