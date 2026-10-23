"""Forward kinematics for the CAD part table; independently cross-checked against FreeCAD exports."""
import json,math,re,numpy as np
from pathlib import Path
D=Path(__file__).resolve().parent
C=json.loads((D/'design.json').read_text())
S=json.loads((D/'scene.json').read_text())
META=S['Open']
def rot(axis,deg):
 a=np.deg2rad(deg);c=np.cos(a);s=np.sin(a)
 if axis==0:return np.array([[1,0,0],[0,c,-s],[0,s,c]])
 if axis==1:return np.array([[c,0,s],[0,1,0],[-s,0,c]])
 return np.array([[c,-s,0],[s,c,0],[0,0,1]])
def T(p=(0,0,0),r=None):
 m=np.eye(4);m[:3,3]=p
 if r is not None:m[:3,:3]=r
 return m
def around(p,axis,deg):return T(p,rot(axis,deg))@T(-np.array(p))
def forward(pose,arm_q=(0,0,0,0)):
 frames={};h=C['hand']['root_axis_offset_z']
 for f in C['hand']['fingers']:
  n=f['name'];rx,ry,rz=f['base_rpy'];B=rot(2,rz)@rot(1,ry)@rot(0,rx)
  A=T(f['root'],B@rot(1,-90));frames[n+'_root_fixed']=A
  O=A@T(r=rot(0,-pose[n][0]));frames[n+'_root_output']=O
  carriage=O@T(r=rot(1,90));frames[n+'_spread_carrier']=carriage
  A=carriage@T((0,0,h))
  for j,tag in enumerate(['mcp','pip','dip']):
   frames[n+'_'+tag+'_fixed']=A;O=A@T(r=rot(0,pose[n][j+1]));frames[n+'_'+tag+'_output']=O
   if j<2:
    A=O@T((0,f['links'][j],0))
    if n=='thumb' and j==0:A=A@T(r=rot(2,f['mcp_mount_yaw']))
 out=[]
 for it in META:
  n=it['name'];m=None
  if n.startswith(('index_','middle_','ring_','little_','thumb_')):
   finger=n.split('_')[0]
   if n.endswith('_spread_carrier'):m=frames[finger+'_spread_carrier']
   else:
    for j in ['root','mcp','pip','dip']:
     if '_'+j+'_' not in n:continue
     fixed=n.endswith(('_motor','_fixed_frame','_root_frame')) or '_'+j+'_base_' in n
     m=frames[finger+'_'+j+('_fixed' if fixed else '_output')];break
  if m is None:m=T(it['pos_mm'],np.array(it['R']))
  out.append(m)
 E=around((0,-C['arm']['wrist_to_palm']-C['arm']['wrist_axis_offset']-C['arm']['elbow_to_wrist_flex'],0),0,arm_q[0])
 wy=-C['arm']['wrist_to_palm']-C['arm']['wrist_axis_offset'];ey=wy-C['arm']['elbow_to_wrist_flex']
 F=around((0,ey+C['arm']['elbow_to_roll'],0),1,arm_q[1])
 P=around((0,wy,0),0,arm_q[2])
 Y=around((C['arm']['wrist_lateral_offset'],-C['arm']['wrist_to_palm'],0),2,arm_q[3])
 for i,it in enumerate(META):
  n=it['name']
  if n in ['AK_elbow_flex','Elbow_fixed_cradle']:continue
  if n in ['Elbow_roll_carrier','AK_forearm_roll']:out[i]=E@out[i]
  elif n in ['Roll_output_flange','Forearm_box_beam','AK_wrist_flex']:out[i]=E@F@out[i]
  elif n in ['Wrist_box_carrier','AK_wrist_deviation']:out[i]=E@F@P@out[i]
  else:out[i]=E@F@P@Y@out[i]
 return np.array(out)
def rigid_body(n):
 if not n.startswith(('index_','middle_','ring_','little_','thumb_')):
  if n in ['AK_elbow_flex','Elbow_fixed_cradle']:return 'fixed'
  if n in ['Elbow_roll_carrier','AK_forearm_roll']:return 'elbow_link'
  if n in ['Roll_output_flange','Forearm_box_beam','AK_wrist_flex']:return 'forearm_link'
  if n in ['Wrist_box_carrier','AK_wrist_deviation']:return 'wrist_link'
  return 'hand_base'
 f=n.split('_')[0]
 if '_root_' in n:return 'hand_base' if n.endswith(('root_motor','root_frame')) or '_root_base_' in n else f+'_spread'
 for j,up in [('mcp','spread'),('pip','mcp'),('dip','pip')]:
  if '_'+j+'_' in n:return f+'_'+up if n.endswith(('_motor','_fixed_frame')) or '_'+j+'_base_' in n else f+'_'+j
 return f+'_spread'
def servo(n):
 m=re.match(r'^(index|middle|ring|little|thumb)_(root|mcp|pip|dip)_',n)
 return m.group(0) if m else None
def eligible(a,b):
 if rigid_body(a['name'])==rigid_body(b['name']):return False
 # Factory TAP screws share submillimetre clearances with their own actuator,
 # measured separately in BRep; general 1mm spacing is for other assemblies.
 if servo(a['name'])==servo(b['name']) and servo(a['name']) is not None:
  if ('_tap' in a['name'] and b['role'].startswith('vendor_')) or ('_tap' in b['name'] and a['role'].startswith('vendor_')):return False
 if b['name'] in a.get('mating_with',[]) or a['name'] in b.get('mating_with',[]):return False
 if a['role'].startswith('vendor_') and b['role'].startswith('vendor_') and servo(a['name'])==servo(b['name']) and servo(a['name']) is not None:return False
 return True
def verify():
 maxpos=0.;maxrot=0.
 for name,pose in C['poses'].items():
  m=forward(pose)
  for i,it in enumerate(S[name]):
   maxpos=max(maxpos,float(np.max(np.abs(m[i,:3,3]-it['pos_mm']))))
   maxrot=max(maxrot,float(np.max(np.abs(m[i,:3,:3]-it['R']))))
 assert maxpos<1e-7 and maxrot<1e-9,(maxpos,maxrot)
 return {'poses':len(S),'parts_per_pose':len(META),'max_position_error_mm':maxpos,'max_rotation_matrix_error':maxrot,'passed':True}
if __name__=='__main__':print(json.dumps(verify(),indent=2))
