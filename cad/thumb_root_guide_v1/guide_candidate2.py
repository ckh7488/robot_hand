"""Separate the keeper seating faces from the cross-brace's forward overhang."""
from guide_model import *
import guide_model as original
ORIGINAL_BUILD=original.build
ORIGINAL_LAYOUT=original.layout
HERE=original.HERE/'selected'

def adjusted_layout():
    rows=ORIGINAL_LAYOUT()
    for row in rows:
        # Begin the inward bend beyond the existing cover edge. Radius stays5.
        shift=(2./row['t'][1])*row['t']
        for key in ['entry','exit']:row[key]=row[key]+shift
        row['socket_transform'][:3,3]+=shift
        row['guide_centers']=[(np.asarray(z)+shift).tolist() for z in row['guide_centers']]
        pieces=[]
        for z in row['pieces']:
            if z['kind']=='line':pieces.append(original.g.line(z['a']+shift,z['z']+shift,z['label']))
            else:pieces.append(original.g.arc(z['a']+shift,z['mid']+shift,z['z']+shift,z['length_mm'],z['label']))
        row['pieces']=pieces
    return rows

original.layout=adjusted_layout

def build():
    parts,design=ORIGINAL_BUILD()
    for row in parts:
        if row['name']=='paired_liner_carrier':
            s=row['local_shape']
            for port in original.layout():
                # A planar front recess belongs to the new carrier; no hand cut.
                pocket=original.box([-5.,2.,-4.],[9.,7.,4.])
                s=s.cut(original.transform(pocket,port['socket_transform']))
            row['local_shape']=original.require(s.clean());row['shape']=original.transform(row['local_shape'],original.WORLD)
    for name in ['guide_candidate2.py','check2.py']:
        design['source_sha256']['cad/thumb_root_guide_v1/'+name]=original.g.drive.sha(original.HERE/name)
    design['correction']='New-carrier front seating pockets remove cross-brace overlap with upper keeper/head.Move liner exit from localY-8to-6mm along the same original slot tangent,keepingR5 and all finger inputs/mounts.Upper inward bend now follows the existing cover edge.No hand cut.'
    design['local_exit_y_mm']=-6.
    return parts,design
