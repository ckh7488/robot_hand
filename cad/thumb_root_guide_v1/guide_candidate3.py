"""Rear nut access pocket in the new carrier, preserving hand and liner path."""
from guide_candidate2 import *
import guide_candidate2 as previous
HERE=original.HERE/'selected2'

def build():
    parts,design=previous.build()
    for row in parts:
        if row['name']=='paired_liner_carrier':
            s=row['local_shape']
            for port in original.layout():
                tool=original.cylinder_y(2.8,-12.,-4.,x=6.)
                s=s.cut(original.transform(tool,port['socket_transform']))
            row['local_shape']=original.require(s.clean());row['shape']=original.transform(row['local_shape'],original.WORLD)
    for name in ['guide_candidate3.py','check3.py']:
        design['source_sha256']['cad/thumb_root_guide_v1/'+name]=original.g.drive.sha(original.HERE/name)
    design['rear_nut_access_counterbore_radius_mm']=2.8
    design['rear_nut_access_correction']='Remove only new-carrier cross-brace material behind the nut seat.Nut remains on rear planeu=-4mm;hand,liner path,motor and keeper positions unchanged.'
    return parts,design
