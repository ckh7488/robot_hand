"""Outward-open straight cord slots in the saved parent link, IP-local mm.

These cuts are open wall slots, not retained guide inserts.
They do not establish wear, surface finish, preload or full-hand tendon routing.
"""
from __future__ import annotations


def build_parent(cq, parent, routing, geometry):
    geometry.require_solid(parent, 'native parent')
    result=parent.copy();tools=[];journal=[]
    for i,row in enumerate(routing.route(0)):
        a=row['motor_contact_mm'];b=row['joint_contact_mm'];x=row['track_x_mm']
        def z_at(y):return a[2]+(y-a[1])*(b[2]-a[2])/(b[1]-a[1])
        lo,hi=-47.,2.
        polygon=([(lo,-13.),(hi,-13.),(hi,z_at(hi)+.6),(lo,z_at(lo)+.6)] if i==0 else
                 [(lo,z_at(lo)-.6),(hi,z_at(hi)-.6),(hi,13.),(lo,13.)])
        tool=cq.Workplane('YZ').polyline(polygon).close().extrude(1.2,clean=False).val().translate((x-.6,0,0))
        result=geometry.boolean(cq,'cut',result,tool)
        journal.append(dict(track=i,**geometry.require_solid(result,'parent passage '+str(i))))
        tools.append(tool)
    return dict(parent=result,tools=tools,operation_journal=journal,
        parameters=dict(open_slot_width_mm=1.2,nominal_cord_diameter_mm=.6,
                        ideal_lateral_clearance_mm=.3,y_span_mm=[-47.,2.],outer_z_mm=[-13.,13.],
                        top_faceplate_removed_for_machining_and_cord_laying=True),
        limitations=['Nominal clearance only; actual rope diameter/position and link tolerances remain unmeasured.',
                     'Open groove edges require a specified deburr/finish; raw printed running surfaces are not approved.',
                     'The upper slot cutter overlaps the separate dark faceplate: machine the parent as an individual part and lay the cord before refitting that plate.',
                     'Slot stiffness, loose-rope derailment, cover removal/installation tools and edge finishing remain unqualified.',
                     'This fixed-parent bench route does not define tendon guides across other moving joints.'])
