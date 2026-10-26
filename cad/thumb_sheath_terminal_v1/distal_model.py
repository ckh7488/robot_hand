"""Candidate distal housing abutments, supported by the existing guide keeper seats.

The captured ferrule is positive axial hardware. Housing-to-ferrule pull-out
retention and load capacity remain unqualified; a loose housing is not clamped.
"""
import sys,math
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'cad/thumb_guide_assembly_v2'))
import radial_model as base
from material_cache import install
g,c,p,np,cq=base.g,base.c,base.p,base.np,base.cq
install(c)
drive=g.g.drive
def bx(lo,hi):return g.original.box(lo,hi)
def cy(r,a,b):return g.original.cylinder_y(r,a,b)
def zbox(lo,hi,side):
    a,b=sorted([lo[2]*side,hi[2]*side]);return bx([lo[0],lo[1],a],[hi[0],hi[1],b])
def zcyl(r,x,u,a,b,side):return g.original.cyl(r,b-a,(x,u,a*side),(0,0,side))
def build():
    prior,design=base.build();retained=[z for z in prior if not z['name'].startswith('liner_flange_keeper_')];new=[]
    def add(name,s,Q,role,color):
        s=g.original.require(s.clean());local=g.original.transform(s,Q)
        new.append(dict(name=name,local_shape=local,shape=g.original.transform(local,g.original.WORLD),body='thumb_mcp',role=role,color=color))
    for row in g.original.layout():
        i,side,Q=row['cord'],row['side'],row['socket_transform']
        outer=zbox([-6.5,-19,0],[9,4,4],side)
        inner=zbox([-6.5,-19,-4],[6.5,-8,0],side)
        tools=[cy(.65,-19.01,4.01),cy(2.05,-19.01,-9),cy(2.7,-16,-14),cy(1.6,0,2),bx([1.1,0,-.6],[2.7,2,.6]),zcyl(1.1,6,0,-6.01,5.21,side)]
        for x in [-4.5,4.5]:tools.append(zcyl(1.1,x,-12.5,-6.01,5.21,side))
        for t in tools:outer=outer.cut(t);inner=inner.cut(t)
        add('distal_housing_keeper_'+str(i),outer,Q,'extended_outer_keeper_on_existing_carrier_and_M2_axis',[.22,.65,.5])
        add('distal_housing_shoe_'+str(i),inner,Q,'removable_inner_ferrule_capture_half',[.24,.53,.65])
        ferrule=cy(2.,-24,-9).fuse(cy(2.6,-16,-14)).cut(cy(1.55,-24.01,-10)).cut(cy(.65,-10,-8.99))
        add('distal_flanged_ferrule_'+str(i),ferrule,Q,'custom_compression_end_stop_with_captured_axial_flange_not_housing_pullout_clamp',[.85,.64,.23])
        for k,x in enumerate([-4.5,4.5]):
            bolt=zcyl(1.,x,-12.5,-6,4,side).fuse(zcyl(1.9,x,-12.5,4,5.2,side))
            add('distal_shoe_M2x10_'+str(i)+'_'+str(k),bolt,Q,'nominal_smooth_fastener_envelope_no_thread_drive_approval',[.48,.5,.54])
            pts=[cq.Vector(x+2.31*math.cos(a*math.pi/3),-12.5+2.31*math.sin(a*math.pi/3),-5.6*side) for a in range(6)]
            nut=cq.Solid.extrudeLinear(cq.Wire.makePolygon(pts,close=True),[],cq.Vector(0,0,1.6*side)).cut(zcyl(1.,x,-12.5,-5.61,-3.99,side))
            add('distal_shoe_M2_nut_'+str(i)+'_'+str(k),nut,Q,'nominal_M2_nut_no_threads',[.48,.5,.54])
    design=dict(design,replaced_guide_parts=['liner_flange_keeper_0','liner_flange_keeper_1'],distal_sheath_OD_ID_candidate_mm=[3.,1.3],ferrule_socket_OD_ID_mm=[4.,3.1],ferrule_flange_OD_length_mm=[5.2,2.],housing_shoulder_u_mm=-10.,ferrule_carrier_shoulder_u_mm=-9.,liner_entry_u_mm=-12.,housing_pullout_clamp_installed=False,long_housing_installed=False,whole_transmission_complete=False,hand_geometry_changed=False,motor_positions_changed=False)
    return new,retained,design
def witnesses():
    rows=[]
    for z in g.original.layout():
        s=cy(1.5,-34,-10).cut(cy(.65,-34.01,-9.99));local=g.original.transform(s,z['socket_transform'])
        rows.append(dict(name='distal_short_housing_'+str(z['cord']),shape=g.original.transform(local,g.original.WORLD),local_shape=local,body='thumb_mcp',role='24mm_reference_only_no_long_housing_or_pullout_approval',color=[.65,.35,.8]))
    return rows
