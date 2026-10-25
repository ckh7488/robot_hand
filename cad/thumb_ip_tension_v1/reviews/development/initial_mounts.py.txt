"""Actual M4 post-to-common-plate stacks from already saved supplier facts."""
import math

def build(cq,geo,holes):
    parts=[]; contacts=[]; assembly=[]
    def cyl(x,y,r,a,b): return cq.Solid.makeCylinder(r,b-a,cq.Vector(x,y,a),cq.Vector(0,0,1))
    def hx(x,y,af,a,b):
        r=af/math.sqrt(3); pts=[(x+r*math.cos(i*math.pi/3),y+r*math.sin(i*math.pi/3)) for i in range(6)]
        return cq.Workplane('XY').workplane(offset=a).polyline(pts).close().extrude(b-a,clean=False).val()
    def op(kind,a,*b): return geo.boolean(cq,kind,a,*b)
    def add(n,s,role):
        geo.require_solid(s,n); parts.append(dict(name=n,shape=s,body='fixed',role=role,color=[.63,.66,.7])); return n
    def seat(a,b,x,y,z,area,radius=4.5):
        contacts.append(dict(a=a,b=b,plane_origin_mm=[x,y,z],normal=[0.,0.,1.],minimum_area_mm2=area,patch_radius_mm=radius,contact_frame='fixed'))
    for i,h in enumerate(holes,1):
        x,y=h['xy']; top=h['underhead_z']; length=h['bolt_length_mm']; foot=h['contact_part']
        if (top,length,h['washer_thickness_mm'])!=(-23.1,20.,.9): raise ValueError('Unexpected post stack')
        p=f'tension_adjuster_mount_{i}'
        shaft=op('fuse',cyl(x,y,2,top-length,top),cyl(x,y,3.5,top,top+4))
        sn=add(p+'_screw',op('cut',shaft,hx(x,y,3,top+2,top+4.01)),'Accu_SSC_M4_20_A2_major_envelope')
        uw=add(p+'_upper_washer',op('cut',cyl(x,y,4.5,top-.9,top),cyl(x,y,2.15,top-.91,top+.01)),'Accu_HPW_M4_A2_max_thickness')
        lw=add(p+'_lower_washer',op('cut',cyl(x,y,4.5,-36.9,-36),cyl(x,y,2.15,-36.91,-35.99)),'Accu_HPW_M4_A2_max_thickness')
        nt=add(p+'_nut',op('cut',hx(x,y,7,-40.1,-36.9),cyl(x,y,2,-40.11,-36.89)),'Accu_HPN_M4_A2_major_bore')
        seat(sn,uw,x,y,top,22,3.5); seat(uw,foot,x,y,top-.9,45)
        seat(lw,'bench_common_plate',x,y,-36,45); seat(nt,lw,x,y,-36.9,26)
        patch=[[x-7,y-6,-28.1],[x+16,y+7,-27.9]] if x<0 else [[x-16,y-6,-28.1],[x+7,y+7,-27.9]]
        contacts.append(dict(a=foot,b='bench_common_plate',plane_origin_mm=[x,y,-28],normal=[0.,0.,1.],
            minimum_area_mm2=250.,patch_box_mm=patch,contact_frame='fixed'))
        assembly.append(dict(kind='axial_screw',part=sn,underhead=[x,y,top],axis=[0.,0.,1.],length_mm=20.,
            head_height_mm=4.,head_radius_mm=3.5,shaft_radius_mm=2.,entry_travel_mm=30.,driver_radius_mm=2.,driver_length_mm=40.,
            stage_excluded=[],access_frame='fixed',stage='fix_post_to_common_plate'))
        assembly.append(dict(kind='axial_nut',part=nt,mate_screw=sn,upper_face=[x,y,-36.9],axis=[0.,0.,-1.],
            across_flats_mm=7.,height_mm=3.2,entry_travel_mm=30.,socket_max_dia_mm=11.8,socket_length_mm=50.,
            stage_excluded=[sn,nt],access_frame='fixed',stage='before_table_placement'))
    return dict(parts=parts,contacts=contacts,assembly=assembly)
