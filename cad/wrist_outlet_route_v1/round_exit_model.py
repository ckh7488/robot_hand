"""Two bounded circular departure alternatives, preserving the real bore.

These are local geometric references until their far ends and bend support
are actually connected. A circular coefficient is not installed hardware.
"""
from outlet_survey import m,g,c,p,np,cq,ROOT,HERE
RADIUS=20.
PHI=np.pi/4

def center_at(z,sign,t):
    angle=np.pi/4+sign*t
    return np.array([33.+sign*RADIUS*(np.sin(angle)-np.sin(np.pi/4)),-4.+sign*RADIUS*(np.cos(np.pi/4)-np.cos(angle)),z])

def build(sign):
    rows=[];paths=[];axis=np.array([1.,1.,0.])/np.sqrt(2.);total=36*np.sqrt(2.);before=14*np.sqrt(2.);tail=total-before-RADIUS*PHI
    for i,z in enumerate([-3.,2.]):
        A=np.array([19.,-18.,z]);P=np.array([33.,-4.,z]);E=center_at(z,sign,PHI);D=np.array([np.cos(np.pi/4+sign*PHI),np.sin(np.pi/4+sign*PHI),0.]);B=E+tail*D
        arc=cq.Edge.makeThreePointArc(cq.Vector(*P),cq.Vector(*center_at(z,sign,PHI/2)),cq.Vector(*E));edges=[cq.Edge.makeLine(cq.Vector(*A),cq.Vector(*P)),arc,cq.Edge.makeLine(cq.Vector(*E),cq.Vector(*B))]
        for kind in ['housing','cord']:
            for j,edge in enumerate(edges):
                sh=m.swept(edge,1.5).cut(m.swept(edge,.65)).clean() if kind=='housing' else m.swept(edge,.3)
                assert sh.isValid() and len(sh.Solids())==1
                rows.append(dict(name=('left' if sign==1 else 'right')+'_exit_'+kind+'_'+str(i)+'_'+str(j),shape=sh,kind=kind,physical_line=i,segment=j,body='palm',color=[.66,.27,.83] if kind=='housing' else [.95,.43,.12],installed_part=False))
        paths.append(dict(line=i,start_mm=A.tolist(),bend_start_mm=P.tolist(),bend_end_mm=E.tolist(),free_end_mm=B.tolist(),end_tangent=D.tolist(),length_mm=sum(e.Length() for e in edges),arc_radius_assumed_mm=RADIUS,arc_angle_deg=45.,physical_bend_support_installed=False,far_end_connected=False))
    return rows,dict(turn='left_to_positiveY' if sign==1 else 'right_to_positiveX',paths=paths,engineering_radius_not_supplier_limit=True,housing_OD_ID_mm=[3.,1.3],cord_OD_mm=.6,whole_motor_thumb_connection=False)
