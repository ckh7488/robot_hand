"""One section-informed S departure ending at the real thumb terminal.

The S bend is a proposed housing shape, not new installed guide hardware.
Only the existing wrist bore and distal captured terminal constrain it.
"""
import round_exit_model as old
m,g,c,p,np,cq,ROOT,HERE=old.m,old.g,old.c,old.p,old.np,old.cq,old.ROOT,old.HERE

def build(q):
    rows=[];paths=[]
    for i,z in enumerate([-3.,2.]):
        ends=m.endpoints(q,i);A=np.array([19.,-18.,z]);P=np.array([33.,-4.,z]);E=old.center_at(z,1,np.pi/4)
        def second(t):return E+np.array([20*(1-np.cos(t)),20*np.sin(t),0.])
        F=second(np.pi/2);B=np.asarray(ends['B']);D=np.asarray(ends['tB']);chord=float(np.linalg.norm(B-F));C=np.array([F,F+np.array([chord/3,0,0]),B-D*chord/3,B])
        arcs=[cq.Edge.makeThreePointArc(cq.Vector(*P),cq.Vector(*old.center_at(z,1,np.pi/8)),cq.Vector(*E)),cq.Edge.makeThreePointArc(cq.Vector(*E),cq.Vector(*second(np.pi/4)),cq.Vector(*F))]
        free=cq.Edge.makeBezier([cq.Vector(*x) for x in C]);prefix=[cq.Edge.makeLine(cq.Vector(*A),cq.Vector(*P)),*arcs,free]
        lengths={};junctions=[]
        for kind,end in [('housing',ends['distal_seat']),('cord',ends['distal_cord'])]:
            edges=prefix+[cq.Edge.makeLine(cq.Vector(*B),cq.Vector(*end))];lengths[kind]=sum(e.Length() for e in edges)
            for j,edge in enumerate(edges):
                sh=m.swept(edge,1.5).cut(m.swept(edge,.65)).clean() if kind=='housing' else m.swept(edge,.3)
                assert sh.isValid() and len(sh.Solids())==1
                rows.append(dict(name='s_exit_'+kind+'_'+str(i)+'_'+str(j),shape=sh,kind=kind,physical_line=i,segment=j,body='palm',color=[.66,.27,.83] if kind=='housing' else [.95,.43,.12],installed_part=False))
            for j in range(4):junctions.append(dict(kind=kind,index=j,gap_mm=(edges[j].endPoint()-edges[j+1].startPoint()).Length,tangent_dot=float(edges[j].tangentAt(1).dot(edges[j+1].tangentAt(0)))))
        paths.append(dict(line=i,entry_mm=A.tolist(),bend_start_mm=P.tolist(),first_bend_end_mm=E.tolist(),second_bend_end_mm=F.tolist(),cubic_controls=C.tolist(),actual_distal_seat_mm=np.asarray(ends['distal_seat']).tolist(),actual_distal_cord_inlet_mm=np.asarray(ends['distal_cord']).tolist(),lengths_mm=lengths,junctions=junctions,arcs_radius_assumed_mm=20.,arcs_angle_deg=[45.,-90.],upstream_motor_connected=False,proposed_bend_shape_physically_retained=False))
    return rows,dict(paths=paths,physical_housings=2,physical_cords=2,CAD_segments_per_physical_line=5,actual_distal_body='thumb_mcp',existing_wrist_attachment='Palm_wrist_saddle,neutral only',new_support_parts=0,whole_housing_length_constancy_verified=False,source='One S alternative informed by actual root material sections;no optimizer/radius sweep;end positions and tangents from real RETAIN026 distal ports.',whole_motor_to_thumb_complete=False)
