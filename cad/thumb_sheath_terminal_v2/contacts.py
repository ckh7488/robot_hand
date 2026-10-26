"""Nominal contact witnesses only, not load or friction qualification."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import selected_model as m
g,c,p,np,cq,ROOT,HERE=m.g,m.c,m.p,m.np,m.cq,m.ROOT,m.HERE
from OCP.OSD import OSD_ThreadPool
def main():
    out=HERE/'reviews/contacts.json';assert not out.exists();r=p.read(HERE/'reviews/static.json');source={};new=c.load(p.read(ROOT/r['scene'])['parts'],source);guide=c.load(p.read(ROOT/'cad/thumb_guide_assembly_v2/generated/parts/scene.json')['parts'],source);housing=c.load(p.read(ROOT/r['housing_scene'])['parts'],source);by={z['name']:z for z in new+guide+housing};rows=[]
    source['cad/thumb_sheath_terminal_v2/contacts.py']=m.drive.sha(Path(__file__))
    for row in g.original.layout():
        i,side,Q=row['cord'],row['side'],row['socket_transform'];dx,du=(0.,0.) if i==0 else (14.,-17.)
        specs=[('housing_shoulder','distal_short_housing_'+str(i),'distal_flanged_ferrule_'+str(i),(dx+1,-10+du,0),(0,1,0)),('flange_rear','distal_housing_keeper_'+str(i),'distal_flanged_ferrule_'+str(i),(dx+1.5,-16+du,1.8*side),(0,1,0)),('flange_front','distal_flanged_ferrule_'+str(i),'distal_housing_keeper_'+str(i),(dx+1.5,-14+du,1.8*side),(0,1,0)),('keeper_to_carrier','paired_liner_carrier','distal_housing_keeper_'+str(i),(4,-2,0),(0,0,side)),('split_shoe_face','distal_housing_shoe_'+str(i),'distal_housing_keeper_'+str(i),(dx+5.8,-11+du,0),(0,0,side))]
        for label,a,b,P,N in specs:
            W=g.original.WORLD@Q;point=np.asarray(P);normal=np.asarray(N);pts=[(W@np.r_[point-normal*.05,1])[:3],(W@np.r_[point+normal*.05,1])[:3]];s,t=by[a]['shape'],by[b]['shape'];classification=[[bool(sh.isInside(cq.Vector(*x),1e-6)) for x in pts] for sh in [s,t]]
            rows.append(dict(id=label+'_'+str(i),a=a,b=b,point_world_mm=((W@np.r_[point,1])[:3]).tolist(),distance_mm=s.distance(t),classification=classification,matched=classification==[[True,False],[False,True]] and s.distance(t)<1e-5))
    ext=c.load(p.read(ROOT/r['cord_scene'])['parts'],source);existing=g.cord_pieces(np.zeros(20));native=c.material(ext,existing);Q=g.original.layout()[1]['socket_transform'];W=g.original.WORLD@Q
    newpoint=(W@np.r_[m.paths()[0]['a'],1])[:3];oldpoint=(g.original.WORLD@np.r_[g.original.layout()[1]['entry'],1])[:3]
    tangent_new=-(W[:3,:3]@m.paths()[0]['tangent']);tangent_old=W[:3,1]
    junction=dict(endpoint_error_mm=float(np.linalg.norm(newpoint-oldpoint)),tangent_dot=float(np.dot(tangent_new,tangent_old)),extension_length_mm=sum(z['length_mm'] for z in m.paths()),constant_on_thumb_mcp_body=True,extra_length_is_not_selected_adjuster_stroke=True)
    p.write(out,dict(completed=True,source_sha256=source,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),contacts=rows,nominal_contacts_verified=all(z['matched'] for z in rows),extension_vs_existing_local_cords=native,nominal_junction=junction,full_six_DOF_or_strength_approved=False,housing_pullout_retention_verified=False,scope='10 selected witness positions,on either side0.05mm;2extension pieces vs12local cord pieces at neutral;analytic tangent/endpoint.Not full face area,force/friction,thread torque,material stiffness or full assembly verification.'));print('contacts',[(z['id'],z['matched']) for z in rows],flush=True)
if __name__=='__main__':main()
