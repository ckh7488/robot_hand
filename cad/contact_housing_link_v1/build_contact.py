"""One coupled neutral candidate, then native material checks of actual tubes."""
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import contact_solver as s
a,m,g,c,p,np,cq,ROOT,HERE=s.a,s.m,s.g,s.c,s.p,s.np,s.cq,s.ROOT,s.HERE
from OCP.TColgp import TColgp_Array1OfPnt
from OCP.TColStd import TColStd_Array1OfReal,TColStd_Array1OfInteger
from OCP.Geom import Geom_BSplineCurve
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
from OCP.gp import gp_Pnt
from OCP.OSD import OSD_ThreadPool

def spline(C):
    poles=TColgp_Array1OfPnt(1,8);knots=TColStd_Array1OfReal(1,6);mult=TColStd_Array1OfInteger(1,6)
    for i,z in enumerate(C):poles.SetValue(i+1,gp_Pnt(*z))
    for i,z in enumerate([0.,.2,.4,.6,.8,1.]):knots.SetValue(i+1,z);mult.SetValue(i+1,4 if i in [0,5] else 1)
    return cq.Edge(BRepBuilderAPI_MakeEdge(Geom_BSplineCurve(poles,knots,mult,3,False)).Edge())

def tubes(result):
    aa=s.anchors(np.zeros(20));rows=[];lengths=[];C=np.asarray(result['controls'])
    for i in [0,1]:
        e=aa[i]['terminal'];up=spline(C[i*2]);down=spline(C[i*2+1]);guide=[a.prior.a.edge(z) for z in a.prior.a.definitions(i)];upper=[g.original.transform(ed,a.frame()) for ed in a.edges()] if i==1 else []
        for kind in ['housing','cord']:
            first=cq.Edge.makeLine(cq.Vector(*e['motor_seat' if kind=='housing' else 'motor_cord']),cq.Vector(*e['A']));last=cq.Edge.makeLine(cq.Vector(*e['B']),cq.Vector(*e['distal_seat' if kind=='housing' else 'distal_cord']));edges=[first]+[up.trim(j/5,(j+1)/5) for j in range(5)]+guide+[down.trim(j/5,(j+1)/5) for j in range(5)]+upper+[last]
            lengths.append(dict(line=i,kind=kind,total_length_mm=sum(z.Length() for z in edges)))
            for j,ed in enumerate(edges):
                sh=m.swept(ed,1.5).cut(m.swept(ed,.65)).clean() if kind=='housing' else m.swept(ed,.3)
                assert sh.isValid() and len(sh.Solids())==1,(i,kind,j)
                rows.append(dict(name='contact_'+kind+'_'+str(i)+'_'+str(j),shape=sh,physical_line=i,kind=kind,segment=j,body='palm',length_mm=ed.Length(),color=[.63,.25,.8] if kind=='housing' else [.97,.45,.12]))
    return rows,lengths

def main():
    out=HERE/'reviews/check.json';assert not out.exists();start=time.monotonic();data,pack,guide,wrist=s.prior.inputs();hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,np.zeros(20));upper=c.load([z for z in p.read(ROOT/'cad/thumb_entry_bend_v1/generated/parts/scene.json')['parts'] if z['installed_part_candidate']],data[0]);hand+=upper;assert len(hand)==397;obstacles=hand+pack+data[6]+data[7]+wrist;assert len(obstacles)==862
    result=s.solve(np.zeros(20),obstacles,HERE/'reviews/solve.json');rows,lengths=tubes(result);scene=a.prior.export(rows,HERE/'generated/neutral');tests={k:c.material(rows,ob) for k,ob in [('hand',hand),('packaging',pack),('electrical',data[6]),('arm',data[7]),('wrist_guide',wrist)]};internal=[c.material([z],rows[j+1:]) for j,z in enumerate(rows)];alltests=[*tests.values(),*internal]
    for n in ['material_field.py','contact_solver.py','build_contact.py']:data[0]['cad/contact_housing_link_v1/'+n]=m.drive.sha(HERE/n)
    report=dict(completed=True,source_sha256=data[0],solver=result,scene=scene,new_CAD_rows=len(rows),physical_housings=2,physical_middle_cords=2,new_hardware_rows=0,neutral_only=True,native=tests,internal=internal,geometric_lengths_mm=lengths,native_candidate_clear=all(m.drive.clear(z) for z in alltests),candidate_admissible=bool(result['admissible_sampled_solution'] and all(m.drive.clear(z) for z in alltests)),pairs_in_scope=sum(z['pairs_in_scope'] for z in alltests),AABB_native_pairs=sum(z['exact_pairs'] for z in alltests),overlap_records=sum(len(z['overlaps']) for z in alltests),operation_failures=sum(len(z['operation_failures']) for z in alltests),OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,actual_material_elastic_equilibrium_or_friction_tension_verified=False,whole_link_installed=False,whole_hand_drive_approved=False,old_local_reference_parts_not_duplicated=True,input_hash_guard=all(m.drive.sha(ROOT/z)==h for z,h in data[0].items()),elapsed_s=time.monotonic()-start)
    p.write(out,report);print('native',report['candidate_admissible'],report['overlap_records'],report['operation_failures'],flush=True)
if __name__=='__main__':main()
