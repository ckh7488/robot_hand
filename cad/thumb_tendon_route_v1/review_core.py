"""Exact-snapshot reproduction including failures; not drive acceptance."""
import argparse,subprocess,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import calibrated as g
import calibrated_audit as selected
import audit as initial
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.OSD import OSD_ThreadPool
from scipy.optimize import linear_sum_assignment
c,b,m,p,cq,np,HERE,ROOT=g.c,g.b,g.m,g.p,g.cq,g.np,g.HERE,g.ROOT


def signature(shape):
    vol=GProp_GProps();BRepGProp.VolumeProperties_s(shape.wrapped,vol,1e-10,False,False)
    area=GProp_GProps();BRepGProp.SurfaceProperties_s(shape.wrapped,area,1e-10,False)
    def values(s,prop):return [*np.asarray(m.pack._bbox(s)).ravel(),*prop.CentreOfMass().Coord()]
    faces=[]
    for f in shape.Faces():
        prop=GProp_GProps();BRepGProp.SurfaceProperties_s(f.wrapped,prop,1e-10,False)
        faces.append(dict(kind=f.geomType(),values=[prop.Mass(),*values(f,prop)]))
    return dict(valid=shape.isValid(),solids=len(shape.Solids()),global_values=[vol.Mass(),area.Mass(),*values(shape,vol)],faces=faces,
        edges=[dict(kind=z.geomType(),values=[z.Length(),*np.asarray(m.pack._bbox(z)).ravel(),*z.Center().toTuple()]) for z in shape.Edges()],
        vertices=[dict(kind='point',values=list(z.toTuple())) for z in shape.Vertices()])


def match(a,z):
    errors=[];maximum=0.
    if not a['valid'] or not z['valid'] or a['solids']!=1 or z['solids']!=1:errors.append('valid/solids')
    difference=float(max(abs(np.asarray(a['global_values'])-z['global_values'])));maximum=max(maximum,difference)
    if difference>1e-5:errors.append('global')
    for key in ['faces','edges','vertices']:
        for kind in set(q['kind'] for q in a[key]+z[key]):
            aa=[q for q in a[key] if q['kind']==kind];zz=[q for q in z[key] if q['kind']==kind]
            if len(aa)!=len(zz):errors.append(key+' count');continue
            cost=np.asarray([[max(abs(np.asarray(x['values'])-y['values'])) for y in zz] for x in aa]);ii,jj=linear_sum_assignment(cost)
            difference=float(max(cost[ii,jj]));maximum=max(maximum,difference)
            if difference>1e-5:errors.append(key+' '+kind)
    return dict(matched=not errors,mismatches=errors,max_numeric_difference=maximum)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    assert not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1')
    manifest=p.read(ROOT/'checkpoints/TENDON019.manifest.json');frozen={}
    for key in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[key])
    def guard():return all(g.drive.sha(ROOT/z)==v['sha256'] for z,v in manifest['files'].items()) and all(g.drive.sha(ROOT/z)==v for z,v in frozen.items())
    assert guard();expected=p.read(HERE/'reviews/calibrated.json');first=p.read(HERE/'reviews/audit.json');interfaces=p.read(HERE/'reviews/interfaces.json')
    assert expected['completed'] and expected['input_hash_guard'] and not expected['sampled_free_guide_envelopes_clear']
    data=initial.inputs();differences=[];samples=[];comparisons=[]
    opening=next(z for z in expected['cases'] if z['id']=='open');saved=c.load(p.read(ROOT/opening['scene'])['parts'])
    rows,kin=g.coordinated(opening['angles_deg']);fresh=g.tubes(rows);lookup={z['name']:z for z in fresh}
    midpoints={}
    for row in rows:
        for i,z in enumerate(row['pieces']):midpoints['cord_'+str(row['cord'])+'_'+str(i)+'_'+z['label']]=(z['a']+z['z'])/2 if z['kind']=='line' else z['mid']
    for row in saved:
        source=lookup[row['name']]['shape'];got=match(signature(source),signature(row['shape']));point=cq.Vector(*midpoints[row['name']])
        got.update(name=row['name'],centerline_interior=[bool(s.isInside(point,1e-6)) for s in [source,row['shape']]])
        if got['centerline_interior']!=[True,True]:got['matched']=False;got['mismatches'].append('interior')
        comparisons.append(got)
        if not got['matched']:differences.append('shape_'+row['name'])
    def compare(label,got,want):
        def mapping(r):return {(z['lead'],z['part']):z['material_intersection_mm3'] for z in r['overlaps']}
        a,z=mapping(got),mapping(want);ok=a.keys()==z.keys() and all(abs(a[k]-z[k])<1e-5 for k in a.keys()&z.keys()) and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope']
        samples.append(dict(id=label,native=got,matched=ok));print(label,ok,flush=True)
        if not ok:differences.append(label)
    for label in ['open','parent_mid','pinch25']:
        want=next(z for z in expected['cases'] if z['id']==label);got,_,_=selected.evaluate(want['angles_deg'],data)
        for key in ['packaging','wires','neutral_arm','hand','between_cords']:compare('calibrated_'+label+'_'+key,got['native'][key],want['native'][key])
    rear=first['plans'][0];got,_,_=initial.evaluate(rear['angles_deg'],rear['plan'],data)
    for key in ['packaging','hand']:compare('original_rear_turn_'+key,got['native'][key],rear['native'][key])
    mid=next(z for z in first['cases'] if z['id']=='parent_mid');got,_,_=initial.evaluate(mid['angles_deg'],mid['plan'],data)
    compare('original_parent_mid_hand',got['native']['hand'],mid['native']['hand'])
    length=[]
    for want in expected['parent_length_study']:
        rows,kin=g.coordinated(want['angles_deg']);ok=all(abs(kin[k]-want['kinematics'][k])<1e-7 for k in ['motor_angle_deg','common_length_residual_mm','differential_residual_mm'])
        length.append(dict(fraction=want['fraction'],matched=ok,kinematics=kin))
        if not ok:differences.append('length_'+str(want['fraction']))
    # Reimported saved cord contact to current nominal clamp faces, only open.
    hand=data[4];motors={z['name']:z['shape'] for z in g.drive.moved(data[8],opening['kinematics']['motor_angle_deg'])};contacts=[]
    for want in [z for z in interfaces['contacts'] if z['case']=='open']:
        cord=next(z for z in saved if z['cord']==want['cord'] and z['role']==want['role']);shape=motors[want['part']] if want['part'] in motors else hand[want['part']]
        distance=cord['shape'].distance(shape);ok=abs(distance-want['distance_mm'])<1e-5
        contacts.append(dict(**want,replayed_distance_mm=distance,matched=ok))
        if not ok:differences.append('contact_'+str(want))
    clean=not git('status','--porcelain=v1')
    result=dict(completed=True,target_commit=args.commit,passed=not differences and guard() and clean,differences=differences,
        key_samples=samples,cord_geometry_comparisons=comparisons,parent_length_replay=length,nominal_contacts=contacts,
        manifest_files=len(manifest['files']),frozen_files=len(frozen),input_hash_guard=guard(),checkout_clean=clean,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),independent_audit=False,
        design_approved=False,installed_guides=False,tensioner_installed=False,whole_hand_drive_complete=False,
        scope='18native groups:3calibrated snapshots x5groups,original rear turn packaging/hand and original parent-mid hand failure;34regenerated/opening saved rope tubes with adaptive metric signatures and centerline interior;17length samples,8open anchor distances.All364hand/358fixed+10posed motor/64wire+outerbox/10neutralarm inputs imported.No full motion,remaining snapshot replay,render,guide mounting,force/tension or physical approval.Passed means failures and partial clear geometry reproduce,not a successful hand drive.')
    p.write(args.output,result);assert result['passed']


if __name__=='__main__':main()
