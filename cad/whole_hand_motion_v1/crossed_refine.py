"""Photo-requested crossed index/middle pose on the saved 364-part hand."""
import argparse,copy,hashlib,importlib.util,json,math,sys
from pathlib import Path
import numpy as np
import cadquery as cq

HERE=Path(__file__).resolve().parent
def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def mod(name,p):
    s=importlib.util.spec_from_file_location(name,p);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    root=HERE.parents[1];candidate=root/'cad/thumb_ip_drive_v1';hashes={}
    def bind(p,h=None):
        actual=sha(p)
        if h and h!=actual:raise ValueError('Stale source '+str(p))
        hashes[Path(p).relative_to(root).as_posix()]=actual;return p
    bind(__file__);base=read(bind(HERE/'reviews/discovery.json'));design=base['requests']['design']
    scene=read(bind(candidate/'generated/scene.json'));parts={}
    native=mod('_cross_native',bind(root/'cad/gesture_hand_v3/audit_gestures.py'));bind(root/'cad/articulation_drive_v2/verify.py');v=native.load_v2(root)
    sys.path.insert(0,str(root/'cad/gesture_hand_v3'));poses=mod('_cross_poses',bind(root/'cad/gesture_hand_v3/poses.py'))
    for r in scene['hand']:
        for key,hkey in [('step','sha256_step'),('mesh','sha256_stl')]:
            p=root/r[key] if r['location']=='repo' else candidate/'generated'/r[key];bind(p,r[hkey])
            if key=='step':
                s=cq.importers.importStep(str(p)).val()
                if not s.isValid() or len(s.Solids())!=1:raise ValueError('Invalid STEP')
                parts[r['name']]=s
    engine=native.PairScreen(v,parts,scene['hand'],design,0.,True)
    initial=np.array(next(c['angles_deg'] for c in base['cases'] if c['id']=='gesture_scissors'))
    bind(HERE/'crossed_fingers.py');bind(HERE/'reviews/crossed_initial.json')
    options=[([0,0,0,0],[20,25,0,0]),([0,0,0,0],[20,30,0,0]),
             ([0,0,0,0],[22,35,0,0]),([-2,0,0,0],[18,30,0,0]),
             ([-3,0,0,0],[18,35,0,0]),([0,0,0,0],[22,40,0,0]),
             ([-4,30,0,0],[18,0,0,0]),([-6,35,0,0],[16,0,0,0])]
    report=dict(completed=False,source_sha256=hashes,design=design,cases=[],paths=[],selected=None,
        input_description='User supplied fingers-crossed emoji; index and middle cross, ring/little folded, thumb across folded digits. No claim of pixel-identical anatomy.',
        scope='364 nominal saved hand parts; finite target poses and <=5-degree two-stage paths. No cords, motor actuation, tolerances, continuous or physical approval.')
    def save():
        a.output.parent.mkdir(parents=True,exist_ok=True);t=a.output.with_suffix('.partial');t.write_text(json.dumps(report,separators=(',',':'),allow_nan=False),encoding='utf-8');t.replace(a.output)
    cache={}
    def evaluate(q):
        key=tuple(float(x).hex() for x in q)
        if key not in cache:
            r=engine.evaluate(q);r.pop('exact_candidate_results');r.pop('near_pairs');cache[key]=r
        return cache[key]
    def crossings(q):
        chains=poses.forward_kinematics(design,q)['chains'];first,second=chains['index'],chains['middle'];out=[]
        for i in range(3):
            for j in range(3):
                u,vv=first[i+1]-first[i],second[j+1]-second[j];matrix=np.column_stack((u[:2],-vv[:2]))
                if abs(np.linalg.det(matrix))<1e-9:continue
                t,z=np.linalg.solve(matrix,second[j,:2]-first[i,:2])
                if 0<t<1 and 0<z<1:
                    p=first[i]+t*u;other=second[j]+z*vv;out.append(dict(index_segment=i+1,middle_segment=j+1,projected_xy_mm=p[:2].tolist(),depth_separation_mm=float(p[2]-other[2])))
        return out
    for i,(index,middle) in enumerate(options):
        q=initial.copy();q[:4]=index;q[4:8]=middle;poses.validate_angles(q,design);r=evaluate(q)
        row=dict(id='refined_cross_'+str(i+1),angles_deg=q.tolist(),result=r,projected_centerline_crossings=crossings(q));report['cases'].append(row);save()
        if r['collision_count'] or r['operation_failures'] or not row['projected_centerline_crossings']:continue
        lifted=q.copy();lifted[0]=initial[0];lifted[4]=initial[4];waypoints=[initial,lifted,q];obs=[]
        for segment,(lo,hi) in enumerate(zip(waypoints,waypoints[1:])):
            steps=max(1,int(math.ceil(max(abs(hi-lo))/5)))
            for k in range(steps+1):
                if segment and k==0:continue
                value=lo+(hi-lo)*k/steps;res=evaluate(value);obs.append(dict(segment=segment,fraction=k/steps,result=res))
        passed=all(not o['result']['collision_count'] and not o['result']['operation_failures'] for o in obs)
        report['paths'].append(dict(target=row['id'],source='gesture_scissors',method='Move fingers to separate depth planes, then move index/middle laterally toward crossing',waypoints=[x.tolist() for x in waypoints],maximum_joint_step_deg=5,observations=obs,sampled_clear=passed))
        if passed:report['selected']=row['id'];save();break
        save()
    report['completed']=True;report['input_hash_guard']=dict(passed=all(sha(root/k)==h for k,h in hashes.items()));report['whole_hand_motion_approved']=False;save()
if __name__=='__main__':main()
