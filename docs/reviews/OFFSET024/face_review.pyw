"""Supplement per-face torus bounding discrepancies, without editing the target."""
import argparse,sys,hashlib,subprocess
from pathlib import Path
import numpy as np
from scipy.optimize import linear_sum_assignment
sys.dont_write_bytecode=True
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,required=True);ap.add_argument('--commit',required=True);ap.add_argument('--initial',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args();root=args.root.resolve();assert not args.output.exists() and not args.output.resolve().is_relative_to(root)
    sys.path.insert(0,str(root/'cad/thumb_sheath_terminal_v2'))
    import selected_model as m
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.BRepClass import BRepClass_FaceClassifier
    from OCP.gp import gp_Pnt2d
    from OCP.TopAbs import TopAbs_IN
    from OCP.OSD import OSD_ThreadPool
    metric=m.drive.module('_offset024_face_metrics',root/'cad/thumb_tendon_route_v1/review_core.py')
    def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=root,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1')
    initial=m.p.read(args.initial);assert initial['target_commit']==args.commit and initial['differences']==['shape_distal_housing_keeper_1'] and len(initial['key_samples'])==20 and all(z['matched'] for z in initial['key_samples']) and sum(z['matched'] for z in initial['geometry_comparisons'])==17 and all(z['matched'] for z in initial['contact_replay'])
    manifest=m.p.read(root/'checkpoints/OFFSET024.manifest.json');frozen={}
    for key in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(m.p.read(root/'AUDIT_EVIDENCE_INDEX.json')[key])
    def guard():return all(sha(root/z)==v['sha256'] for z,v in manifest['files'].items()) and all(sha(root/z)==h for z,h in frozen.items())
    assert guard();fresh=next(z['shape'] for z in m.build()[0] if z['name']=='distal_housing_keeper_1');row=next(z for z in m.p.read(root/'cad/thumb_sheath_terminal_v2/selected/generated/parts/scene.json')['parts'] if z['name']=='distal_housing_keeper_1');saved=m.c.load([row])[0]['shape'];a,b=metric.signature(fresh),metric.signature(saved)
    faces_a=fresh.Faces();faces_b=saved.Faces();errors=[];groups=[];points=[]
    global_error=float(max(abs(np.asarray(a['global_values'])-b['global_values'])))
    if global_error>1e-5:errors.append('global')
    def torus_values(face,values):
        t=BRepAdaptor_Surface(face.wrapped,True).Torus();axis=np.array(t.Axis().Direction().Coord());axis=axis if axis[np.argmax(abs(axis))]>0 else -axis
        return np.r_[values[0],values[-3:],t.Location().Coord(),axis,t.MajorRadius(),t.MinorRadius()]
    for kind in sorted(set(z['kind'] for z in a['faces']+b['faces'])):
        ii=[i for i,z in enumerate(a['faces']) if z['kind']==kind];jj=[j for j,z in enumerate(b['faces']) if z['kind']==kind]
        assert len(ii)==len(jj)
        va=[torus_values(faces_a[i],a['faces'][i]['values']) if kind=='TORUS' else np.asarray(a['faces'][i]['values']) for i in ii];vb=[torus_values(faces_b[j],b['faces'][j]['values']) if kind=='TORUS' else np.asarray(b['faces'][j]['values']) for j in jj]
        cost=np.array([[max(abs(x-y)) for y in vb] for x in va]);rr,cc=linear_sum_assignment(cost)
        for x,y in zip(rr,cc):
            i,j=ii[x],jj[y];error=float(cost[x,y]);groups.append(dict(kind=kind,source_index=i,saved_index=j,values_source=va[x].tolist(),values_saved=vb[y].tolist(),maximum_difference=error,matched=error<=1e-5))
            if error>1e-5:errors.append('face_'+kind)
            if kind!='TORUS':continue
            for direction,first,second in [('source_to_STEP',faces_a[i],faces_b[j]),('STEP_to_source',faces_b[j],faces_a[i])]:
                adapt=BRepAdaptor_Surface(first.wrapped,True);samples=[]
                for u in np.linspace(adapt.FirstUParameter(),adapt.LastUParameter(),13)[1:-1]:
                    for v in np.linspace(adapt.FirstVParameter(),adapt.LastVParameter(),13)[1:-1]:
                        if BRepClass_FaceClassifier(first.wrapped,gp_Pnt2d(float(u),float(v)),1e-7).State()!=TopAbs_IN:continue
                        pt=adapt.Value(float(u),float(v));vertex=m.cq.Vertex.makeVertex(*pt.Coord());distance=vertex.distance(second);samples.append(dict(uv=[float(u),float(v)],point_world_mm=list(pt.Coord()),distance_to_other_trimmed_face_mm=distance))
                ok=len(samples)>=3 and all(z['distance_to_other_trimmed_face_mm']<=1e-5 for z in samples);points.append(dict(source_index=i,saved_index=j,direction=direction,sample_count=len(samples),maximum_distance=max([z['distance_to_other_trimmed_face_mm'] for z in samples],default=None),matched=ok,samples=samples))
                if not ok:errors.append('torus_interior_'+str(i)+'_'+direction)
    # All original edge/vertex metrics retain their original threshold.
    for key in ['edges','vertices']:
        for kind in sorted(set(z['kind'] for z in a[key]+b[key])):
            aa=[z for z in a[key] if z['kind']==kind];bb=[z for z in b[key] if z['kind']==kind];assert len(aa)==len(bb);cost=np.array([[max(abs(np.asarray(x['values'])-y['values'])) for y in bb] for x in aa]);ii,jj=linear_sum_assignment(cost);error=float(max(cost[ii,jj]));groups.append(dict(group=key,kind=kind,maximum_difference=error,matched=error<=1e-5))
            if error>1e-5:errors.append(key+'_'+kind)
    clean=not git('status','--porcelain=v1');result=dict(completed=True,target_commit=args.commit,passed=not errors and guard() and clean,differences=errors,initial_core_passed=False,initial_core_sha256=sha(args.initial),initial_core_preserved=True,initial_native_groups_matched=20,initial_other_geometry_matched=17,initial_contacts_matched=10,comparison_absolute_tolerance=1e-5,adaptive_integration_epsilon=1e-10,global_metric_max_difference=global_error,face_and_boundary_comparisons=groups,torus_interior_checks=points,manifest_files=len(manifest['files']),frozen_files=len(frozen),input_hash_guard=guard(),checkout_clean=clean,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),comparison_code_sha256=sha(__file__),method='Original face TORUS AddOptimal bounding boxes differ while area/center/edges/vertices/global metrics agree.Keep original failed core false.Supplement all torus face area/centroid plus analytic center/axis/major-minor radii,all non-torus original face metrics,all edge/vertex metrics and bidirectional interior point-to-trimmed-face distances.No tolerance change,no source/STEP/clone change.No additional collision/assembly/render/force approval;finite points are not proof of all trimmed-face points or full Boolean equivalence.')
    m.p.write(args.output,result);print('face review',result['passed'],errors,sum(z['sample_count'] for z in points),flush=True);assert result['passed'],errors
if __name__=='__main__':main()
