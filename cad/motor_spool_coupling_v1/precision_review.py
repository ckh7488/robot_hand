"""Supplement the preserved failed scalar signature at an unchanged exact SHA.

Use adaptive mass integration, retaining the original absolute comparison limit.
No shape, collision threshold, source target or first core result is changed.
"""
import argparse,hashlib,importlib.util,json,subprocess,sys
from pathlib import Path
import numpy as np
from scipy.optimize import linear_sum_assignment
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,required=True);ap.add_argument('--commit',required=True)
    ap.add_argument('--initial',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    root=args.root.resolve();assert not args.output.exists() and not args.output.resolve().is_relative_to(root)
    sys.dont_write_bytecode=True;sys.path.insert(0,str(root/'cad/motor_spool_coupling_v1'))
    import candidate2 as g
    def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
    def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=root,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1')
    initial=read(args.initial)
    assert initial['target_commit']==args.commit and initial['differences']==['signature_thumb_ip_two_tail_spool']
    assert len(initial['key_samples'])==23 and all(z['matched'] for z in initial['key_samples'])
    assert sum(z['matched'] for z in initial['geometry_signature_comparisons'])==9
    manifest=read(root/'checkpoints/DRIVE018.manifest.json');frozen={}
    for key in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(read(root/'AUDIT_EVIDENCE_INDEX.json')[key])
    def guard():return all(sha(root/z)==v['sha256'] for z,v in manifest['files'].items()) and all(sha(root/z)==v for z,v in frozen.items())
    assert guard()
    fresh=next(z['shape'] for z in g.build()[0] if z['name']=='thumb_ip_two_tail_spool')
    row=next(z for z in read(root/'cad/motor_spool_coupling_v1/selected/generated/scene.json')['parts'] if z['name']=='thumb_ip_two_tail_spool')
    saved=g.c.load([row])[0]['shape']
    def values(shape,properties):return [*np.asarray(g.m.pack._bbox(shape)).ravel(),*properties.CentreOfMass().Coord()]
    def signature(shape):
        vol=GProp_GProps();ve=BRepGProp.VolumeProperties_s(shape.wrapped,vol,1e-10,False,False)
        area=GProp_GProps();ae=BRepGProp.SurfaceProperties_s(shape.wrapped,area,1e-10,False)
        faces=[]
        for f in shape.Faces():
            prop=GProp_GProps();error=BRepGProp.SurfaceProperties_s(f.wrapped,prop,1e-10,False)
            faces.append(dict(kind=f.geomType(),values=[prop.Mass(),*values(f,prop)],integration_error=error))
        edges=[dict(kind=e.geomType(),values=[e.Length(),*np.asarray(g.m.pack._bbox(e)).ravel(),*e.Center().toTuple()]) for e in shape.Edges()]
        return dict(valid=shape.isValid(),solids=len(shape.Solids()),
            global_values=[vol.Mass(),area.Mass(),*values(shape,vol)],faces=faces,edges=edges,
            vertices=[dict(kind='point',values=list(v.toTuple())) for v in shape.Vertices()],
            integration_errors=dict(volume=ve,area=ae),default_volume=shape.Volume(),default_area=shape.Area())
    a,z=signature(fresh),signature(saved);differences=[];groups=[]
    if not a['valid'] or not z['valid'] or a['solids']!=1 or z['solids']!=1:differences.append('valid/solids')
    delta=np.asarray(a['global_values'])-z['global_values']
    if max(abs(delta))>1e-5:differences.append('global_values')
    for key in ['faces','edges','vertices']:
        matched=[]
        for kind in sorted(set(q['kind'] for q in a[key]+z[key])):
            aa=[q for q in a[key] if q['kind']==kind];zz=[q for q in z[key] if q['kind']==kind]
            if len(aa)!=len(zz):differences.append(key+' '+kind+' count');continue
            cost=np.array([[max(abs(np.asarray(x['values'])-y['values'])) for y in zz] for x in aa])
            ii,jj=linear_sum_assignment(cost)
            for i,j in zip(ii,jj):
                error=float(cost[i,j]);matched.append(dict(kind=kind,source_index=int(i),saved_index=int(j),max_numeric_difference=error,matched=error<=1e-5))
                if error>1e-5:differences.append(key+' '+kind)
        groups.append(dict(group=key,comparisons=matched,max_numeric_difference=max(q['max_numeric_difference'] for q in matched)))
    clean=not git('status','--porcelain=v1')
    result=dict(completed=True,passed=not differences and guard() and clean,target_commit=args.commit,differences=differences,
        initial_core_passed=False,initial_core_sha256=sha(args.initial),initial_core_preserved=True,original_native_probes_matched=23,original_other_shape_signatures_matched=9,
        comparison_absolute_tolerance=1e-5,adaptive_integration_relative_epsilon=1e-10,
        original_default_volume_delta_mm3=a['default_volume']-z['default_volume'],
        adaptive_global_delta=delta.tolist(),adaptive_volume_delta_mm3=float(delta[0]),adaptive_area_delta_mm2=float(delta[1]),
        source_signature=a,saved_signature=z,matched_groups=groups,manifest_files=len(manifest['files']),frozen_files=len(frozen),
        input_hash_guard=guard(),checkout_clean=clean,comparison_code_sha256=sha(__file__),
        method='Original default integration generated a1.343e-5mm3 scalar volume discrepancy at a1e-5 absolute limit.Adaptive volume/surface integration epsilon1e-10 and type-constrained minimum-cost matching of all face/edge/vertex metric signatures,keeping absolute limit1e-5.Original failed core remains false.No clone or CAD/threshold changed.No additional collision,render,physical or full hand drive approval.No proof of every surface/curve equation.')
    g.p.write(args.output,result);print('precision supplement',result['passed'],result['adaptive_volume_delta_mm3'],flush=True)
    assert result['passed']


if __name__=='__main__':main()
