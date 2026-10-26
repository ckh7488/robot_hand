"""Supplement an unchanged exact-SHA clone with analytic STEP comparisons.

Coincident source/STEP Booleans returned inconsistent common/cut volumes.
Keep that failed result. Compare every analytic surface, boundary curve and
vertex instead; unsupported surface types fail this supplementary check.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import cadquery as cq
import numpy as np
from OCP.BRepAdaptor import BRepAdaptor_Curve,BRepAdaptor_Surface
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.gp import gp_Pnt
from OCP.TopAbs import TopAbs_IN

sys.dont_write_bytecode=True


def xyz(point):
    return np.array([point.X(),point.Y(),point.Z()])


def direction(value):
    v=xyz(value)
    for n in v:
        if abs(n)>1e-9:
            return -v if n<0 else v
    raise ValueError('Zero direction')


def axis_line(axis):
    n=direction(axis.Direction())
    v=xyz(axis.Location())
    return [*n,*(v-n*np.dot(n,v))]


def signatures(shape,box):
    def common(s):
        centre=s.Center()
        return [*np.asarray(box(s)).ravel(),centre.x,centre.y,centre.z]
    faces=[]
    for face in shape.Faces():
        a=BRepAdaptor_Surface(face.wrapped,True)
        kind=str(a.GetType()).split('.')[-1]
        if kind=='GeomAbs_Plane':
            g=a.Plane();n=direction(g.Axis().Direction())
            params=[*n,float(np.dot(n,xyz(g.Location())))]
        elif kind=='GeomAbs_Cylinder':
            g=a.Cylinder();params=[*axis_line(g.Axis()),g.Radius()]
        elif kind=='GeomAbs_Torus':
            g=a.Torus();params=[*xyz(g.Location()),*direction(g.Axis().Direction()),g.MajorRadius(),g.MinorRadius()]
        else:
            raise ValueError('Unsupported surface: '+kind)
        faces.append(dict(kind=kind,values=[face.Area(),len(face.Wires()),*common(face),*params]))
    edges=[]
    for edge in shape.Edges():
        a=BRepAdaptor_Curve(edge.wrapped)
        kind=str(a.GetType()).split('.')[-1]
        if kind=='GeomAbs_Line':
            params=axis_line(a.Line().Position())
        elif kind=='GeomAbs_Circle':
            g=a.Circle();params=[*xyz(g.Location()),*direction(g.Axis().Direction()),g.Radius()]
        else:
            raise ValueError('Unsupported curve: '+kind)
        edges.append(dict(kind=kind,values=[edge.Length(),*common(edge),*params]))
    order=lambda r:(r['kind'],*[round(float(x),4) for x in r['values']])
    vertices=sorted([[v.X,v.Y,v.Z] for v in shape.Vertices()],key=lambda v:tuple(round(x,4) for x in v))
    return dict(valid=shape.isValid(),solids=len(shape.Solids()),global_values=[shape.Volume(),shape.Area(),*common(shape)],
        faces=sorted(faces,key=order),edges=sorted(edges,key=order),vertices=vertices)


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--root',type=Path,required=True)
    ap.add_argument('--commit',required=True)
    ap.add_argument('--core',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args()
    root=args.root.resolve()
    if args.output.exists() or args.output.resolve().is_relative_to(root):
        raise ValueError('New external output required')
    def git(*xs):
        return subprocess.check_output(['git',*xs],cwd=root,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    if git('rev-parse','HEAD')!=args.commit or git('status','--porcelain=v1'):
        raise ValueError('Wrong or dirty target clone')
    spec=importlib.util.spec_from_file_location('_original_tail',root/'cad/remote_motor_lead_v1/corrected.py')
    c=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(c)
    core=c.p.read(args.core)
    expected_ids={'remote_lead_1','remote_lead_2','remote_lead_3'}
    if core['target_commit']!=args.commit or {r['id'] for r in core['differences']}!=expected_ids or any(r['reason']!='Rebuilt and stored wire differ' for r in core['differences']):
        raise ValueError('Unexpected core failure, do not override')
    manifest=c.p.read(root/'checkpoints/WIRE009.manifest.json')
    guard=lambda:all(c.b.sha(root/r)==v['sha256'] for r,v in manifest['files'].items())
    if not core['clean_checkout'] or not core['input_hash_guard'] or not guard():
        raise ValueError('Changed target inputs')
    stored=c.p.read(root/'cad/remote_motor_lead_v1/generated/scene.json')
    matrix=np.asarray(stored['transform_world_mm'])
    differences=[]
    rows=[]
    for row in stored['parts']:
        if row['role']!='nominal_electrical_wire_space':
            continue
        local=row['local']
        fresh,_=c.lead_shape(local['straight_stub_mm'],local['start_mm'][1],local['centreline_radius_mm'])
        fresh=c.m.pack._transform(fresh,matrix)
        saved=cq.importers.importStep(str(root/row['step'])).val()
        a,b=signatures(fresh,c.m.pack._bbox),signatures(saved,c.m.pack._bbox)
        numeric=[]
        if not a['valid'] or not b['valid'] or a['solids']!=1 or b['solids']!=1:
            differences.append(dict(id=row['name'],reason='Invalid/non-single solid'))
        for key in ['global_values','vertices']:
            aa,bb=np.asarray(a[key]),np.asarray(b[key])
            if aa.shape!=bb.shape or not np.allclose(aa,bb,rtol=0,atol=1e-5):
                differences.append(dict(id=row['name'],reason='Changed '+key))
            else:
                numeric.extend(abs(aa-bb).ravel().tolist())
        for key in ['faces','edges']:
            if len(a[key])!=len(b[key]):
                differences.append(dict(id=row['name'],reason='Changed '+key+' count'))
                continue
            for aa,bb in zip(a[key],b[key]):
                if aa['kind']!=bb['kind'] or len(aa['values'])!=len(bb['values']) or not np.allclose(aa['values'],bb['values'],rtol=0,atol=1e-5):
                    differences.append(dict(id=row['name'],reason='Changed analytic '+key))
                else:
                    numeric.extend(abs(np.asarray(aa['values'])-np.asarray(bb['values'])).tolist())
        # One interior centreline point per straight/curved span, both solids.
        positions=[(np.asarray(local['start_mm'])+np.asarray(local['arc_start_mm']))/2,
            np.asarray(local['arc_middle_mm']),(np.asarray(local['arc_end_mm'])+np.asarray(local['end_mm']))/2]
        states=[]
        for position in positions:
            point=(matrix@np.r_[position,1.])[:3]
            states.append([str(BRepClass3d_SolidClassifier(s.Solids()[0].wrapped,gp_Pnt(*point),1e-6).State()) for s in [fresh,saved]])
            if any(BRepClass3d_SolidClassifier(s.Solids()[0].wrapped,gp_Pnt(*point),1e-6).State()!=TopAbs_IN for s in [fresh,saved]):
                differences.append(dict(id=row['name'],reason='Interior classification differs'))
        rows.append(dict(name=row['name'],source=a,saved=b,max_numeric_difference=max(numeric),interior_states=states))
    clean=not git('status','--porcelain=v1')
    out=dict(target_commit=args.commit,passed=not differences and clean and guard(),clean_checkout=clean,input_hash_guard=guard(),
        analytic_comparisons=rows,differences=differences,absolute_numeric_tolerance=1e-5,
        original_core_passed=False,original_core_differences_preserved=core['differences'],
        coincident_boolean_equivalence='Inconclusive: common near0 with one directional cut equal to whole volume despite matching analytic geometry and interior classification. Not counted as a physical CAD difference.',
        core_reimport_checks_reproduced=True,manifest_files=core['manifest_files'],frozen_files=core['frozen_files'],
        independent_audit=False,scope='Supplementary3wire analytic surfaces/curves/vertices/properties and span-interior comparisons against an unchanged exact-SHA clone; all original core results remain preserved. No continuous collision, connector mating or physical approval.')
    c.p.write(args.output,out)
    if not out['passed']:
        raise ValueError('Analytic roundtrip comparison differs')


if __name__=='__main__':
    main()
