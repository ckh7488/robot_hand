"""Exact-SHA bounded replay. Open vendor surfaces never enter a volume Boolean."""
import argparse
import importlib.util
import json
import subprocess
import sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm062_review',Path(__file__).with_name('model.py'))
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
n,np,d,ROOT,HERE=a.n,a.np,a.d,a.ROOT,a.HERE
metrics=n.m.drive.module('_arm062_metrics',ROOT/'cad/long_housing_route_v1/review_core.py')
interp=n.m.drive.module('_arm062_interpret_review',HERE/'interpret_interface.py')
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.OSD import OSD_ThreadPool
from scipy.optimize import linear_sum_assignment


def numeric_match(left,right):
    if isinstance(left,dict):return left.keys()==right.keys() and all(numeric_match(left[k],right[k]) for k in left)
    if isinstance(left,list):return isinstance(right,list) and len(left)==len(right) and all(numeric_match(x,y) for x,y in zip(left,right))
    if isinstance(left,(float,int)) and not isinstance(left,bool):return abs(left-right)<1e-5
    return left==right


def surface_metrics(sh):
    faces=[]
    for f in sh.Faces():
        g=GProp_GProps();BRepGProp.SurfaceProperties_s(f.wrapped,g,1e-10,False)
        faces.append(dict(kind=f.geomType(),values=[g.Mass(),*g.CentreOfMass().Coord()]))
    return dict(valid=sh.isValid(),solids=len(sh.Solids()),shells=len(sh.Shells()),faces=faces)


def surface_match(left,right):
    maximum=0.;bad=[]
    # Invalid/open topology is preserved and reported,never called a solid pass.
    if left['valid']!=right['valid'] or left['solids']!=right['solids']:bad.append('topology_changed')
    for kind in sorted({q['kind'] for q in left['faces']+right['faces']}):
        x=[q['values'] for q in left['faces'] if q['kind']==kind];y=[q['values'] for q in right['faces'] if q['kind']==kind]
        if len(x)!=len(y):bad.append('face_count_'+kind);continue
        cost=np.array([[float(max(abs(np.array(i)-j))) for j in y] for i in x])
        ii,jj=linear_sum_assignment(cost);maximum=max(maximum,float(max(cost[ii,jj])))
    if maximum>1e-5:bad.append('surface_signature_difference')
    return dict(matched=not bad,differences=bad,maximum_absolute_difference=maximum,
        source_valid=left['valid'],STEP_valid=right['valid'],source_solids=left['solids'],STEP_solids=right['solids'],
        source_faces=len(left['faces']),STEP_faces=len(right['faces']),source_shells=left['shells'],STEP_shells=right['shells'],
        volume_or_all_trim_Boolean_equivalence_queried=False)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',required=True,type=Path);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1')
    assert not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
    man=n.p.read(ROOT/'checkpoints/ARM062.manifest.json');frozen={}
    for k in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(n.p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[k])
    def guard():return all(n.m.drive.sha(ROOT/f)==v['sha256'] for f,v in man['files'].items()) and all(n.m.drive.sha(ROOT/f)==v for f,v in frozen.items())
    assert guard() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    raw=n.p.read(HERE/'reviews/check.json');follow=n.p.read(HERE/'reviews/interface_followup.json');ev=n.p.read(HERE/'reviews/evidence.json')
    rows,membership,source=a.context();unit,entry,contract=a.mounting_unit(rows)
    vendor,character=a.vendor_interface();correction,probes=interp.calculate(dict(raw,vendor_interface=character))
    result=dict(completed=False,target_commit=args.commit,differences=[],native_samples=[],geometry_comparisons=[],
        manifest_files=len(man['files']),frozen_files=len(frozen),OCC_pool_threads=1,same_agent_not_independent_audit=True)
    def save():n.a.save(args.output,result)
    def geom(fresh,saved):
        assert len(fresh)==len(saved)
        for x,y in zip(fresh,saved):
            assert x['name']==y['name'];got=dict(name=x['name'],**metrics.match(metrics.metrics(x['shape']),metrics.metrics(y['shape'])))
            result['geometry_comparisons'].append(got)
            if not got['matched']:result['differences'].append(x['name'])
            save()
    saved_unit=n.c.load(n.p.read(ROOT/raw['mounting_reference_scene'])['parts'])
    saved_entry=n.c.load(n.p.read(ROOT/raw['entry_reference_scene'])['parts'])
    saved_probes=n.c.load(n.p.read(ROOT/follow['scene'])['parts'])
    geom(unit,saved_unit);geom(entry,saved_entry);geom(probes,saved_probes)
    def compare(label,got,want):
        l,r=d.overlap_map(got),d.overlap_map(want)
        error=max([abs(l[k]['material_intersection_mm3']-r[k]['material_intersection_mm3']) for k in l.keys()&r.keys()]+[0.])
        ok=l.keys()==r.keys() and error<1e-5 and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope']
        result['native_samples'].append(dict(id=label,native=got,matched=bool(ok),max_volume_difference_mm3=error))
        if not ok:result['differences'].append(label)
        save();print(label,bool(ok),flush=True)
    compare('new_axial_entry4__all_existing871',n.c.material(saved_entry,rows),raw['native']['new_axial_entry4__all_existing871'])
    bounds=n.c.load(n.p.read(ROOT/'cad/rotation_reserved_support_v1/generated/protection/scene.json')['parts'])
    compare('new_axial_entry4__existing_rotation_bounds9',n.c.material(saved_entry,bounds[:9]),raw['native']['new_axial_entry4__existing_rotation_bounds9'])
    lookup={z['name']:z for z in rows}
    compare('output_tap_axis6__existing_flange_beam2',n.c.material(saved_probes,[lookup['Roll_output_flange'],lookup['Forearm_box_beam']]),follow['native'])
    want=ev['common_source_pair'];E=next(z for z in saved_entry if z['name']==want['lead'])
    common=E['shape'].intersect(lookup[want['part']]['shape'],tol=1e-6)
    geom([dict(name='arm062_entry_reference_carrier_common',shape=common)],n.c.load(n.p.read(ROOT/ev['common_scene'])['parts']))
    result['common_query_matched']=abs(common.Volume()-ev['common_volume_mm3'])<1e-5
    meta=n.p.read(ROOT/raw['vendor_reference_scene'])['parts'][0]
    stored=a.cq.importers.importStep(str(ROOT/meta['step'])).val()
    result['open_surface_comparison']=surface_match(surface_metrics(vendor['shape']),surface_metrics(stored))
    if not result['open_surface_comparison']['matched']:result['differences'].append('open_surface_signature')
    plain=lambda v:json.loads(json.dumps(v,default=lambda q:q.tolist() if hasattr(q,'tolist') else q))
    result['unit_contract_matched']=plain(contract)==raw['unit_contract']
    result['initial_vendor_characterization_matched']=numeric_match(plain(character),raw['vendor_interface'])
    result['corrected_interface_contract_matched']=numeric_match(plain(correction),{k:follow[k] for k in correction})
    for k in ['common_query_matched','unit_contract_matched','initial_vendor_characterization_matched','corrected_interface_contract_matched']:
        if not result[k]:result['differences'].append(k)
    result.update(completed=True,native_groups=len(result['native_samples']),native_matches=sum(q['matched'] for q in result['native_samples']),
        geometry_shapes=len(result['geometry_comparisons']),geometry_matches=sum(q['matched'] for q in result['geometry_comparisons']),
        input_hash_guard=guard(),checkout_clean=not git('status','--porcelain=v1'),
        replay_scope='New3native groups:entry4 vs existing871 and existing9bounds;actual-source tap axes6 vs plate/beam2.17solid reference signatures(6unit+4entry+6axes+1common),one common query;one invalid/open615face source-STEP area/centroid signature without volume;initial/corrected source tap/pin and datum contracts. No oldARM06117/3/34/5/distances/shortentry24,alltrim Boolean,wholearm/longmaterials/tools/assembly/strength/torque/electrical/physical/exterior approval. No new installed hardware;vendor registration is a reference assumption,not adoption.')
    result['initial_core_passed']=not result['differences'] and result['input_hash_guard'] and result['checkout_clean']
    save();print('core',result['initial_core_passed'],'diff',result['differences'],flush=True)


if __name__=='__main__':main()
