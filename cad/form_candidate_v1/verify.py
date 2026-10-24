"""Check actual STEP round-trip and source/mesh integrity of the form study.

This does not perform collision, assembly, transmission or dynamics testing.
"""
from pathlib import Path
import hashlib,json
import cadquery as cq
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from lightweight_thumb import build_thumb_metacarpal, verify_distal_axes

D=Path(__file__).resolve().parent
G=D/'generated'
source=json.loads((G/'geometry_review.json').read_text(encoding='utf-8'))
scene=json.loads((G/'scene.json').read_text(encoding='utf-8'))
checks={}
design=json.loads((D/'design.json').read_text(encoding='utf-8'))
thumb=design['thumb']
thumb_axes=verify_distal_axes(build_thumb_metacarpal(thumb['links'][0],thumb['second_yaw_deg']),
    thumb['links'][0],thumb['second_yaw_deg'])
checks['thumb_actual_brep_distal_bore_axes_match_child']=thumb_axes['passed']
checks['source_hashes_match']=all(hashlib.sha256((D/name).read_bytes()).hexdigest()==value
    for name,value in source['input_sha256'].items())
checks['all_mesh_hashes_match']=all(hashlib.sha256((D/part['mesh']).read_bytes()).hexdigest()==part['sha256_stl']
    for part in scene['hand']+scene['arm_reference'])
checks['new_parts_single_solids']=all(p['solid_count']==1 and p['valid'] for p in scene['hand'])
step=cq.importers.importStep(str(G/'hand_form_candidate.step')).val()
parts=step.Solids()
checks['step_valid']=step.isValid() and all(s.isValid() for s in parts)
checks['step_solid_count_matches']=len(parts)==len(scene['hand'])
expected=sum(p['volume_mm3'] for p in scene['hand'])
actual=sum(s.Volume() for s in parts)
relative=abs(actual-expected)/expected
checks['step_volume_relative_error_below_1e_6']=relative<1e-6
box=Bnd_Box();BRepBndLib.AddOptimal_s(step.wrapped,box,False,False)
ext=box.Get()
actual_bounds=[list(ext[:3]),list(ext[3:])]
expected_bounds=[[min(p['bbox_mm'][0][i] for p in scene['hand']) for i in range(3)],
                 [max(p['bbox_mm'][1][i] for p in scene['hand']) for i in range(3)]]
bound_error=max(abs(actual_bounds[j][i]-expected_bounds[j][i]) for j in range(2) for i in range(3))
checks['step_bounds_error_below_0_01_mm']=bound_error<0.01
report={'passed':all(checks.values()),'checks':checks,'step_solid_count':len(parts),
        'thumb_distal_bore_axes':thumb_axes,
        'step_volume_relative_error':relative,'step_bounds_max_error_mm':bound_error,
        'step_bounds_mm':actual_bounds,
        'step_sha256':hashlib.sha256((G/'hand_form_candidate.step').read_bytes()).hexdigest(),
        'scope':'Export/import integrity only; no motors fitted and no assembly, clearance or latency approval.'}
(G/'step_roundtrip_review.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report),flush=True)
if not report['passed']:raise SystemExit(1)
