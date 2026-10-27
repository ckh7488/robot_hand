"""Recover only lost non-native comparisons; inherit six observed native stdout verdicts."""
import sys,json,re,hashlib
from pathlib import Path
sys.dont_write_bytecode=True
clone=Path('D:/Codex-work/robot-hand-workspace/reviews/CONTACT036_d03af1f60143')
base=Path('D:/Codex-work/robot-hand-workspace')
source=clone/'cad/contact_housing_link_v1/review_core.py'
native_log=base/'logs/contact036-review.out';error_log=base/'logs/contact036-review.err'
def sha(z):return hashlib.sha256(z.read_bytes()).hexdigest()
original_sha=sha(source);original=source.read_text(encoding='utf-8')
assert 'Object of type bool is not JSON serializable' in error_log.read_text(encoding='utf-8')
verdicts=re.findall(r'^(hand|packaging|electrical|arm|wrist_guide|internal) (True|False)$',native_log.read_text(encoding='utf-8'),re.M)
assert [z[0] for z in verdicts]==['hand','packaging','electrical','arm','wrist_guide','internal'] and all(z[1]=='True' for z in verdicts)
remove=["    data,pack,guide,wrist=s.prior.inputs();hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,np.zeros(20));hand+=c.load([z for z in p.read(ROOT/'cad/thumb_entry_bend_v1/generated/parts/scene.json')['parts'] if z['installed_part_candidate']],data[0]);assert len(hand)==397\n","    for k,ob in [('hand',hand),('packaging',pack),('electrical',data[6]),('arm',data[7]),('wrist_guide',wrist)]:compare(k,c.material(saved,ob),raw['native'][k])\n","    compare('internal',combined([c.material([z],saved[j+1:]) for j,z in enumerate(saved)]),combined(raw['internal']))\n"]
adapted=original
for line in remove:
    assert adapted.count(line)==1
    adapted=adapted.replace(line,"    native.extend(_native_log_evidence())\n" if line==remove[0] else '')
assert 'c.material(' not in adapted
def evidence():return [dict(id=k,matched=v=='True',evidence='initial exact-SHA native execution stdout verdict; raw result lost on JSON serialization',stdout_sha256=sha(native_log),native_result_recomputed=False) for k,v in verdicts]
ns=dict(__name__='_contact036_serialization_recovery',__file__=str(source),_native_log_evidence=evidence)
exec(compile(adapted,str(source),'exec'),ns)
oldwrite=ns['p'].write
def normalize(z):
    if isinstance(z,dict):return {k:normalize(v) for k,v in z.items()}
    if isinstance(z,(tuple,list)):return [normalize(v) for v in z]
    if isinstance(z,ns['np'].generic):return z.item()
    return z
def write(path,value):
    value=normalize(value)
    value['serialization_recovery']=dict(original_review_source_sha256=original_sha,unchanged_target_clone=True,initial_review_execution_succeeded=False,initial_native_verdicts_inherited_from_stdout=True,initial_native_raw_numeric_reports_not_saved=True,native_recomputations=0,non_native_geometry_signature_comparisons_recomputed=20,original_stdout_sha256=sha(native_log),original_error_sha256=sha(error_log),adapter_source_sha256=sha(Path(__file__)),scope='Original6native comparisons executed and printed True before NumPy bool JSON serialization failed. Only20geometry/4anchor+curvature+length/2residual comparisons re-executed. Native JSON raw numbers unavailable, not copied from original input. All initial code/error/stdout preserved. No solver/field/native/render rerun.')
    assert sha(source)==original_sha
    oldwrite(path,value)
ns['p'].write=write
ns['main']()
