"""Repair report-only list aliasing, preserving the original native results.

The executed function built its obstacle list at call time; the report retained
the mutable caller list, which later grew. Check every stored pair count against
the exact call sequence before publishing corrected stage metadata.
"""
import sys,copy,json,hashlib
from pathlib import Path
sys.dont_write_bytecode=True
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
def read(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    source=HERE/'selected2/reviews/service.json';out=source.with_name('service_scopes.json');assert not out.exists();old=read(source);assert old['completed'] and old['input_hash_guard']
    assert all(sha(ROOT/z)==h for z,h in old['source_sha256'].items())
    record=copy.deepcopy(old);installed=[];diagnostics=[]
    for row in record['paths']:
        assert len(row['moving'])==1
        expected=804+len(installed)
        counts=[z['native']['pairs_in_scope'] for z in row['samples']]
        assert all(z==expected for z in counts),(row['id'],counts,expected)
        diagnostics.append(dict(id=row['id'],original_reported_preinstalled=list(row['already_installed_new_parts']),corrected_preinstalled=list(installed),pairs_each=expected,all_sample_pair_counts_verified=True))
        row['already_installed_new_parts']=list(installed)
        if row['id']!='initial_upper_keeper_straight_side':installed+=row['moving']
    assert len(installed)==11 and len(record['paths'])==12
    record['source_sha256']['cad/thumb_root_guide_v1/selected2/reviews/service.json']=sha(source)
    record['source_sha256']['cad/thumb_root_guide_v1/correct_service_scopes.py']=sha(Path(__file__))
    record['scope_metadata_correction']=dict(original_report=source.relative_to(ROOT).as_posix(),native_results_changed=False,native_samples_rerun=0,
        reason='Report stored the mutable installed list by reference;later append operations changed earlier report metadata.Obstacle shapes had already been built from the then-current list.Canonical call order and804base+prefix pair counts verify all68stored samples.',paths=diagnostics)
    out.write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');print('Corrected12path stage lists;68native pair counts match;no native result altered.',flush=True)
if __name__=='__main__':main()
