"""Finalize completed native paths after a set-union report aggregation error."""
import json,hashlib,ast
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
def read(path):return json.loads(path.read_text(encoding='utf-8-sig'))
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def write(path,value):path.write_bytes((json.dumps(value,ensure_ascii=False,indent=2)+'\n').encode('utf-8'))
def main():
    out=HERE/'reviews/selected_service.json';assert not out.exists();raw=HERE/'reviews/corrected_service.json';r=read(raw);assert not r['completed'] and r['paths'][-1]['id']=='housing_end_0' and all(z['clear'] for z in r['paths']) and r['bench_subassembly']['clear']
    assert all(sha(ROOT/z)==h for z,h in r['source_sha256'].items());source=read(HERE/'reviews/service.json');assert source['stopped_at_failed_path']=='upper_shoe' and not source['sampled_sequence_clear']
    names=list(r['initial_installed'])
    for path in r['paths']:
        assert path['installed_before']==names and path['action'] in ['install','remove']
        assert all(not z['native']['overlaps'] and not z['native']['operation_failures'] for z in path['samples'])
        if path['action']=='install':names.extend(n for n in path['moving'] if n not in names)
        else:names=[n for n in names if n not in path['moving']]
    candidate=read(ROOT/'cad/thumb_sheath_terminal_v2/selected/generated/candidate_scene.json');housing=read(ROOT/candidate['housing_reference'])['parts'];wanted={z['name'] for z in candidate['parts']+candidate['retained_guide_parts']+housing}
    assert set(names)==wanted and len(names)==25
    code=(HERE/'correct_order.py').read_text(encoding='utf-8');assert 'wanted={z[\'name\'] for z in a.new+a.housing}+{z[\'name\']' in code;ast.parse(code)
    r.update(completed=True,sampled_sequence_clear=True,new_samples=sum(len(z['samples']) for z in r['paths']),bench_samples=len(r['bench_subassembly']['samples']),final_installed=names,final_candidate_rows_restored=True,input_hash_guard=True,finalized_without_native_reexecution=True,new_native_samples_in_finalizer=0,finalization_reason='All22native paths finished;final report set+set expression raised TypeError before aggregate completion flags.Addonlycorrect union/counts after validating every recorded stage,transform scope/continuity/hash.No native result or geometry altered.')
    r['source_sha256']=dict(r['source_sha256']);r['source_sha256']['cad/distal_terminal_assembly_v1/reviews/corrected_service.json']=sha(raw);r['source_sha256']['cad/distal_terminal_assembly_v1/finalize.py']=sha(Path(__file__));write(out,r)
    write(HERE/'reviews/selection.json',dict(selected_report='reviews/selected_service.json',history_reports=['reviews/service.json'],corrected_native_report='reviews/corrected_service.json',completed=True,required_sequence_clear=True,raw_first_samples=source['new_samples'],raw_first_completed=False,selected_new_samples=r['new_samples'],bench_samples=r['bench_samples'],reused_native_samples=0,previous_offset024_keeper6_not_reused=True,geometry_changed=False,geometry_catalog='cad/thumb_sheath_terminal_v2/selected/generated/candidate_scene.json',known_inherited_source_STEP_face_comparison_inconclusive=True,whole_hand_drive_complete=False))
    print(json.dumps(dict(completed=True,paths=len(r['paths']),new_hand_samples=r['new_samples'],bench_samples=r['bench_samples'],final_rows=len(names))),flush=True)
if __name__=='__main__':main()
