"""Extend only the untested approach before saved clear entry segments."""
import importlib.util,sys,time
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm068_entry',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,d,ROOT,HERE=m.n,m.np,m.d,m.ROOT,m.HERE
from OCP.OSD import OSD_ThreadPool


def main():
    out=HERE/'reviews/entry_extension.json';assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    started=time.monotonic();rows,_,source=m.context();lookup={z['name']:z for z in rows};old=n.p.read(HERE/'reviews/selected_check.json')
    assert old['restored_modules']==['index_mcp_actuator']
    for f in [Path(__file__),HERE/'reviews/selected_check.json']:source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    raw=dict(completed=False,source_sha256=source,paths=[],native={},new_hardware_geometry=0,OCC_pool_threads=1,reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,
        scope='Only new approach before saved clear segments. Rack screws start X+160/Y+40 outside all original stage obstacles in X,side-enter to same original X0/Y40 sample then reuse Y20/10/0. Four-base-nut compound starts X160/140/120/100 before saved X80..0. No old path repetition,no motor movement or material deletion.')
    def save():n.a.save(out,raw)
    blocked=False
    for p in [z for z in old['paths'] if z['stage']=='world']:
        if p['id'].startswith('rack_M3'):
            vectors=[[x,40.,0] for x in [160.,140.,120.,100.,80.,60.,40.,20.]]
            suffix=p['samples'][1:];assert suffix[0]['translation_mm']==[0,40.,0]
        else:
            vectors=[[x,0,0] for x in [160.,140.,120.,100.]];suffix=p['samples']
        rec=dict(id='extend_'+p['id'],moving_names=p['moving_names'],present_names=p['present_names'],absent_names=p['absent_names'],samples=[],completed=False,
            reused_suffix_path_id=p['id'],reused_suffix_samples=suffix,reused_suffix_native_groups=len(suffix),saved_segment_repeated=0)
        assert all(not old['native'][s['id']]['overlaps'] and not old['native'][s['id']]['operation_failures'] for s in suffix)
        raw['paths'].append(rec)
        for i,v in enumerate(vectors):
            key=rec['id']+'_'+str(i);M=m.translate(v);moving=m.moved([lookup[k] for k in p['moving_names']],M)
            q=n.c.material(moving,[lookup[k] for k in p['present_names']]);raw['native'][key]=q;rec['samples'].append(dict(id=key,transform=M,translation_mm=v))
            if i==0:
                rec['initial_X_AABB_separation_from_all_present_mm']=float(m.bbox(moving)[0,0]-m.bbox([lookup[k] for k in p['present_names']])[1,0])
                assert rec['initial_X_AABB_separation_from_all_present_mm']>0
            save();print(key,q['pairs_in_scope'],q['exact_pairs'],len(q['overlaps']),flush=True)
            if q['overlaps'] or q['operation_failures']:
                raw['first_blocked_sample']=key;blocked=True;break
        rec['completed']=not blocked;save()
        if blocked:break
    total=d.k.flat(list(raw['native'].values()))
    raw.update(completed=True,new_native_groups=len(raw['native']),pairs_in_scope=total['pairs_in_scope'],AABB_native_pairs=total['exact_pairs'],overlap_records=len(total['overlaps']),
        operation_failures=len(total['operation_failures']),extended_access_clear=not blocked and len(raw['paths'])==3,
        input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items()),elapsed_s=time.monotonic()-started)
    save();print('extension',raw['new_native_groups'],raw['overlap_records'],raw['extended_access_clear'],flush=True)


if __name__=='__main__':main()
