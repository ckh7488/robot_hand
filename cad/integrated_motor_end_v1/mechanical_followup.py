"""Locate new dogleg/nut and moving-wrist obstructions without31-group replay."""
import importlib.util
import sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm059_follow_post',Path(__file__).with_name('postprocess.py'))
p=importlib.util.module_from_spec(spec)
spec.loader.exec_module(p)
a,n,np,d,ROOT,HERE=p.a,p.n,p.np,p.d,p.ROOT,p.HERE


def calculate(raw):
    _,before,proposed,membership,delta,reloc,clock,shift=p.metadata(raw)
    lookup={z['name']:z for z in before+proposed}
    names=['thumb_ip_two_tail_spool','arm054_eight_bank_carrier',a.CARRIER,a.JAWS[0],
        'compact_saddle_nested_turns','middle_dip_actuator_rack_nut_2']
    source={}
    loaded=n.c.load([lookup[k] for k in names],source)
    sh={}
    for z in loaded:
        name=z['name']
        M=np.array(delta.get(name,np.eye(4)))
        if name in reloc: M=reloc[name]@M
        if name in a.a.CLOCK_NAMES: M=clock@M
        sh[name]=n.g.original.transform(z['shape'],M)
    queries=[]
    rows=[]
    def query(name,kind,left,right):
        got=left.intersect(right,tol=1e-6) if kind=='common' else left.cut(right,tol=1e-6)
        assert got.isValid()
        queries.append(dict(id=name,operation=kind,volume_mm3=got.Volume() if got.Solids() else 0.,solids=len(got.Solids()),
            bbox_mm=n.c.m.pack._bbox(got) if got.Vertices() else None))
        if got.Solids(): rows.append(dict(name=name,shape=got,body='forearm',evidence_only=True,color=[.98,.04,.03]))
        return got
    C=np.array(next(z['transform'] for z in raw['output_command_checks'] if z['command_deg']==-90.))
    spool=n.g.original.transform(sh['thumb_ip_two_tail_spool'],C)
    common=query('arm059_command_minus90_spool_carrier_common','common',spool,sh[a.CARRIER])
    query('arm059_command_minus90_common_outside_old_carrier','cut',common,sh['arm054_eight_bank_carrier'])
    state=next(z for z in raw['arm_motion'] if z['id']=='deviation_-20')
    F=state['transforms']
    jaw=n.g.original.transform(sh[a.JAWS[0]],np.array(F['forearm']))
    guide=n.g.original.transform(sh['compact_saddle_nested_turns'],np.array(F[membership['compact_saddle_nested_turns']]))
    query('arm059_deviation_minus20_jaw_wristguide_common','common',jaw,guide)
    query('arm059_new_carrier_middle_dip_nut_common','common',sh[a.CARRIER],sh['middle_dip_actuator_rack_nut_2'])
    return dict(completed=True,source_sha256=source,native_queries=queries,native_boolean_operations=4,operation_failures=[],
        whole31_groups_reexecuted=0,extra_precision_or_failed_M3_pad_query_repeated=False,
        includes_actual_moving_bodies=True,installation_or_geometry_changed=False),rows


def main():
    out=HERE/'reviews/mechanical_followup.json'
    assert not out.exists()
    raw=n.p.read(HERE/'reviews/check.json')
    result,rows=calculate(raw)
    result['scene']=n.a.a.prior.export(rows,HERE/'generated/mechanical_common')
    for f in [Path(__file__),HERE/'reviews/check.json']:
        result['source_sha256'][f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    result['input_hash_guard']=all(n.m.drive.sha(ROOT/f)==h for f,h in result['source_sha256'].items())
    n.a.save(out,result)
    print([(z['id'],z['volume_mm3'],z['solids']) for z in result['native_queries']],flush=True)


if __name__=='__main__': main()
