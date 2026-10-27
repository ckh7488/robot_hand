"""Locate the remaining +90 output obstruction without replaying completed groups."""
import importlib.util
import sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm060_follow_model',Path(__file__).with_name('model.py'))
a=importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)
n,np,d,ROOT,HERE=a.n,a.np,a.d,a.ROOT,a.HERE


def calculate(raw):
    prior=n.p.read(ROOT/'cad/integrated_motor_end_v1/reviews/check.json')
    _,before,_,_,delta,reloc,clock,_=a.p.metadata(prior)
    saved=n.p.read(ROOT/raw['scene'])['parts']
    lookup={z['name']:z for z in before+saved}
    names=['thumb_ip_two_tail_spool','arm054_eight_bank_carrier',a.CARRIER]
    source={};shapes={}
    for z in n.c.load([lookup[name] for name in names],source):
        name=z['name'];M=np.array(delta.get(name,np.eye(4)))
        if name in reloc: M=reloc[name]@M
        if name in a.a.a.CLOCK_NAMES: M=clock@M
        shapes[name]=n.g.original.transform(z['shape'],M)
    C=np.array(next(z['transform'] for z in raw['output_commands'] if z['command_deg']==90.))
    spool=n.g.original.transform(shapes['thumb_ip_two_tail_spool'],C)
    queries=[]
    def query(label,kind,left,right):
        sh=left.intersect(right,tol=1e-6) if kind=='common' else left.cut(right,tol=1e-6)
        assert sh.isValid()
        queries.append(dict(id=label,operation=kind,volume_mm3=sh.Volume() if sh.Solids() else 0.,solids=len(sh.Solids()),
            bbox_mm=n.c.m.pack._bbox(sh) if sh.Vertices() else None))
        return sh
    common=query('arm060_plus90_spool_carrier_common','common',spool,shapes[a.CARRIER])
    query('arm060_plus90_common_outside_original_ARM054_carrier','cut',common,shapes['arm054_eight_bank_carrier'])
    bridges=a.a.a.box([34,-162,9],[38,-118,17]).fuse(a.a.a.box([34,-162,-17],[38,-118,-9]))
    query('arm060_plus90_common_outside_original_thumb_bridge_regions','cut',common,bridges)
    rows=[dict(name=queries[0]['id'],shape=common,body='forearm',evidence_only=True,color=[.98,.04,.03])]
    return dict(completed=True,source_sha256=source,native_queries=queries,native_boolean_operations=3,
        original_thumb_bridge_regions_mm=[[[34,-162,9],[38,-118,17]],[[34,-162,-17],[38,-118,-9]]],
        previous_groups_reexecuted=0,old_failed_M3_pad_or_small_seam_retested=False),rows


def main():
    out=HERE/'reviews/followup.json'
    assert not out.exists()
    raw=n.p.read(HERE/'reviews/check.json')
    result,rows=calculate(raw)
    result['scene']=n.a.a.prior.export(rows,HERE/'generated/actual_common')
    for f in [Path(__file__),HERE/'reviews/check.json']:
        result['source_sha256'][f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    result['input_hash_guard']=all(n.m.drive.sha(ROOT/f)==h for f,h in result['source_sha256'].items())
    n.a.save(out,result)
    print([(z['id'],z['volume_mm3']) for z in result['native_queries']],flush=True)


if __name__=='__main__': main()
