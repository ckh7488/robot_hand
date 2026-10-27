"""Distinguish retained clamp space from the new web/fastener obstruction."""
import importlib.util
import sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm061_follow',Path(__file__).with_name('model.py'))
a=importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)
n,np,ROOT,HERE=a.n,a.np,a.ROOT,a.HERE


def calculate(raw):
    prior=n.p.read(ROOT/'cad/integrated_motor_end_v1/reviews/check.json')
    _,_,base,_,delta,reloc,clock,shift=a.p.p.metadata(prior)
    oldparts=n.p.read(ROOT/'cad/rotation_reserved_support_v1/generated/hardware/scene.json')['parts']
    fresh=n.p.read(ROOT/raw['scene'])['parts']
    protection=n.p.read(ROOT/raw['protection_input_scene'])['parts']
    lookup={z['name']:z for z in base+oldparts+fresh+protection}
    names=[a.OLD,a.NAME,'arm060_rotation_bound_thumb_ip_two_tail_spool','arm054_M3x12_envelope_1_0']
    source={};shapes={}
    for z in n.c.load([lookup[name] for name in names],source):
        name=z['name'];M=np.array(delta.get(name,np.eye(4)))
        if name in reloc: M=reloc[name]@M
        if name in a.p.a.a.CLOCK_NAMES: M=clock@M
        if name in prior['design']['moved_names']: M=shift@M
        shapes[name]=n.g.original.transform(z['shape'],M)
    geom=n.m.drive.module('_arm061_follow_collar',ROOT/'cad/eight_bank_layout_v1/model.py')
    collar=geom.prior.prior.collar(-162.5,False)
    queries=[]
    def query(label,kind,left,right):
        sh=left.intersect(right,tol=1e-6) if kind=='common' else left.cut(right,tol=1e-6)
        assert sh.isValid()
        queries.append(dict(id=label,operation=kind,volume_mm3=sh.Volume() if sh.Solids() else 0.,solids=len(sh.Solids()),
            bbox_mm=n.c.m.pack._bbox(sh) if sh.Vertices() else None))
        return sh
    common=query('arm061_spool_bound_carrier_common','common',shapes[names[2]],shapes[a.NAME])
    query('arm061_bound_common_outside_ARM060_carrier','cut',common,shapes[a.OLD])
    query('arm061_bound_common_outside_original_front_lower_collar','cut',common,collar)
    bolt=query('arm061_front_M3_bolt_carrier_common','common',shapes[names[3]],shapes[a.NAME])
    query('arm061_bolt_common_outside_ARM060_carrier','cut',bolt,shapes[a.OLD])
    evidence=[dict(name=queries[i]['id'],shape=sh,body='forearm',evidence_only=True,color=[.98,.04,.03])
        for i,sh in [(0,common),(3,bolt)]]
    return dict(completed=True,source_sha256=source,native_queries=queries,native_boolean_operations=5,
        previous17_groups_or_output_commands_reexecuted=0,
        conservative_common_is_not_actual_output_common=True,old_failed_M3_pad_or_seams_retested=False),evidence


def main():
    out=HERE/'reviews/followup.json';assert not out.exists()
    raw=n.p.read(HERE/'reviews/check.json')
    result,evidence=calculate(raw)
    result['scene']=n.a.a.prior.export(evidence,HERE/'generated/common_evidence')
    for f in [Path(__file__),HERE/'reviews/check.json']:
        result['source_sha256'][f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    result['input_hash_guard']=all(n.m.drive.sha(ROOT/f)==h for f,h in result['source_sha256'].items())
    n.a.save(out,result)
    print([(z['id'],z['volume_mm3'],z['solids']) for z in result['native_queries']],flush=True)


if __name__=='__main__': main()
