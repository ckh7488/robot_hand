"""Prove unchanged STEP inputs/poses for the three existing port1 conflicts."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm073_continuity',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,ROOT,HERE=m.n,m.np,m.ROOT,m.HERE
old=n.m.drive.module('_arm073_current_meta',ROOT/'cad/ordered_roll_root_assembly_v1/render_evidence.py')
prev=n.m.drive.module('_arm073_ARM059_meta',ROOT/'cad/integrated_motor_end_v1/postprocess.py')


def records():
    raw=n.p.read(ROOT/'cad/integrated_motor_end_v1/reviews/check.json');_,_,metas,_,delta,reloc,clock,shift=prev.metadata(raw);before={z['name']:z for z in metas};actual,matrix=old.metadata();after={z['name']:z for z in actual};moved=set(raw['design']['moved_names']);source={};result=[]
    names=['arm059_integrated_electrical_R5_base','thumb_ip_barrel_adjuster_1','thumb_ip_barrel_top_nut_1','thumb_ip_barrel_bottom_nut_1']
    for name in names:
        M=np.array(delta.get(name,np.eye(4)))
        if name in reloc:M=reloc[name]@M
        if name in prev.a.a.CLOCK_NAMES:M=clock@M
        if name in moved:M=shift@M
        a,b=before[name],after[name];same=a['step']==b['step'] and a['sha256_step']==b['sha256_step']==n.m.drive.sha(ROOT/a['step']);err=float(np.max(abs(M-matrix(name))))
        result.append(dict(name=name,original_ARM059_step=a['step'],current_step=b['step'],sha256_step=a['sha256_step'],same_source_STEP=same,original_ARM059_transform=M,current_transform=matrix(name),transform_max_difference=err,matched=bool(same and err<1e-10)))
        source[a['step']]=a['sha256_step']
    for f in [Path(__file__),HERE/'model.py',ROOT/'cad/integrated_motor_end_v1/reviews/check.json',ROOT/'cad/integrated_motor_end_v1/postprocess.py',ROOT/'cad/ordered_roll_root_assembly_v1/render_evidence.py']:
        source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    return result,source


def main():
    out=HERE/'reviews/conflict_continuity.json';assert not out.exists();got,source=records();assert all(z['matched'] for z in got)
    n.a.save(out,dict(completed=True,records=got,source_sha256=source,all_four_unchanged=all(z['matched'] for z in got),new_native_groups=0,new_common_queries=0,existing_ARM059_final_port1_conflicts_retested=0,
        scope='Same stored STEP source and exactly same registered pose for base/adjuster1/topnut1/bottomnut1 relative to the ARM059 candidate metadata. Read existing three native records only. No old geometry/31native/common/precision replay. Same input continuity does not erase prior sourceSTEP/trim equivalence limits or approve wholefactory/physical.',input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items())))
    print('old port1 four STEP/pose continuities',len(got),'native0/common0',flush=True)


if __name__=='__main__':main()
