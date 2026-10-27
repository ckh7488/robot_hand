"""Read saved failed pose/module membership and current world bounds; no Booleans."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm066_survey',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
from OCP.OSD import OSD_ThreadPool


def main():
    out=m.HERE/'reviews/survey.json';assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    rows,membership,source=m.context();lookup={z['name']:z for z in rows};packets=m.packets(rows)
    q=m.n.p.read(m.ROOT/'cad/shared_roll_root_v1/reviews/check.json')
    result=dict(completed=True,source_sha256=source,packets=packets,final_names=sorted(lookup),
        packet_count=16,packet_rows=sum(len(p['names']) for p in packets),native_groups=0,new_geometry=0,
        inherited_failed_cap_pose=q['assembly'][0],inherited_failed_cap_overlaps=q['native']['assembly_0_0']['overlaps'],
        bounds={k:m.bbox([lookup[k]]) for k in [m.a.NAME,m.a.CAP,'arm063_recessed_front_cap','Forearm_box_beam']},
        packet_body_bounds={p['module']:m.bbox([lookup[k] for k in p['body_names']]) for p in packets},
        remaining_forearm_names=sorted(z['name'] for z in rows if membership.get(z['name'])=='forearm' and z['name'] not in {k for p in packets for k in p['names']}),
        OCC_pool_threads=1,input_hash_guard=all(m.n.m.drive.sha(m.ROOT/f)==h for f,h in source.items()))
    m.n.a.save(out,result);print('survey',len(rows),len(packets),result['packet_rows'],flush=True)


if __name__=='__main__':main()
