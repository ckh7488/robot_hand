"""Static scope inherited explicitly; changed guide hardware is recalculated."""
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import radial_model as model
g,check,HERE,ROOT,p,np=model.g,model.check,model.HERE,model.ROOT,model.p,model.np
from OCP.OSD import OSD_ThreadPool
def main():
    out=HERE/'reviews/static.json';assert not out.exists();start=time.monotonic();parts,design=model.build()
    source_module=g.g.drive.module('_guide021_static',ROOT/'cad/thumb_root_guide_v1/build_check.py')
    source_module.g=type('Current',(),dict(cord_pieces=g.cord_pieces,g=g.g))
    data=check.inputs(design);data[0]['cad/thumb_guide_assembly_v2/check_radial.py']=g.g.drive.sha(Path(__file__))
    record=dict(completed=False,design=design,source_sha256=data[0],cases=[],new_pairs=[],OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,whole_hand_drive_complete=False)
    def save():p.write(out,record)
    record['scene']=check.export(HERE/'generated/parts',parts,dict(design=design,hand_geometry_unchanged=True,reference_hand='cad/motion_clearance_v2/generated/assembly_catalog.json',reference_packaging='cad/motor_spool_coupling_v1/selected/generated/scene.json'))
    for i,z in enumerate(parts):record['new_pairs'].append(g.c.material([z],parts[i+1:]))
    old=p.read(g.HERE/'reviews/check.json')
    for case in old['cases']:
        result,_,_=source_module.evaluate(case['angles_deg'],parts,design,data);result['id']=case['id'];record['cases'].append(result);save()
        print(case['id'],result['nominal_geometry_clear'],{k:len(v['overlaps']) for k,v in result['native'].items()},flush=True)
    record.update(completed=True,all_nominal_snapshots_clear=all(z['nominal_geometry_clear'] for z in record['cases']) and all(g.g.drive.clear(z) for z in record['new_pairs']),input_hash_guard=all(g.g.drive.sha(ROOT/z)==h for z,h in data[0].items()),elapsed_s=time.monotonic()-start)
    save();assert record['input_hash_guard']
if __name__=='__main__':main()
