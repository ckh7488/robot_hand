"""Bench-fit the upper collar clamp before placing its shoe/ferrule kit."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import retention_model as m
e=m.drive.module('_retain026_order_inputs',Path(__file__).resolve().parent/'check.py')
g,c,p,np,cq,ROOT,HERE=e.g,e.c,e.p,e.np,e.cq,e.ROOT,e.HERE
def main():
    out=HERE/'reviews/service_order.json';assert not out.exists();r=p.read(HERE/'selected/reviews/check.json');data,pack,guide,housing=e.inputs(r['design']);new=c.load(p.read(ROOT/r['scene'])['parts']);by={z['name']:z for z in new+guide+housing};tag='distal_housing_clamp_1';port=next(z for z in r['design']['ports'] if z['tag']==tag);axis=np.asarray(port['Q'])[:3,2];kit=[z['name'] for z in new if z['name'].startswith(tag+'_')]+['distal_flanged_ferrule_1_captured','distal_housing_shoe_1','distal_short_housing_1'];assert len(kit)==9
    fixednames=kit[-3:];benchfixed=[dict(by[n],shape=by[n]['shape'].translate((500,0,0))) for n in fixednames];benchinstalled=[];paths=[];record=dict(completed=False,source_sha256=data[0],bench_translation_world_mm=[500,0,0],bench_paths=paths,hand_paths=[],kit_names=kit,hand_angles_deg=[0.]*20,long_housing_installed=False,full_assembly_approved=False,physical_fixture_tools_strength_verified=False)
    for name,direction,ds in [(tag+'_jaw_-1',-axis,[14,7,2,0]),(tag+'_jaw_1',axis,[14,7,2,0])]+[(tag+'_nut_'+str(j),-axis,[10,5,0]) for j in [0,1]]+[(tag+'_bolt_'+str(j),axis,[10,5,0]) for j in [0,1]]:
        samples=[]
        for d in ds:
            delta=np.array([500.,0,0])+direction*d;z=by[name];samples.append(dict(translation_world_mm=delta.tolist(),native=c.material([dict(z,shape=z['shape'].translate(tuple(delta)))],benchfixed+[dict(by[n],shape=by[n]['shape'].translate((500,0,0))) for n in benchinstalled])))
        got=dict(moving=[name],installed_before=list(benchinstalled),fixed_bench=fixednames,samples=samples,clear=all(m.drive.clear(s['native']) for s in samples));paths.append(got);p.write(out,record);print('bench',name,got['clear'],flush=True)
        if not got['clear']:record.update(completed=True,bench_clear=False,hand_not_executed=True);p.write(out,record);return
        benchinstalled.append(name)
    removed=['distal_housing_keeper_1','distal_housing_shoe_1','liner_keeper_M2x10_1','liner_keeper_M2_nut_1','distal_shoe_M2x10_1_0','distal_shoe_M2x10_1_1','distal_shoe_M2_nut_1_0','distal_shoe_M2_nut_1_1']
    kept=[z for z in guide if z['name'] not in removed];hand,_,_,_=m.s.k.e.prior.hand_rows(data,kept,np.zeros(20));base=hand+pack+data[6]+data[7]+[z for z in new+housing if z['name'] not in kit];installed=[]
    Q=g.original.layout()[1]['socket_transform'];wx=g.original.WORLD[:3,:3]@Q[:3,0];wz=g.original.WORLD[:3,:3]@Q[:3,2]
    sequence=[('upper_assembled_kit',kit,wx,[20,10,5,1,0]),('upper_keeper_after_kit',['distal_housing_keeper_1'],wz,[14,7,3,1,.5,0]),('original_nut',['liner_keeper_M2_nut_1'],wx,[-12,-6,0]),('original_bolt',['liner_keeper_M2x10_1'],wz,[8,4,0])]+[('new_nut_'+str(j),['distal_shoe_M2_nut_1_'+str(j)],wx,[12,6,0]) for j in [0,1]]+[('new_bolt_'+str(j),['distal_shoe_M2x10_1_'+str(j)],wz,[8,4,0]) for j in [0,1]]
    for label,names,direction,ds in sequence:
        samples=[];ob=base+[by[n] for n in installed]
        for d in ds:
            delta=direction*d;samples.append(dict(translation_world_mm=delta.tolist(),native=c.material([dict(by[n],shape=by[n]['shape'].translate(tuple(delta))) for n in names],ob)))
        got=dict(id=label,moving=names,installed_before=list(installed),omitted_guide=removed,base_extra_names=[z['name'] for z in new+housing if z['name'] not in kit],samples=samples,clear=all(m.drive.clear(z['native']) for z in samples));record['hand_paths'].append(got);p.write(out,record);print(label,got['clear'],flush=True)
        if not got['clear']:break
        installed.extend(names)
    record.update(completed=True,bench_clear=True,bench_samples=sum(len(z['samples']) for z in paths),hand_samples=sum(len(z['samples']) for z in record['hand_paths']),upper_required_sequence_clear=len(record['hand_paths'])==len(sequence) and all(z['clear'] for z in record['hand_paths']),input_hash_guard=all(m.drive.sha(ROOT/z)==h for z,h in data[0].items()),scope='Bench9partkit includes24mmhousing reference;manualholding before bolt engagement;upper replacement ferrule installed by kit.Lower replacement collar/remaining2clamps,new wholehousing/cordthreading and tools not verified.Original failed postinstallation preserved.')
    p.write(out,record)
if __name__=='__main__':main()
