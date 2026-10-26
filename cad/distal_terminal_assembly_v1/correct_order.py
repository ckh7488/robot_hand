"""Preassemble the upper inner shoe/ferrule, place laterally, then fit keeper."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from service import *
def bench(a):
    Q=g.original.layout()[1]['socket_transform'];axis=Q[:3,2];offset=translation([0,0,-500]);Wb=g.original.WORLD@offset@g.original.INV;shoe=a.by['distal_housing_shoe_1'];ferrule=a.by['distal_flanged_ferrule_1'];ob=[dict(name=shoe['name'],shape=g.original.transform(shoe['shape'],Wb))];samples=[]
    for d in [8.,4.,1.,0.]:
        T=offset@translation(axis*d);W=g.original.WORLD@T@g.original.INV;samples.append(dict(transform_local=T.tolist(),native=c.material([dict(name=ferrule['name'],shape=g.original.transform(ferrule['shape'],W))],ob)))
    a.record['bench_subassembly']=dict(moving=[ferrule['name']],fixed=[shoe['name']],bench_translation_neutral_IP_mm=[0,0,-500],samples=samples,clear=all(m.drive.clear(z['native']) for z in samples),scope='Separate bench500mm away from hand reference,only shoe/ferrule.No hand/motor at that bench;manual support of loose two-piece kit,fixture/tool/strength not qualified.');a.save();return a.record['bench_subassembly']['clear']
def sequence_corrected(a):
    Q1=g.original.layout()[1]['socket_transform'];Q0=g.original.layout()[0]['socket_transform'];x1,z1=Q1[:3,0],Q1[:3,2];x0,u0,z0=Q0[:3,0],Q0[:3,1],-Q0[:3,2]
    def straight(label,n,axis,ds,action='install',scope=''):return a.path(label,[n],[translation(axis*d) for d in ds],action,scope)
    if not bench(a):return
    if not a.path('upper_kit_lateral',['distal_housing_shoe_1','distal_flanged_ferrule_1'],[translation(x1*d) for d in [20,12,6,2,0]],scope='Bench-positioned two-piece kit enters from outboard positiveX BEFORE new upper keeper.Shoe/ferrule manually maintained until keeper and bolts secure them;full hand/child/pin present.'):return
    if not straight('upper_keeper_after_kit','distal_housing_keeper_1',z1,[14,7,3,1,.5,0],scope='Actual additional shoe/ferrule obstacles present,so prior six-sample cap entry is not reused.'):return
    if not straight('upper_original_nut','liner_keeper_M2_nut_1',x1,[-12,-6,0],scope='Restore original nut with kit/new keeper present;manual holding before bolt.'):return
    if not straight('upper_original_bolt','liner_keeper_M2x10_1',z1,[8,4,0],scope='Nominal bolt envelope,not screw thread/driver qualification.'):return
    for j in [0,1]:
        if not straight('upper_shoe_nut_'+str(j),'distal_shoe_M2_nut_1_'+str(j),x1,[12,6,0],scope='Inner lowerX nut first from positiveX;both nuts before new bolts.'):return
    for j in [0,1]:
        if not straight('upper_shoe_bolt_'+str(j),'distal_shoe_M2x10_1_'+str(j),z1,[8,4,0],scope='Nominal fastener approaches outward radial side.'):return
    if not straight('remove_lower_original_bolt','liner_keeper_M2x10_0',z0,[0,4,8],'remove','Upper assembly present;original lower cover still installed.'):return
    if not straight('remove_lower_original_nut','liner_keeper_M2_nut_0',x0,[0,-6,-12],'remove','Original lower bolt already removed.'):return
    if not straight('remove_lower_original_keeper','liner_flange_keeper_0',z0,[0,7,14],'remove','Original formed liner remains manually supported in carrier;child/pin present.'):return
    if not straight('lower_bare_ferrule','distal_flanged_ferrule_0',u0,[-25,-15,-5,-1,0],scope='Bare ferrule axial over thin liner before lower half-seats;no housing/cord.'):return
    Ts=[translation(x0*d-z0*3) for d in [-20,-10,-5,0]]+[translation(z0*d) for d in [-2,-1,-.5,0]]
    if not a.path('lower_shoe_side_then_radial',['distal_housing_shoe_0'],Ts,scope='Lateral start outside hand at3mm inward offset,then radial seating.Ferrule/liner manually supported.No physical fixture validation.'):return
    if not straight('lower_new_keeper','distal_housing_keeper_0',z0,[14,7,3,1,0],scope='Close new outer keeper on lower shoe/ferrule;full child and pin present.'):return
    if not straight('lower_original_nut','liner_keeper_M2_nut_0',x0,[-12,-6,0],scope='Restore original mounting nut before new shoe fasteners.'):return
    if not straight('lower_original_bolt','liner_keeper_M2x10_0',z0,[8,4,0],scope='Nominal original M2 envelope only.'):return
    for j in [1,0]:
        if not straight('lower_shoe_nut_'+str(j),'distal_shoe_M2_nut_0_'+str(j),x0,[-12,-6,0],scope='Inner higherX nut first from negativeX;both nuts before new bolts.'):return
    for j in [1,0]:
        if not straight('lower_shoe_bolt_'+str(j),'distal_shoe_M2x10_0_'+str(j),z0,[8,4,0],scope='Nominal fastener approach;real driver/thread/torque unverified.'):return
    for i,Q in [(1,Q1),(0,Q0)]:
        if not straight('housing_end_'+str(i),'distal_short_housing_'+str(i),Q[:3,1],[-30,-15,-5,-1,0],scope='24mm straight end reference only;not full long housing insertion,retention,flexible motion or cord threading.'):return
    wanted={z['name'] for z in a.new+a.housing}+{z['name'] for z in a.original if z['name'] not in ['liner_flange_keeper_0','liner_flange_keeper_1']}
    a.record.update(completed=True,sampled_sequence_clear=True,new_samples=sum(len(z['samples']) for z in a.record['paths']),bench_samples=len(a.record['bench_subassembly']['samples']),final_installed=list(a.installed),final_candidate_rows_restored=set(a.installed)==wanted,input_hash_guard=all(m.drive.sha(ROOT/z)==h for z,h in a.data[0].items()));a.save()
def main():
    out=HERE/'reviews/corrected_service.json';assert not out.exists();start=time.monotonic();a=Assembly(*setup(),out);a.installed.remove('distal_housing_keeper_1');a.record['initial_installed']=list(a.installed);a.record['starting_state_reuse']=dict(reused=False,reason='Changed order begins before upper keeper and preassembles real kit;prior keeper6samples not reused with additional kit obstacles.');a.record['source_sha256']['cad/distal_terminal_assembly_v1/correct_order.py']=m.drive.sha(Path(__file__));sequence_corrected(a);a.record['elapsed_s']=time.monotonic()-start;a.save()
if __name__=='__main__':main()
