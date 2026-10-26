"""Finite assembly paths against saved OFFSET024 solids, without geometry edits."""
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'cad/thumb_sheath_terminal_v2'))
import selected_check as k
m=k.m;g,c,p,np,cq=m.g,m.c,m.p,m.np,m.cq
from OCP.OSD import OSD_ThreadPool
def translation(v):
    T=np.eye(4);T[:3,3]=v;return T
def setup():
    static=p.read(ROOT/'cad/thumb_sheath_terminal_v2/selected/reviews/static.json');design=static['design'];data,pack=k.inputs(design);source=data[0]
    original=c.load(p.read(ROOT/'cad/thumb_guide_assembly_v2/generated/parts/scene.json')['parts'],source);new=c.load(p.read(ROOT/static['scene'])['parts'],source);housing=c.load(p.read(ROOT/static['housing_scene'])['parts'],source)
    hand,_,_,_=k.e.prior.hand_rows(data,[],np.zeros(20));source['cad/distal_terminal_assembly_v1/service.py']=m.drive.sha(Path(__file__))
    source['cad/thumb_sheath_terminal_v2/selected/reviews/static.json']=m.drive.sha(ROOT/'cad/thumb_sheath_terminal_v2/selected/reviews/static.json')
    return data,pack,hand,original,new,housing
class Assembly:
    def __init__(self,data,pack,hand,original,new,housing,out):
        self.data,self.pack,self.hand,self.original,self.new,self.housing,self.out=data,pack,hand,original,new,housing,out
        self.by={z['name']:z for z in original+new+housing};removed=['liner_flange_keeper_1','liner_keeper_M2x10_1','liner_keeper_M2_nut_1'];self.installed=[z['name'] for z in original if z['name'] not in removed]+['distal_housing_keeper_1']
        entry=p.read(ROOT/'cad/thumb_sheath_terminal_v2/selected/reviews/keeper_entry.json');assert entry['sampled_upper_keeper_entry_clear'] and all(m.drive.sha(ROOT/z)==h for z,h in entry['source_sha256'].items())
        assert set(entry['installed_original_guide'])==set(self.installed)-{'distal_housing_keeper_1'}
        assert entry['samples'][0]['native']['pairs_in_scope']==len(hand+pack+data[6]+data[7])+len(self.installed)-1
        self.record=dict(completed=False,source_sha256=data[0],OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,paths=[],hand_angles_deg=[0.]*20,missing_hand_parts=[],initial_installed=list(self.installed),starting_state_reuse=dict(report='cad/thumb_sheath_terminal_v2/selected/reviews/keeper_entry.json',sha256=m.drive.sha(ROOT/'cad/thumb_sheath_terminal_v2/selected/reviews/keeper_entry.json'),previous_samples=6,included_in_new_sample_total=False,geometry_and_input_hashes_verified=True),physical_jig_tools_threads_strength_verified=False,cord_threading_tested=False,long_housing_installed=False,whole_hand_drive_complete=False)
    def save(self):p.write(self.out,self.record)
    def path(self,label,names,Ts,action='install',scope=''):
        ob=self.hand+self.pack+self.data[6]+self.data[7]+[self.by[n] for n in self.installed if n not in names];rows=[]
        for T in Ts:
            W=g.original.WORLD@T@g.original.INV;rows.append(dict(transform_local=T.tolist(),native=c.material([dict(name=n,shape=g.original.transform(self.by[n]['shape'],W)) for n in names],ob)))
        got=dict(id=label,moving=list(names),action=action,installed_before=list(self.installed),omitted_original_guide=[z['name'] for z in self.original if z['name'] not in self.installed],not_yet_installed_new=[z['name'] for z in self.new+self.housing if z['name'] not in self.installed+names],missing_hand_parts=[],scope=scope,samples=rows,clear=all(m.drive.clear(z['native']) for z in rows))
        self.record['paths'].append(got);self.save();print(label,got['clear'],flush=True)
        if not got['clear']:
            self.record.update(completed=False,stopped_at_failed_path=label,new_samples=sum(len(z['samples']) for z in self.record['paths']),sampled_sequence_clear=False);self.save();return False
        if action=='install':self.installed.extend(n for n in names if n not in self.installed)
        elif action=='remove':self.installed=[n for n in self.installed if n not in names]
        return True
def sequence(a):
    Q1=g.original.layout()[1]['socket_transform'];Q0=g.original.layout()[0]['socket_transform'];x1,z1=Q1[:3,0],Q1[:3,2];x0,u0,z0=Q0[:3,0],Q0[:3,1],-Q0[:3,2]
    def straight(label,name,axis,ds,action='install',scope=''):
        return a.path(label,[name],[translation(axis*d) for d in ds],action,scope)
    if not straight('upper_original_nut','liner_keeper_M2_nut_1',x1,[-12,-6,0],scope='Restore upper original nut beside carrier;new cap is manually held until original bolt is inserted.'):return
    if not straight('upper_original_bolt','liner_keeper_M2x10_1',z1,[8,4,0],scope='Restore original M2 envelope from outward radial side;no thread/driver validation.'):return
    if not straight('upper_ferrule','distal_flanged_ferrule_1',z1,[-14,-7,-3,-1,0],scope='Bare ferrule from open inner side of new upper half-seat;no housing or cord installed.'):return
    if not straight('upper_shoe','distal_housing_shoe_1',z1,[-12,-6,-3,-1,0],scope='Open lower half shoe enters inward radial side;ferrule manually supported until fasteners engage.'):return
    for j in [0,1]:
        if not straight('upper_shoe_nut_'+str(j),'distal_shoe_M2_nut_1_'+str(j),x1,[12,6,0],scope='Both nuts placed from positiveX before either new bolt;lowerX nut first to avoid traversing the later outer nut.'):return
    for j in [0,1]:
        if not straight('upper_shoe_bolt_'+str(j),'distal_shoe_M2x10_1_'+str(j),z1,[8,4,0],scope='Nominal smooth bolt from outward side;nut already held at seat.'):return
    if not straight('remove_lower_original_bolt','liner_keeper_M2x10_0',z0,[0,4,8],'remove','Original lower guide still present;unscrewing/thread/tool motion not modeled.'):return
    if not straight('remove_lower_original_nut','liner_keeper_M2_nut_0',x0,[0,-6,-12],'remove','Original lower bolt removed;manual nut removal through existing side approach.'):return
    if not straight('remove_lower_original_keeper','liner_flange_keeper_0',z0,[0,7,14],'remove','Original lower liner retained manually in carrier half-seat;no child/pin removed.'):return
    if not straight('lower_bare_ferrule','distal_flanged_ferrule_0',u0,[-25,-15,-5,-1,0],scope='Bare closed ferrule threads axially over the existing thin liner before either new lower half-seat is fitted.'):return
    Ts=[translation(x0*d-z0*3) for d in [-20,-10,-5,0]]+[translation(z0*d) for d in [-2,-1,-.5,0]]
    if not a.path('lower_shoe_side_then_radial',['distal_housing_shoe_0'],Ts,scope='Shoe approaches laterally at3mm inward radial offset,then closes radially;actual start outside the hand,not a teleport into a trapped slot.Ferrule/liner held manually.No fixture/tool qualification.'):return
    if not straight('lower_new_keeper','distal_housing_keeper_0',z0,[14,7,3,1,0],scope='New outer half keeper closes after shoe/ferrule;thin liner and full child/pin remain present.'):return
    if not straight('lower_original_nut','liner_keeper_M2_nut_0',x0,[-12,-6,0],scope='Restore original lower mounting nut with new keeper present.'):return
    if not straight('lower_original_bolt','liner_keeper_M2x10_0',z0,[8,4,0],scope='Restore original lower bolt before new shoe fasteners.'):return
    for j in [1,0]:
        if not straight('lower_shoe_nut_'+str(j),'distal_shoe_M2_nut_0_'+str(j),x0,[-12,-6,0],scope='HigherX nut first when entering from negativeX;other nut not yet installed.'):return
    for j in [1,0]:
        if not straight('lower_shoe_bolt_'+str(j),'distal_shoe_M2x10_0_'+str(j),z0,[8,4,0],scope='Nominal bolt envelope;thread,torque and real driver unqualified.'):return
    for i,Q in [(1,Q1),(0,Q0)]:
        if not straight('housing_end_'+str(i),'distal_short_housing_'+str(i),Q[:3,1],[-30,-15,-5,-1,0],scope='Only24mm straight housing-end reference insertion into ferrule.Not full long housing installation,real flexible shape,retention or cord threading.'):return
    wanted={z['name'] for z in a.new+a.housing}+{z['name'] for z in a.original if z['name'] not in ['liner_flange_keeper_0','liner_flange_keeper_1']}
    a.record.update(completed=True,sampled_sequence_clear=True,new_samples=sum(len(z['samples']) for z in a.record['paths']),final_installed=list(a.installed),final_candidate_rows_restored=set(a.installed)==wanted,input_hash_guard=all(m.drive.sha(ROOT/z)==h for z,h in a.data[0].items()));a.save()
def main():
    out=HERE/'reviews/service.json';assert not out.exists();start=time.monotonic();a=Assembly(*setup(),out);sequence(a);a.record['elapsed_s']=time.monotonic()-start;a.save()
if __name__=='__main__':main()
