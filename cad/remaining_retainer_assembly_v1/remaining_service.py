"""Remaining clamp entries and the captured lower ferrule, using saved solids.

Two independent conditional assembly studies. No whole-housing or grip load
approval is inferred from finite rigid-part translations.
"""
import sys,time
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'cad/wrist_external_feed_v1'))
import feed_model as a
m,g,c,p,np,cq=a.m,a.g,a.c,a.p,a.np,a.cq
base=m.drive.module('_remaining032_inputs',ROOT/'cad/long_housing_route_v1/build_check.py')
from OCP.OSD import OSD_ThreadPool

def inputs():
    data,pack,guide=base.inputs();hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,np.zeros(20))
    scene=p.read(ROOT/'cad/wrist_external_feed_v1/generated/assembly_scene.json')
    hardware=c.load([z for z in scene['new_parts']+scene['retained_parts'] if z['installed_part_candidate']],data[0])
    housing=c.load(p.read(ROOT/'cad/thumb_sheath_terminal_v2/selected/generated/housing_reference/scene.json')['parts'],data[0])+c.load(p.read(ROOT/'cad/motor_tendon_terminal_v1/selected/generated/housing_reference/scene.json')['parts'],data[0])
    original=c.load([z for z in p.read(ROOT/'cad/thumb_guide_assembly_v2/generated/parts/scene.json')['parts'] if z['name']=='liner_flange_keeper_0'],data[0])
    allrows=hand+pack+data[6]+data[7]+hardware+housing
    assert [len(hand),len(pack),len(data[6]),len(data[7]),len(hardware),len(housing)]==[397,385,64,10,6,4]
    by={z['name']:z for z in allrows+original};assert len(by)==len(allrows)+1
    for rel in ['cad/wrist_external_feed_v1/generated/assembly_scene.json','cad/sheath_retention_v1/selected/reviews/check.json','cad/remaining_retainer_assembly_v1/remaining_service.py']:data[0][rel]=m.drive.sha(ROOT/rel)
    return data,allrows,by

class Study:
    def __init__(self,by,initial,label):
        self.by=by;self.installed=list(initial);self.paths=[];self.label=label;self.initial=list(initial)
    def path(self,label,names,deltas,action='install'):
        assert len(set(self.installed))==len(self.installed)
        assert all((n in self.installed)==(action=='remove') for n in names)
        obstacle=[n for n in self.installed if n not in names];samples=[]
        for delta in deltas:
            delta=np.asarray(delta,dtype=float);moving=[dict(self.by[n],shape=self.by[n]['shape'].translate(tuple(delta))) for n in names]
            samples.append(dict(translation_world_mm=delta.tolist(),native=c.material(moving,[self.by[n] for n in obstacle])))
        got=dict(id=label,moving=list(names),action=action,installed_before=list(self.installed),obstacle_names=obstacle,absent_before=sorted(set(self.by)-set(self.installed)),samples=samples,clear=all(m.drive.clear(s['native']) for s in samples))
        self.paths.append(got);print(self.label,label,got['clear'],flush=True)
        if got['clear']:
            if action=='remove':self.installed=[n for n in self.installed if n not in names]
            else:self.installed+=names
        return got['clear']
    def straight(self,label,name,axis,ds,action='install'):
        return self.path(label,[name],[np.asarray(axis)*d for d in ds],action)
    def report(self):return dict(id=self.label,initial_installed=self.initial,paths=self.paths,final_installed=list(self.installed),clear=all(z['clear'] for z in self.paths))

def motor(st,ports):
    tag='motor_housing_clamp_1';axis=np.asarray(next(z for z in ports if z['tag']==tag)['Q'])[:3,2]
    for suffix,direction,ds in [('jaw_-1',-axis,[14,7,2,0]),('jaw_1',axis,[14,7,2,0])]+[('nut_'+str(j),-axis,[10,5,0]) for j in [0,1]]+[('bolt_'+str(j),axis,[10,5,0]) for j in [0,1]]:
        if not st.straight(suffix,tag+'_'+suffix,direction,ds):return False
    return True

def lower(st,ports):
    Q=g.original.layout()[0]['socket_transform'];R=g.original.WORLD[:3,:3];x,u,z=R@Q[:3,0],R@Q[:3,1],-R@Q[:3,2]
    seq=[('remove_original_bolt','liner_keeper_M2x10_0',z,[0,4,8],'remove'),('remove_original_nut','liner_keeper_M2_nut_0',x,[0,-6,-12],'remove'),('remove_original_keeper','liner_flange_keeper_0',z,[0,7,14],'remove'),('captured_ferrule_axial','distal_flanged_ferrule_0_captured',u,[-25,-15,-5,-1,0],'install')]
    for label,name,axis,ds,action in seq:
        if not st.straight(label,name,axis,ds,action):return False
    if not st.path('shoe_side_then_radial',['distal_housing_shoe_0'],[x*d-z*3 for d in [-20,-10,-5,0]]+[z*d for d in [-2,-1,-.5,0]]):return False
    seq=[('keeper','distal_housing_keeper_0',z,[14,7,3,1,0]),('original_nut','liner_keeper_M2_nut_0',x,[-12,-6,0]),('original_bolt','liner_keeper_M2x10_0',z,[8,4,0])]+[('shoe_nut_'+str(j),'distal_shoe_M2_nut_0_'+str(j),x,[-12,-6,0]) for j in [1,0]]+[('shoe_bolt_'+str(j),'distal_shoe_M2x10_0_'+str(j),z,[8,4,0]) for j in [1,0]]+[('straight_end_reference','distal_short_housing_0',u,[-30,-15,-5,-1,0])]
    for label,name,axis,ds in seq:
        if not st.straight(label,name,axis,ds):return False
    tag='distal_housing_clamp_0';axis=np.asarray(next(z for z in ports if z['tag']==tag)['Q'])[:3,2]
    for suffix,direction,ds in [('jaw_-1',-axis,[14,7,2,0]),('jaw_1',axis,[14,7,2,0])]+[('nut_'+str(j),-axis,[10,5,0]) for j in [0,1]]+[('bolt_'+str(j),axis,[10,5,0]) for j in [0,1]]:
        if not st.straight(suffix,tag+'_'+suffix,direction,ds):return False
    return True

def main():
    out=HERE/'reviews/service.json';assert not out.exists();start=time.monotonic();data,allrows,by=inputs();names=[z['name'] for z in allrows];ports=p.read(ROOT/'cad/sheath_retention_v1/selected/reviews/check.json')['design']['ports']
    absent_motor={n for n in names if n.startswith('motor_housing_clamp_1_')};assert len(absent_motor)==6
    st=Study(by,[n for n in names if n not in absent_motor],'remaining_motor_1');motor(st,ports);studies=[st.report()]
    absent_lower={n for n in names if n.startswith('distal_housing_clamp_0_') or n in ['distal_flanged_ferrule_0_captured','distal_housing_shoe_0','distal_housing_keeper_0','distal_short_housing_0'] or n.startswith('distal_shoe_M2') and '_0_' in n};assert len(absent_lower)==14,absent_lower
    st=Study(by,[n for n in names if n not in absent_lower]+['liner_flange_keeper_0'],'lower_captured_ferrule_and_clamp');lower(st,ports);studies.append(st.report())
    tests=[s['native'] for study in studies for path in study['paths'] for s in path['samples']]
    r=dict(completed=True,source_sha256=data[0],studies=studies,sampled_paths_clear=all(z['clear'] for z in studies),new_finite_samples=len(tests),pairs_in_scope=sum(z['pairs_in_scope'] for z in tests),AABB_native_pairs=sum(z['exact_pairs'] for z in tests),overlap_records=sum(len(z['overlaps']) for z in tests),operation_failures=sum(len(z['operation_failures']) for z in tests),final_current_configuration_restored=[set(z['final_installed'])==set(names) for z in studies],OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,hand_angles_deg=[0.]*20,wrist_flex_deviation_deg=[0,0],independent_conditional_studies=True,manual_holding_of_loose_parts_required=True,physical_fixture_tools_threads_torque_strength_verified=False,grip_compression_friction_pullout_verified=False,long_housing_installed=False,cord_threading_tested=False,whole_hand_drive_approved=False,geometry_changed=False,input_hash_guard=all(m.drive.sha(ROOT/z)==h for z,h in data[0].items()),elapsed_s=time.monotonic()-start)
    p.write(out,r);print('finished',r['sampled_paths_clear'],len(tests),r['overlap_records'],flush=True)
if __name__=='__main__':main()
