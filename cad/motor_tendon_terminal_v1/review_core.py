"""Exact-SHA scoped native replay; no physical or full-thumb approval."""
import sys,argparse,subprocess
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import terminal_selected as model
g,c,p,np,cq,HERE,ROOT=model.g,model.c,model.p,model.np,model.cq,model.HERE,model.ROOT
H=Path(__file__).resolve().parent
check=model.drive.module('_terminal022_replay_inputs',H/'build_check.py');check.model=model;check.HERE=HERE
metric=model.drive.module('_terminal022_metrics',ROOT/'cad/thumb_tendon_route_v1/review_core.py')
from OCP.OSD import OSD_ThreadPool
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert not args.output.exists() and not args.output.resolve().is_relative_to(ROOT) and git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1')
    manifest=p.read(ROOT/'checkpoints/TERMINAL022.manifest.json');frozen={}
    for key in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[key])
    def guard():return all(model.drive.sha(ROOT/z)==v['sha256'] for z,v in manifest['files'].items()) and all(model.drive.sha(ROOT/z)==h for z,h in frozen.items())
    assert guard();expected=p.read(HERE/'reviews/check.json');service=p.read(HERE/'reviews/selected_service.json');parts,design=model.build();data,retained,guide=check.inputs(design)
    saved=c.load(p.read(ROOT/expected['scene'])['parts']);by={z['name']:z for z in saved};fixedby={z['name']:z for z in retained};geometry=[];native=[];differences=[];contacts=[];lengths=[]
    for fresh,rel in [(parts,expected['scene']),(model.cords(),expected['cord_scene']),(model.witnesses(),expected['housing_scene'])]:
        lookup={z['name']:z for z in fresh}
        for z in c.load(p.read(ROOT/rel)['parts']):
            got=metric.match(metric.signature(lookup[z['name']]['shape']),metric.signature(z['shape']));got['name']=z['name'];geometry.append(got)
            if not got['matched']:differences.append('shape_'+z['name'])
    def compare(label,got,want):
        def mapping(z):return {(r['lead'],r['part']):r['material_intersection_mm3'] for r in z['overlaps']}
        a,b=mapping(got),mapping(want);ok=a.keys()==b.keys() and all(abs(a[k]-b[k])<1e-5 for k in a.keys()&b.keys()) and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope']
        native.append(dict(id=label,native=got,matched=ok));print(label,ok,flush=True)
        if not ok:differences.append(label)
    def combined(rows):return dict(pairs_in_scope=sum(z['pairs_in_scope'] for z in rows),overlaps=[v for z in rows for v in z['overlaps']],operation_failures=[v for z in rows for v in z['operation_failures']])
    compare('internal21',combined([c.material([z],saved[i+1:]) for i,z in enumerate(saved)]),combined(expected['internal']))
    coupling,_=model.drive.build();envelopes,_=model.drive.custom_envelopes(coupling);cords=model.cords();housing=model.witnesses()
    tests=[('new_packaging',saved,retained),('new_wires',saved,data[6]),('new_arm',saved,data[7]),('full_coupling_rotation',envelopes,saved),('cords_new',cords,saved),('cords_packaging',cords,retained),('cords_wires',cords,data[6]),('housing_new',housing,saved),('housing_packaging',housing,retained),('housing_wires',housing,data[6]),('housing_arm',housing,data[7])]
    for label,a,b in tests:compare(label,c.material(a,b),expected['tests'][label])
    v,cat,scene,hand=data[1:5];neutral=v.frames(cat['design'],np.zeros(20));replaced=['thumb_segment2_dark_faceplate_clamp1_bolt','thumb_segment2_dark_faceplate_clamp2_bolt']
    def hand_at(q):
        frames=v.frames(cat['design'],q)
        return [dict(name=z['name'],shape=v.relocate(hand[z['name']],frames[z['body']],neutral[z['body']])) for z in scene['hand'] if z['name'] not in replaced]+[dict(z,shape=v.relocate(z['shape'],frames[z['body']],neutral[z['body']])) for z in guide]
    for case in expected['hand_cases']:compare('hand_'+case['id'],c.material(saved+cords+housing,hand_at(case['angles_deg'])),case['native'])
    old=p.read(H/'reviews/check.json');oldparts=c.load(p.read(ROOT/old['scene'])['parts'])
    for label,a,b in [('new_packaging',oldparts,retained),('full_coupling_rotation',envelopes,oldparts)]:compare('original_'+label,c.material(a,b),old['tests'][label])
    hand_fixed=hand_at(np.zeros(20))
    def replay_path(path,index):
        names=path['moving'];vector=path['samples'][index]['translation_mm'];obstacles=[z for z in retained if z['name'] not in path['not_installed_packaging']+names]+[z for z in data[6] if z['name'] not in path['not_connected_electrical']]+data[7]+hand_fixed+[by[n] for n in path['already_installed_new'] if n not in names]
        moving=[dict(name=n,shape=(by[n] if n in by else fixedby[n])['shape'].translate(vector)) for n in names]
        compare(path['id']+'_'+str(index),c.material(moving,obstacles),path['samples'][index]['native'])
    for path in service['paths']:
        indices=[0,2,4] if path['id'] in ['frame_from_below','preassembled_motor_and_output'] else [0,4] if path['id'] in ['M3_before_motor','base_M2_nuts_after_unit'] or path['id'].startswith('barrel_from_below') else [4]
        for i in indices:replay_path(path,i)
    oldservice=p.read(HERE/'reviews/service.json');bad=oldservice['paths'][0];i=max(range(len(bad['samples'])),key=lambda i:sum(z['material_intersection_mm3'] for z in bad['samples'][i]['native']['overlaps']));replay_path(bad,i)
    for want in service['adjustments']:
        i=want['cord'];n='thumb_ip_barrel_adjuster_'+str(i);t=want['barrel_translation_z_mm'];other=[z for z in saved if z['name']!=n]+retained+data[6]+data[7]
        compare('adjust_'+str(i)+'_'+str(t),c.material([dict(name=n,shape=by[n]['shape'].translate((0,0,t))),dict(name=housing[i]['name'],shape=housing[i]['shape'].translate((0,0,t)))],other),want['native'])
    for want,port in zip(service['full_stroke_bounds'],model.channels()):
        i=want['cord'];xy=port['xy'];n='thumb_ip_barrel_adjuster_'+str(i);s=model.cyl(2.5,-159,-135,xy).fuse(model.cyl(3.5,-165,-155,xy)).cut(model.cyl(.65,-165.01,-134.99,xy))
        compare('full_stroke_'+str(i),c.material([dict(name='barrel_'+str(i)+'_whole_stroke_bound',shape=s)],[z for z in saved if z['name']!=n]+retained+data[6]+data[7]),want['native'])
    coupling_names=[z['name'] for z in coupling]
    base_lengths=[sum(z['length_mm'] for z in cords if z['cord']==i) for i in [0,1]]
    for want in service['cord_cases']:
        angle=want['motor_angle_candidate_deg'];now=model.cords(angle);rotated=model.drive.moved(coupling,angle);other=[z for z in retained if z['name'] not in coupling_names]+rotated+saved+data[6]+data[7]
        compare('cord_'+str(angle),c.material(now,other),want['native']);compare('between_'+str(angle),c.material([z for z in now if z['cord']==0],[z for z in now if z['cord']==1]),want['between_cords'])
        self_tests=[]
        for cord in [0,1]:
            same=[z for z in now if z['cord']==cord];self_tests.extend(c.material([z],same[k+2:]) for k,z in enumerate(same))
        compare('self_'+str(angle),combined(self_tests),want['nonadjacent_same_cord'])
        actual=[sum(z['length_mm'] for z in now if z['cord']==i) for i in [0,1]];q=np.zeros(20);q[19]=want['ip_deg'];distal=g.cord_pieces(q);distal0=g.cord_pieces(np.zeros(20))
        change=[sum(z['length_mm'] for z in distal if z['cord']==i)-sum(z['length_mm'] for z in distal0 if z['cord']==i) for i in [0,1]];residual=np.asarray(actual)-base_lengths+change
        ok=max(abs(np.asarray(actual)-want['motor_terminal_lengths_mm']))<1e-7 and max(abs(residual-want['ideal_fixed_housing_residual_mm']))<1e-7
        lengths.append(dict(ip_deg=want['ip_deg'],motor_angle_candidate_deg=angle,residual_mm=residual.tolist(),matched=bool(ok),future_constant_housing_length_assumed_not_built=True))
        if not ok:differences.append('length_'+str(angle))
    supplement=p.read(HERE/'reviews/interfaces.json')
    for want in service['contacts']+supplement['M3_head_contacts']:
        ident=want['id'];frame=by['thumb_ip_twin_sheath_terminal_frame']['shape']
        if ident.startswith('M3_head'):a=frame;b=fixedby[want['part_b']]['shape']
        elif ident.startswith('rack_pad'):a=fixedby[model.NAME+'_wire_guide_base']['shape'];b=frame
        elif ident.startswith('jam_nut'):
            i=int(ident[-1]);side='top' if 'top' in ident else 'bottom';a=by['thumb_ip_barrel_'+side+'_nut_'+str(i)]['shape'];b=frame
        else:i=int(ident[-1]);a=by['thumb_ip_barrel_adjuster_'+str(i)]['shape'];b=housing[i]['shape']
        point=np.asarray(want['point_mm']);axis=np.asarray(want['axis']);above=point+.02*axis;below=point-.02*axis
        vals=[bool(s.isInside(cq.Vector(*pt),1e-6)) for s in [a,b] for pt in [above,below]];distance=a.distance(b);ok=vals==[want[k] for k in ['a_above','a_below','b_above','b_below']] and abs(distance-want['distance_mm'])<1e-5
        contacts.append(dict(id=ident,distance_mm=distance,classifications=vals,matched=ok))
        if not ok:differences.append('contact_'+ident)
    clean=not git('status','--porcelain=v1');result=dict(completed=True,target_commit=args.commit,passed=not differences and guard() and clean,differences=differences,key_samples=native,geometry_comparisons=geometry,contact_replay=contacts,conditional_length_replay=lengths,manifest_files=len(manifest['files']),frozen_files=len(frozen),input_hash_guard=guard(),checkout_clean=clean,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),independent_audit=False,whole_hand_drive_approved=False,
        scope='21shapes/53native groups/22contacts/4conditional motor-terminal plus distal analytical length balances.Selected assembly probes only,not whole40samples/render,continuous full-hand/housing motion,tension,power or strength approval.')
    p.write(args.output,result);assert result['passed'],differences
if __name__=='__main__':main()
