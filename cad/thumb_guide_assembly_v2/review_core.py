"""Exact-SHA scoped replay; failed old paths remain failed evidence."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from probe import *
from material_cache import install
install(c)
import argparse,subprocess
from OCP.OSD import OSD_ThreadPool
metric=g.g.drive.module('_guide021_metrics',ROOT/'cad/thumb_tendon_route_v1/review_core.py')
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1')
    manifest=p.read(ROOT/'checkpoints/GUIDE021.manifest.json');frozen={}
    for key in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[key])
    def guard():return all(g.g.drive.sha(ROOT/z)==v['sha256'] for z,v in manifest['files'].items()) and all(g.g.drive.sha(ROOT/z)==h for z,h in frozen.items())
    assert guard();static=p.read(HERE/'reviews/static.json');service=p.read(HERE/'reviews/selected_service.json');probe=p.read(HERE/'reviews/probe.json')
    model=g.g.drive.module('_guide021_model',HERE/'radial_model.py');parts,design=model.build();data=check.inputs(design)
    saved=c.load(p.read(ROOT/static['scene'])['parts']);by={z['name']:z for z in saved};fresh={z['name']:z for z in parts};differences=[];geometry=[];native=[]
    for z in saved:
        got=metric.match(metric.signature(fresh[z['name']]['shape']),metric.signature(z['shape']));got['name']=z['name'];geometry.append(got)
        if not got['matched']:differences.append('shape_'+z['name'])
    def compare(label,got,want):
        def mapping(z):return {(r['lead'],r['part']):r['material_intersection_mm3'] for r in z['overlaps']}
        a,b=mapping(got),mapping(want);ok=a.keys()==b.keys() and all(abs(a[k]-b[k])<1e-5 for k in a.keys()&b.keys()) and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope']
        native.append(dict(id=label,native=got,matched=ok));print(label,ok,flush=True)
        if not ok:differences.append(label)
    def combined(rows):return dict(overlaps=[q for z in rows for q in z['overlaps']],operation_failures=[q for z in rows for q in z['operation_failures']],pairs_in_scope=sum(z['pairs_in_scope'] for z in rows))
    compare('new_internal55',combined([c.material([z],saved[i+1:]) for i,z in enumerate(saved)]),combined(static['new_pairs']))
    v,cat,scene,hand=data[1:5];neutral=v.frames(cat['design'],np.zeros(20))
    for label,keys in [('open',['new_hand','new_packaging','cords_new']),('parent_full',['new_hand','cords_new'])]:
        case=next(z for z in static['cases'] if z['id']==label);frames=v.frames(cat['design'],case['angles_deg']);posed=[dict(z,shape=v.relocate(z['shape'],frames[z['body']],neutral[z['body']])) for z in saved]
        retained=[dict(name=z['name'],shape=v.relocate(hand[z['name']],frames[z['body']],neutral[z['body']])) for z in scene['hand'] if z['name'] not in design['replaced_hand_parts']]
        for key in keys:
            got=c.material(posed,retained) if key=='new_hand' else c.material(posed,data[5]) if key=='new_packaging' else c.material(g.cord_pieces(case['angles_deg']),posed)
            compare(label+'_'+key,got,case['native'][key])
    def replay_path(path,index):
        T=np.asarray(path['samples'][index]['transform_local']);names=path['moving'];absent=path['missing_hand_parts']
        obstacles=[dict(name=z['name'],shape=hand[z['name']]) for z in scene['hand'] if z['name'] not in absent+design['replaced_hand_parts']+names]
        obstacles+=data[5]+data[6]+data[7]+[by[n] for n in path['installed_new'] if n not in names]
        moving=[dict(name=n,shape=g.original.transform(by[n]['shape'] if n in by else hand[n],g.original.WORLD@T@g.original.INV)) for n in names]
        compare('assembly_'+path['id']+'_'+str(index),c.material(moving,obstacles),path['samples'][index]['native'])
    for path in service['paths']:
        if path['id'] in ['flanged_liner_1_radial','flanged_liner_0_radial','restore_child_and_inner_thrust_washers','restore_pin','liner_keeper_M2_nut_1_side_insert','liner_keeper_M2_nut_0_side_insert']:
            for i in sorted({0,len(path['samples'])//2,len(path['samples'])-1}):replay_path(path,i)
        elif path['id'] in ['liner_flange_keeper_1_radial','liner_flange_keeper_0_radial','restore_thumb_distal2_retaining_screw']:
            for i in [0,len(path['samples'])-1]:replay_path(path,i)
    for want in service['roll_probes']:
        n=want['id'].rsplit('_',1)[0];Q=g.original.layout()[int(n[-1])]['socket_transform'];T=rotate(Q[:3,3],Q[:3,1],want['angle_deg'])
        local={n:g.original.transform(z['shape'],g.original.INV) for n,z in by.items()}
        compare('indexed_roll_'+want['id'],c.material([dict(name=n,shape=g.original.transform(local[n],T))],[dict(name=k,shape=s) for k,s in local.items() if k!=n]),want['native'])
        tab=g.original.transform(g.original.box([1.6,0,-.5],[2.6,2,.5]),Q)
        compare('indexed_tab_'+want['id'],c.material([dict(name=n+'_tab_evidence_only',shape=g.original.transform(tab,T))],[dict(name=k,shape=s) for k,s in local.items() if k!=n]),want['tab_only_native'])
    prior_service=p.read(HERE/'reviews/final_service.json')
    for path in prior_service['paths']:
        if path['id'] not in ['liner_keeper_M2_nut_1_radial','liner_keeper_M2_nut_0_radial']:continue
        index=max(range(len(path['samples'])),key=lambda i:sum(z['material_intersection_mm3'] for z in path['samples'][i]['native']['overlaps']))
        replay_path(path,index)
    # Preserve old no-child insertion failure and upper roll freedom, without re-running the old search.
    oldparts=c.load(p.read(ROOT/'cad/thumb_root_guide_v1/selected2/generated/parts/scene.json')['parts']);old={z['name']:dict(z,local_shape=g.original.transform(z['shape'],g.original.INV)) for z in oldparts};localhand={n:g.original.transform(s,g.original.INV) for n,s in hand.items()}
    path=next(z for z in probe['probes'] if z['id']=='no_tip_straight_1');i=2;T=np.asarray(path['samples'][i]['transform_local']);names=path['moving']
    obstacles=[dict(name=n,shape=s) for n,s in localhand.items() if n not in path['missing_hand_parts']+design['replaced_hand_parts']+names]+[dict(name=n,shape=old[n]['local_shape']) for n in path['installed_new'] if n not in names]
    compare('old_upper_no_tip_axial_failure',c.material([dict(name=n,shape=g.original.transform(old[n]['local_shape'],T)) for n in names],obstacles),path['samples'][i]['native'])
    for want in probe['roll_probes']:
        if want['id'] not in ['flanged_liner_1_-2.0','flanged_liner_1_2.0']:continue
        n='flanged_liner_1';T=np.asarray(want['transform_local']);moving=[dict(name=n,shape=g.original.transform(old[n]['local_shape'],T))]
        compare('old_upper_free_roll_'+want['id'],c.material(moving,[dict(name=k,shape=s) for k,s in localhand.items() if k.startswith('thumb_segment2_') and k not in design['replaced_hand_parts']]),want['native_parent'])
    clean=not git('status','--porcelain=v1')
    result=dict(completed=True,target_commit=args.commit,passed=not differences and guard() and clean,differences=differences,key_samples=native,geometry_comparisons=geometry,manifest_files=len(manifest['files']),frozen_files=len(frozen),input_hash_guard=guard(),checkout_clean=clean,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),independent_audit=False,whole_hand_drive_approved=False,
        scope='11regenerated/saved guide shapes,selected static/assembly/index-stop groups,original upper failure and free roll.Not all8static or all installation samples/render,continuous motion,forces,tools,pretension,full motor-root route or physical certification.')
    p.write(args.output,result);assert result['passed'],differences
if __name__=='__main__':main()
