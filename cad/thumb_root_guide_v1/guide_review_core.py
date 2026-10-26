"""Exact snapshot replay of a partial distal-guide candidate, not hand approval."""
import sys,argparse,subprocess
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import guide_candidate3 as g
check=g.g.drive.module('_guide020_review_inputs',Path(__file__).resolve().parent/'build_check.py')
check.g=g;check.HERE=g.HERE
c,b,m,p,cq,np,ROOT=g.c,g.b,g.m,g.p,g.cq,g.np,g.ROOT
HERE=Path(__file__).resolve().parent
from OCP.OSD import OSD_ThreadPool
metric=g.g.drive.module('_guide020_metrics',ROOT/'cad/thumb_tendon_route_v1/review_core.py')

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    assert not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1')
    manifest=p.read(ROOT/'checkpoints/GUIDE020.manifest.json');frozen={}
    for key in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[key])
    def guard():return all(g.g.drive.sha(ROOT/z)==v['sha256'] for z,v in manifest['files'].items()) and all(g.g.drive.sha(ROOT/z)==h for z,h in frozen.items())
    assert guard();expected=p.read(g.HERE/'reviews/check.json');service=p.read(g.HERE/'reviews/final_service.json');differences=[];samples=[];comparisons=[]
    parts,design=g.build();saved=c.load(p.read(ROOT/expected['scene'])['parts']);fresh={z['name']:z for z in parts}
    for z in saved:
        got=metric.match(metric.signature(fresh[z['name']]['shape']),metric.signature(z['shape']));got['name']=z['name'];comparisons.append(got)
        if not got['matched']:differences.append('shape_'+z['name'])
    opening=next(z for z in expected['cases'] if z['id']=='open');cord_saved=c.load(p.read(ROOT/opening['cord_scene'])['parts']);cord_fresh={z['name']:z for z in g.cord_pieces(opening['angles_deg'])}
    for z in cord_saved:
        got=metric.match(metric.signature(cord_fresh[z['name']]['shape']),metric.signature(z['shape']));got['name']=z['name'];comparisons.append(got)
        if not got['matched']:differences.append('shape_'+z['name'])
    by={z['name']:z for z in saved};data=check.inputs(design);v=data[1];cat=data[2];scene=data[3];hand=data[4];neutral=v.frames(cat['design'],np.zeros(20))
    def compare(label,got,want):
        def mapping(z):return {(r['lead'],r['part']):r['material_intersection_mm3'] for r in z['overlaps']}
        a,z=mapping(got),mapping(want);ok=a.keys()==z.keys() and all(abs(a[k]-z[k])<1e-5 for k in a.keys()&z.keys()) and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope']
        samples.append(dict(id=label,native=got,matched=ok));print(label,ok,flush=True)
        if not ok:differences.append(label)
    def combined(rows):return dict(overlaps=[q for z in rows for q in z['overlaps']],operation_failures=[q for z in rows for q in z['operation_failures']],pairs_in_scope=sum(z['pairs_in_scope'] for z in rows))
    tests=[c.material([z],saved[i+1:]) for i,z in enumerate(saved)]
    compare('new11_internal55',combined(tests),combined(expected['new_pairs']))
    for label,keys in [('open',list(opening['native'])),('parent_full',['new_hand','cords_hand','cords_new'])]:
        want=next(z for z in expected['cases'] if z['id']==label);got,_,_=check.evaluate(want['angles_deg'],saved,design,data)
        for key in keys:compare(label+'_'+key,got['native'][key],want['native'][key])
    def replay_path(path,index):
        q=path['hand_angles_deg'];frames=v.frames(cat['design'],q);T=np.asarray(path['samples'][index]['transform_local']);names=path['moving']
        def pose(row,local_transform=None):
            s=row['shape']
            if local_transform is not None:s=g.original.transform(s,g.original.WORLD@local_transform@g.original.INV)
            return v.relocate(s,frames['thumb_mcp'],neutral['thumb_mcp'])
        obstacles=[dict(name=z['name'],shape=v.relocate(hand[z['name']],frames[z['body']],neutral[z['body']])) for z in scene['hand'] if z['name'] not in design['replaced_hand_parts']]
        obstacles+=data[5]+data[6]+data[7]+[dict(name=n,shape=pose(by[n])) for n in path['already_installed_new_parts'] if n not in names]
        moving=[dict(name=n,shape=pose(by[n],T)) for n in names]
        compare('service_'+path['id']+'_'+str(index),c.material(moving,obstacles),path['samples'][index]['native'])
    paths={z['id']:z for z in service['paths']}
    for name in ['carrier_side_entry','flanged_liner_1_axial_insert','flanged_liner_0_axial_insert']:
        first=max(range(len(paths[name]['samples'])),key=lambda i:sum(z['material_intersection_mm3'] for z in paths[name]['samples'][i]['native']['overlaps'])) if name=='flanged_liner_1_axial_insert' else 0
        for index in [first,len(paths[name]['samples'])-1]:replay_path(paths[name],index)
    bad=paths['initial_upper_keeper_straight_side'];index=max(range(len(bad['samples'])),key=lambda i:len(bad['samples'][i]['native']['overlaps']));replay_path(bad,index)
    cap=paths['liner_flange_keeper_1_forward_then_side_then_seat']
    for index in [4,len(cap['samples'])-1]:replay_path(cap,index)
    path=paths['liner_keeper_M2_nut_1_insert'];replay_path(path,len(path['samples'])-1)
    first=p.read(HERE/'reviews/check.json');old=c.load(p.read(ROOT/first['scene'])['parts']);old_liner=next(z for z in old if z['name']=='flanged_liner_1');cover=dict(name='thumb_segment2_dark_faceplate',shape=hand['thumb_segment2_dark_faceplate'])
    want=next(z for z in first['cases'] if z['id']=='open')['native']['new_hand'];want=dict(want,overlaps=[z for z in want['overlaps'] if z['lead']=='flanged_liner_1'],pairs_in_scope=1,operation_failures=[])
    compare('original_liner_cover_failure',c.material([old_liner],[cover]),want)
    contacts=[];classifications=[];local={n:g.original.transform(z['shape'],g.original.INV) for n,z in by.items()};localhand=g.original.local_hand()[0]
    for want in service['contacts']:
        if 'distance_mm' in want:
            carrier=local['paired_liner_carrier'];washer=localhand[want['id']];distance=carrier.distance(washer);ok=abs(distance-want['distance_mm'])<1e-5;contacts.append(dict(id=want['id'],matched=ok,distance_mm=distance))
            for row in want['classification']:
                point=np.asarray(row['point_mm']);a=point+[0,0,.02];d=point-[0,0,.02];got=[carrier.isInside(cq.Vector(*a),1e-6),carrier.isInside(cq.Vector(*d),1e-6),washer.isInside(cq.Vector(*a),1e-6),washer.isInside(cq.Vector(*d),1e-6)]
                classifications.append(dict(id=want['id'],matched=got==[row[k] for k in ['carrier_above','carrier_below','washer_above','washer_below']]))
        else:
            index=int(want['id'].split('_')[-1]);liner=local['flanged_liner_'+str(index)];base=local['paired_liner_carrier'];cap=local['liner_flange_keeper_'+str(index)];axis=g.original.layout()[index]['socket_transform'][:3,1]
            for key,s in [('rear_seat_distance_mm',base),('front_keeper_distance_mm',cap)]:
                distance=liner.distance(s);contacts.append(dict(id=want['id']+'_'+key,matched=abs(distance-want[key])<1e-5,distance_mm=distance))
            for row in want['classification']:
                point=np.asarray(row['point_mm']);a=point+.02*axis;d=point-.02*axis;s=base if row['plane_y_mm']==0 else cap
                got=[liner.isInside(cq.Vector(*a),1e-6),liner.isInside(cq.Vector(*d),1e-6),s.isInside(cq.Vector(*a),1e-6),s.isInside(cq.Vector(*d),1e-6)]
                classifications.append(dict(id=want['id'],matched=got==[row[k] for k in ['liner_above','liner_below','support_above','support_below']]))
    for z in contacts+classifications:
        if not z['matched']:differences.append('interface_'+z['id'])
    lengths=[]
    for want in service['constraints']['parent_only_distal_length_study']:
        q=want['parent_angles_deg'];actual=[]
        for row in g.original.layout():
            angle=(-row['side'])*(np.deg2rad(q[19])-g.g.old.route(0.)[row['cord']]['contact_angle_rad'])
            actual.append(sum(z['length_mm'] for z in row['pieces'])+np.linalg.norm(row['exit']-row['target'])+7.7*angle+5.)
        ok=max(abs(np.asarray(actual)-want['local_lengths_mm']))<1e-7;lengths.append(dict(fraction=want['fraction'],local_lengths_mm=actual,matched=bool(ok)))
        if not ok:differences.append('local_length_'+str(want['fraction']))
    clean=not git('status','--porcelain=v1');result=dict(completed=True,target_commit=args.commit,passed=not differences and guard() and clean,differences=differences,key_samples=samples,geometry_comparisons=comparisons,contact_distances=contacts,interface_point_classifications=classifications,local_parent_length_replay=lengths,
        manifest_files=len(manifest['files']),frozen_files=len(frozen),input_hash_guard=guard(),checkout_clean=clean,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),independent_audit=False,whole_hand_drive_approved=False,
        scope='25native groups:11new internal55pairs;open10groups/fullparent3;10selected installation probes incl.original cap failure;first liner/cover obstruction.23regenerated hardware/cord geometry metric comparisons;6interface distances/24points96material predicates;17analytical distal-only lengths.All364hand/368packaging/64electrical+outerbox/10neutralarm imported.Not all8native poses/all63service samples/render/full root conduit/tension/supplier/physical replay or approval.')
    p.write(args.output,result);assert result['passed']
if __name__=='__main__':main()
