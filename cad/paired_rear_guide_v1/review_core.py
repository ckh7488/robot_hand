"""Exact-SHA scoped replay of actual paired channels and prescribed feed space."""
import sys,argparse,subprocess
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import check_pair as b
a,m,g,c,p,np,cq,ROOT,HERE=b.a,b.m,b.g,b.c,b.p,b.np,b.cq,b.ROOT,b.HERE
measure=m.drive.module('_pair034_metrics',ROOT/'cad/long_housing_route_v1/review_core.py')
from OCP.OSD import OSD_ThreadPool

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert not args.output.exists() and not args.output.resolve().is_relative_to(ROOT) and git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1')
    man=p.read(ROOT/'checkpoints/PAIR034.manifest.json');frozen={}
    for k in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[k])
    def guard():return all(m.drive.sha(ROOT/z)==v['sha256'] for z,v in man['files'].items()) and all(m.drive.sha(ROOT/z)==v for z,v in frozen.items())
    assert guard();static=p.read(HERE/'reviews/static.json');sr=p.read(HERE/'reviews/service.json');new,retained,design=a.build();scene=p.read(HERE/'generated/assembly_scene.json');saved=c.load(scene['new_parts']);rows=saved+retained;geometry=[];native=[];differences=[]
    L=design['centerline_length_mm'];envelopes=[a.window(i,L+10,L+230) for i in [0,1]]
    for i,z in enumerate(envelopes):z['name']='prescribed_feed_union_envelope_'+str(i)
    lookup={z['name']:z for z in new+envelopes};saved_env=c.load(p.read(ROOT/sr['feed_union_envelope_scene'])['parts'])
    for z in saved+saved_env:
        got=dict(name=z['name'],**measure.match(measure.metrics(lookup[z['name']]['shape']),measure.metrics(z['shape'])));geometry.append(got)
        if not got['matched']:differences.append('geometry_'+z['name'])
    data,pack,guide=b.base.inputs()
    def combined(xs):return dict(pairs_in_scope=sum(x['pairs_in_scope'] for x in xs),overlaps=[z for x in xs for z in x['overlaps']],operation_failures=[z for x in xs for z in x['operation_failures']])
    def compare(label,got,want):
        mapping=lambda x:{(z['lead'],z['part']):z['material_intersection_mm3'] for z in x['overlaps']}
        aa,bb=mapping(got),mapping(want);err=max([abs(aa[k]-bb[k]) for k in aa.keys()&bb.keys()]+[0.]);ok=aa.keys()==bb.keys() and err<1e-5 and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope']
        native.append(dict(id=label,native=got,max_numeric_difference=err,matched=bool(ok)));print(label,ok,flush=True)
        if not ok:differences.append(label)
    compare('internal45',combined([c.material([z],rows[j+1:]) for j,z in enumerate(rows)]),combined(static['internal']))
    for tag in ['hand_open','hand_parent_full','wrist_-20_-15','wrist_20_15']:
        case=next(z for z in static['cases'] if z['id']==tag);P,H=a.a.wrist_frames(case['wrist_flex_deg'],case['wrist_deviation_deg']);hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,case['angles_deg']);moving=[a.a.transform(z,H) for z in rows]
        for k,ob in [('hand',[a.a.transform(z,H) for z in hand]),('packaging',b.scope.posed_pack(pack,H)),('electrical',data[6]),('arm',a.a.posed_arm(data[7],P,H))]:compare(tag+'_'+k,c.material(moving,ob),case['native'][k])
    hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,np.zeros(20));fixed=hand+pack+data[6]+data[7];hardware=[z for z in rows if z['installed_part_candidate']];installed=[]
    for path in sr['paths']:
        assert path['installed_before']==[z['name'] for z in installed];part=next(z for z in hardware if z['name']==path['name']);assert path['absent_hardware']==[z['name'] for z in hardware if z['name'] not in path['installed_before']+[part['name']]]
        for sample in [path['samples'][0],path['samples'][-1]]:compare('assembly_'+path['name']+'_'+str(sample['translation_world_mm'][1]),c.material([dict(part,shape=part['shape'].translate(tuple(sample['translation_world_mm'])))],fixed+installed),sample['native'])
        installed.append(part)
    for i,env in enumerate(saved_env):compare('prescribed_feed_union_'+str(i),c.material([env],fixed+hardware),sr['feed_union_envelope_native'][i])
    compare('between_prescribed_feed_unions',c.material([saved_env[0]],[saved_env[1]]),sr['between_union_envelopes'])
    containment=[]
    for j in [0,3,len(sr['feeding_samples'])-1]:
        sample=sr['feeding_samples'][j];tip=sample['tip_material_coordinate_mm'];refs=[a.window(i,tip) for i in [0,1]]
        for i,z in enumerate(refs):
            compare('feed_'+str(j)+'_'+str(i),c.material([z],fixed+hardware),sample['native'][i]);sh=z['shape'].cut(saved_env[i]['shape'],tol=1e-6);v=sh.Volume() if sh.Solids() else 0.;containment.append(dict(sample_index=j,line=i,outside_mm3=v))
            if abs(v-sample['material_outside_union_envelope_mm3'][i])>1e-8:differences.append('feed_containment_'+str(j)+'_'+str(i))
        compare('feed_between_'+str(j),c.material([refs[0]],[refs[1]]),sample['between_lines'])
    clip=hardware[0]['shape'];cap=next(z['shape'] for z in hardware if z['name']=='wrist_edge_clip_front_cap');saddle=next(z['shape'] for z in data[7] if z['name']=='Palm_wrist_saddle');contacts=[]
    for item in sr['contacts']:
        pt=np.asarray(item['point_mm']);n=np.asarray(item['normal']);first=cap if item['id']=='front_face' else clip;ds=[sh.distance(cq.Vertex.makeVertex(*pt)) for sh in [first,saddle]];mat=[[sh.isInside(cq.Vector(*(pt+s*.05*n)),1e-6) for s in [-1,1]] for sh in [first,saddle]];ok=np.max(abs(np.asarray(ds)-item['distance_mm']))<1e-5 and mat==item['material'];contacts.append(dict(id=item['id'],distance_mm=ds,material=mat,matched=bool(ok)))
        if not ok:differences.append('contact_'+item['id'])
    if abs(design['old_body_material_removed_mm3']-static['design']['old_body_material_removed_mm3'])>1e-8:differences.append('old_body_material')
    clean=not git('status','--porcelain=v1');result=dict(completed=True,target_commit=args.commit,passed=not differences and guard() and clean,differences=differences,key_samples=native,geometry_comparisons=geometry,contacts=contacts,feed_containment=containment,manifest_files=len(man['files']),frozen_files=len(frozen),input_hash_guard=guard(),checkout_clean=clean,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),independent_audit=False,whole_hand_drive_approved=False,scope='41nativegroups:internal1+4posesx4=17;6hardware paths start/end12=29;2feedunions+between1=32;3feeding references x(2environment+between)=41.7adaptive source/STEP signatures abs1e-5(new5+union2),3seats/6feedcontainment. Not other12static/middle18assembly/other7feeding/render/fullflexiblelink/friction/elasticfeeding/tension/physical.39palm packaging followsH,346forearm+64elect fixed;reference arm frames not actual hand-arm attachment approval.Inherited comparisons untouched.')
    p.write(args.output,result);assert result['passed'],differences
if __name__=='__main__':main()
