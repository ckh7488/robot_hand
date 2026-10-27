"""Exact-SHA replay of rear-feed hardware, selected poses and insertion bound."""
import sys,argparse,subprocess
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import correct_wrist_scope as b
a,m,g,c,p,np,cq,ROOT,HERE=b.a,b.m,b.g,b.c,b.p,b.np,b.cq,b.ROOT,b.HERE
check=b.b
service=m.drive.module('_feed031_review_service',HERE/'feed_service.py')
measure=m.drive.module('_feed031_review_metrics',ROOT/'cad/long_housing_route_v1/review_core.py')
from OCP.OSD import OSD_ThreadPool

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert not args.output.exists() and not args.output.resolve().is_relative_to(ROOT) and git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1')
    man=p.read(ROOT/'checkpoints/FEED031.manifest.json');frozen={}
    for k in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[k])
    def guard():return all(m.drive.sha(ROOT/z)==v['sha256'] for z,v in man['files'].items()) and all(m.drive.sha(ROOT/z)==v for z,v in frozen.items())
    assert guard();static=p.read(HERE/'reviews/final.json');sr=p.read(HERE/'reviews/service.json');survey=p.read(HERE/'reviews/survey.json');geometry=[];native=[];differences=[]
    new,retained,_=a.build();lookup={z['name']:z for z in new};scene=p.read(HERE/'generated/assembly_scene.json');saved=c.load(scene['new_parts']);rows=saved+retained
    envelopes=[]
    for i,z in enumerate([-3.,2.]):envelopes.append(dict(name='straight_reference_housing_'+str(i)+'_axial_insertion_envelope',shape=a.cyl(100,z,-238,8,1.5).cut(a.cyl(100,z,-238.01,8.01,.65))))
    lookup.update({z['name']:z for z in envelopes});saved_metrics=saved+c.load(p.read(ROOT/sr['insertion_envelope_scene'])['parts'])
    for z in saved_metrics:
        got=dict(name=z['name'],**measure.match(measure.metrics(lookup[z['name']]['shape']),measure.metrics(z['shape'])));geometry.append(got)
        if not got['matched']:differences.append('geometry_'+z['name'])
    data,pack,guide=check.base.inputs()
    def combined(xs):return dict(pairs_in_scope=sum(x['pairs_in_scope'] for x in xs),overlaps=[z for x in xs for z in x['overlaps']],operation_failures=[z for x in xs for z in x['operation_failures']])
    def compare(label,got,want):
        mapping=lambda x:{(z['lead'],z['part']):z['material_intersection_mm3'] for z in x['overlaps']}
        aa,bb=mapping(got),mapping(want);err=max([abs(aa[k]-bb[k]) for k in aa.keys()&bb.keys()]+[0.]);ok=aa.keys()==bb.keys() and err<1e-5 and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope']
        native.append(dict(id=label,native=got,max_numeric_difference=err,matched=bool(ok)));print(label,ok,flush=True)
        if not ok:differences.append(label)
    compare('internal45',combined([c.material([z],rows[j+1:]) for j,z in enumerate(rows)]),combined(static['internal']))
    selected=['hand_open','hand_parent_full','wrist_-20_-15','wrist_20_15']
    for tag in selected:
        case=next(z for z in static['cases'] if z['id']==tag);P,H=a.wrist_frames(case['wrist_flex_deg'],case['wrist_deviation_deg']);hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,case['angles_deg']);moving=[a.transform(z,H) for z in rows]
        for k,ob in [('hand',[a.transform(z,H) for z in hand]),('packaging',b.posed_pack(pack,H)),('electrical',data[6]),('arm',a.posed_arm(data[7],P,H))]:compare(tag+'_'+k,c.material(moving,ob),case['native'][k])
    hand,_,_,_=m.prior.s.k.e.prior.hand_rows(data,guide,np.zeros(20));fixed=hand+pack+data[6]+data[7];hardware=[z for z in rows if z['installed_part_candidate']];installed=[]
    for path in sr['paths']:
        assert path['installed_before']==[z['name'] for z in installed];part=next(z for z in hardware if z['name']==path['name'])
        for sample in [path['samples'][0],path['samples'][-1]]:compare('assembly_'+path['name']+'_'+str(sample['translation_mm'][1]),c.material([dict(part,shape=part['shape'].translate(tuple(sample['translation_mm'])))],fixed+installed),sample['native'])
        installed.append(part)
    outside=[]
    for i,env in enumerate(envelopes):
        compare('insertion_full_envelope_'+str(i),c.material([env],fixed+hardware),sr['straight_end_insertion'][i]['native']);housing=next(z for z in rows if z['name']=='external_feed_housing_'+str(i))
        for dy in [-128,0]:
            sh=housing['shape'].translate((0,dy,0)).cut(env['shape'],tol=1e-6);v=sh.Volume() if sh.Solids() else 0.;outside.append(v)
            if v>1e-8:differences.append('insertion_containment_'+str(i)+'_'+str(dy))
    probes=[dict(name='rear_axis_'+str(x)+'_'+str(i),shape=a.cyl(x,z,-110,-4,2.5)) for x in [26.,100.] for i,z in enumerate([-3.,2.])]
    compare('rear_axes_four_wrist_reference_material',c.material(probes,[z for z in data[7] if z['name'] in ['AK_wrist_flex','AK_wrist_deviation','Wrist_box_carrier','Palm_wrist_saddle']]),survey['cases'][0]['native']['four_wrist_references'])
    envelope=cq.Solid.makeCylinder(60,110,cq.Vector(32,-65,-60),cq.Vector(0,0,1));sleeves=[dict(name='evidence_sleeve_only_'+str(i),shape=a.cyl(100,z,-100,-4,2.5).cut(a.cyl(100,z,-100.01,-3.99,1.6))) for i,z in enumerate([-3.,2.])]
    compare('sleeves_only_vs_R60_two_wrist_part_bound',c.material(sleeves,[dict(name='two_wrist_parts_R60_bound',shape=envelope)]),sr['sleeve_vs_wrist_carrier_rotation_bound'])
    containment=[]
    for item in survey['wrist_carrier_rotation_bound']['containment']:
        part=next(z for z in data[7] if z['name']==item['name']);sh=part['shape'].cut(envelope,tol=1e-6);v=sh.Volume() if sh.Solids() else 0.;containment.append(dict(name=item['name'],outside_mm3=v))
        if abs(v-item['material_outside_cylinder_mm3'])>1e-8:differences.append('wrist_bound_'+item['name'])
    clip=next(z['shape'] for z in hardware if z['name']=='wrist_external_feed_clip');cap=next(z['shape'] for z in hardware if z['name']=='wrist_edge_clip_front_cap');saddle=next(z['shape'] for z in data[7] if z['name']=='Palm_wrist_saddle');contacts=[]
    for item in sr['contacts']:
        pt=np.asarray(item['point_mm']);n=np.asarray(item['normal']);first=cap if item['id']=='front_face' else clip;ds=[sh.distance(cq.Vertex.makeVertex(*pt)) for sh in [first,saddle]];mat=[[sh.isInside(cq.Vector(*(pt+s*.05*n)),1e-6) for s in [-1,1]] for sh in [first,saddle]];ok=np.max(abs(np.asarray(ds)-item['distance_mm']))<1e-5 and mat==item['material'];contacts.append(dict(id=item['id'],distance_mm=ds,material=mat,matched=bool(ok)))
        if not ok:differences.append('contact_'+item['id'])
    clean=not git('status','--porcelain=v1');result=dict(completed=True,target_commit=args.commit,passed=not differences and guard() and clean,differences=differences,key_samples=native,geometry_comparisons=geometry,contacts=contacts,wrist_part_containment=containment,straight_insertion_start_end_outside_mm3=outside,manifest_files=len(man['files']),frozen_files=len(frozen),input_hash_guard=guard(),checkout_clean=clean,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),independent_audit=False,whole_hand_drive_approved=False,
        scope='33nativegroups:internal1+4selectedposesx4=17,6hardwareentry start/end12=29,2full straight-end insertion envelopes=31,rear-wrist four-proxy group1=32,sleeves-only rotational bound1=33.7source/STEP adaptive signatures abs1e-5(new5+2envelopes),3contacts,2actualwristpart containment/4insertion endpoints.Not remaining12poses/middle18assembly/render/wholeflexiblehousing/cordthreading/pullout/preload/stiffness/friction/tension/continuouswrist/fullhand.39palmpack follows hand,346forearmpack+64leads fixed;V9referencearm axes,not actual force-constrained hand-arm attachment approval.Inherited comparison limitations untouched.')
    p.write(args.output,result);assert result['passed'],differences
if __name__=='__main__':main()
