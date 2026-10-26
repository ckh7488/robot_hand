"""Exact snapshot replay, including prototype failures and limited entry scope."""
import sys,subprocess,argparse
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import selected_check as k
m=k.m;g,c,p,np,cq,ROOT,HERE=m.g,m.c,m.p,m.np,m.cq,m.ROOT,m.HERE
metric=m.drive.module('_offset024_metrics',ROOT/'cad/thumb_tendon_route_v1/review_core.py')
from OCP.OSD import OSD_ThreadPool
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert not args.output.exists() and not args.output.resolve().is_relative_to(ROOT) and git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1')
    manifest=p.read(ROOT/'checkpoints/OFFSET024.manifest.json');frozen={}
    for key in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[key])
    def guard():return all(m.drive.sha(ROOT/z)==v['sha256'] for z,v in manifest['files'].items()) and all(m.drive.sha(ROOT/z)==h for z,h in frozen.items())
    assert guard();static=p.read(HERE/'reviews/static.json');entry=p.read(HERE/'reviews/keeper_entry.json');contact=p.read(HERE/'reviews/contacts.json');fresh,guide,design=m.build();data,pack=k.inputs(design)
    saved=c.load(p.read(ROOT/static['scene'])['parts']);housing=c.load(p.read(ROOT/static['housing_scene'])['parts']);extension=c.load(p.read(ROOT/static['cord_scene'])['parts']);lookup={z['name']:z for z in fresh+m.witnesses()+m.extension_cords()};geometry=[];differences=[];native=[]
    for z in saved+housing+extension:
        got=dict(metric.match(metric.signature(lookup[z['name']]['shape']),metric.signature(z['shape'])),name=z['name']);geometry.append(got)
        if not got['matched']:differences.append('shape_'+z['name'])
    def compare(label,got,want):
        def mapping(r):return {(z['lead'],z['part']):z['material_intersection_mm3'] for z in r['overlaps']}
        a,b=mapping(got),mapping(want);ok=a.keys()==b.keys() and all(abs(a[x]-b[x])<1e-5 for x in a.keys()&b.keys()) and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope']
        native.append(dict(id=label,native=got,matched=ok));print(label,ok,flush=True)
        if not ok:differences.append(label)
    def combine(rows):return dict(overlaps=[q for z in rows for q in z['overlaps']],operation_failures=[q for z in rows for q in z['operation_failures']],pairs_in_scope=sum(z['pairs_in_scope'] for z in rows))
    compare('new_internal91',combine([c.material([z],saved[j+1:]) for j,z in enumerate(saved)]),combine(static['internal']))
    compare('housing_internal',c.material(housing,saved+guide),static['housing_internal']);compare('extension_internal',c.material(extension,saved+guide+housing),static['extension_internal'])
    for label in ['open','parent_full']:
        case=next(z for z in static['cases'] if z['id']==label);hand,v,f,neutral=k.e.prior.hand_rows(data,guide,case['angles_deg']);moving=[dict(z,shape=v.relocate(z['shape'],f[z['body']],neutral[z['body']])) for z in saved+housing+extension]
        for key,ob in [('hand',hand),('packaging',pack),('electrical',data[6]),('arm',data[7])]:compare(label+'_'+key,c.material(moving,ob),case['native'][key])
    carrier=next(z for z in guide if z['name']=='paired_liner_carrier');old=p.read(ROOT/'cad/thumb_sheath_terminal_v1/reviews/static.json');oldshoe=c.load([z for z in p.read(ROOT/old['scene'])['parts'] if z['name']=='distal_housing_shoe_1'])[0]
    def one(report,lead,part):return dict(pairs_in_scope=1,operation_failures=[],overlaps=[z for z in report['overlaps'] if z['lead']==lead and z['part']==part])
    compare('original_coaxial_shoe_carrier',c.material([oldshoe],[carrier]),one(old['cases'][0]['native']['hand'],oldshoe['name'],carrier['name']))
    prototype=p.read(Path(__file__).resolve().parent/'reviews/static.json');oldcap=c.load([z for z in p.read(ROOT/prototype['scene'])['parts'] if z['name']=='distal_housing_keeper_1'])[0]
    for n in ['paired_liner_carrier','flanged_liner_1']:
        compare('prototype_'+n,c.material([oldcap],[next(z for z in guide if z['name']==n)]),one(prototype['cases'][0]['native']['hand'],oldcap['name'],n))
    case=prototype['cases'][-1];hand,v,f,neutral=k.e.prior.hand_rows(data,guide,case['angles_deg']);posed=dict(oldcap,shape=v.relocate(oldcap['shape'],f['thumb_mcp'],neutral['thumb_mcp']));root=next(z for z in hand if z['name']=='thumb_segment1_thenar_shell');compare('prototype_parent_root',c.material([posed],[root]),one(case['native']['hand'],oldcap['name'],root['name']))
    original=c.load(p.read(ROOT/'cad/thumb_guide_assembly_v2/generated/parts/scene.json')['parts']);installed=[z for z in original if z['name'] in entry['installed_original_guide']];hand,_,_,_=k.e.prior.hand_rows(data,installed,entry['hand_angles_deg']);ob=hand+pack+data[6]+data[7];cap=next(z for z in saved if z['name']=='distal_housing_keeper_1')
    for i in [0,3,4,5]:
        sample=entry['samples'][i];T=np.asarray(sample['transform_local']);W=g.original.WORLD@T@g.original.INV;compare('upper_keeper_entry_'+str(sample['distance_mm']),c.material([dict(cap,shape=g.original.transform(cap['shape'],W))],ob),sample['native'])
    compare('extension_existing_cords',c.material(extension,g.cord_pieces(np.zeros(20))),contact['extension_vs_existing_local_cords'])
    by={z['name']:z for z in original+saved+housing};contacts=[]
    for want in contact['contacts']:
        i=int(want['id'][-1]);row=g.original.layout()[i];Q=row['socket_transform'];normal=Q[:3,2]*row['side'] if want['id'].startswith(('keeper_to_carrier','split_shoe_face')) else Q[:3,1];normal=g.original.WORLD[:3,:3]@normal;point=np.asarray(want['point_world_mm']);a,b=by[want['a']]['shape'],by[want['b']]['shape'];classification=[[bool(sh.isInside(cq.Vector(*pt),1e-6)) for pt in [point-normal*.05,point+normal*.05]] for sh in [a,b]];dist=a.distance(b);ok=classification==want['classification'] and abs(dist-want['distance_mm'])<1e-5;contacts.append(dict(id=want['id'],classification=classification,distance_mm=dist,matched=ok))
        if not ok:differences.append(want['id'])
    Q=g.original.layout()[1]['socket_transform'];W=g.original.WORLD@Q;A=(W@np.r_[m.paths()[0]['a'],1])[:3];B=(g.original.WORLD@np.r_[g.original.layout()[1]['entry'],1])[:3];junction=dict(endpoint_error_mm=float(np.linalg.norm(A-B)),tangent_dot=float(np.dot(-(W[:3,:3]@m.paths()[0]['tangent']),W[:3,1])),extension_length_mm=sum(z['length_mm'] for z in m.paths()))
    if any(abs(junction[x]-contact['nominal_junction'][x])>1e-7 for x in junction):differences.append('junction')
    clean=not git('status','--porcelain=v1');result=dict(completed=True,target_commit=args.commit,passed=not differences and guard() and clean,differences=differences,key_samples=native,geometry_comparisons=geometry,contact_replay=contacts,junction_replay=junction,manifest_files=len(manifest['files']),frozen_files=len(frozen),input_hash_guard=guard(),checkout_clean=clean,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),independent_audit=False,whole_hand_drive_approved=False,whole_distal_assembly_approved=False,scope='20native groups,18adaptive original/STEP shapes,10contact witnesses and neutral nominal cord junction;open/full static and4of6upper-keeper entry samples;prototype failures.Not all8static/6entry/render,full hardware assembly,real housing retention,long route,force or continuous motion.')
    p.write(args.output,result);assert result['passed'],differences
if __name__=='__main__':main()
