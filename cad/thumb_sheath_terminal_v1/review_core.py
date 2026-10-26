"""Clean exact-snapshot replay of an explicitly rejected local candidate."""
import sys,argparse,subprocess
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import check as k
m,g,c,p,np,cq,ROOT,HERE=k.m,k.g,k.c,k.p,k.np,k.cq,k.ROOT,k.HERE
from OCP.OSD import OSD_ThreadPool
metric=m.drive.module('_distal023_metrics',ROOT/'cad/thumb_tendon_route_v1/review_core.py')
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--commit',required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    def git(*xs):return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1')
    manifest=p.read(ROOT/'checkpoints/DISTAL023.manifest.json');frozen={}
    for key in ['geometry_sha256','code_sha256','report_sha256']:frozen.update(p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[key])
    def guard():return all(m.drive.sha(ROOT/z)==v['sha256'] for z,v in manifest['files'].items()) and all(m.drive.sha(ROOT/z)==h for z,h in frozen.items())
    assert guard();want=p.read(HERE/'reviews/static.json');assert want['completed'] and not want['static_candidate_clear']
    fresh,guide,design=m.build();data,pack=k.inputs(design);saved=c.load(p.read(ROOT/want['scene'])['parts']);housing=c.load(p.read(ROOT/want['housing_scene'])['parts'])
    source={z['name']:z for z in fresh+m.witnesses()};differences=[];geometry=[];native=[]
    for z in saved+housing:
        got=dict(metric.match(metric.signature(source[z['name']]['shape']),metric.signature(z['shape'])),name=z['name']);geometry.append(got)
        if not got['matched']:differences.append('shape_'+z['name'])
    def compare(label,got,expected):
        def mapping(r):return {(z['lead'],z['part']):z['material_intersection_mm3'] for z in r['overlaps']}
        a,b=mapping(got),mapping(expected);ok=a.keys()==b.keys() and all(abs(a[x]-b[x])<1e-5 for x in a.keys()&b.keys()) and got['operation_failures']==expected['operation_failures'] and got['pairs_in_scope']==expected['pairs_in_scope']
        native.append(dict(id=label,native=got,matched=ok));print(label,ok,flush=True)
        if not ok:differences.append(label)
    def combine(rows):return dict(overlaps=[q for z in rows for q in z['overlaps']],operation_failures=[q for z in rows for q in z['operation_failures']],pairs_in_scope=sum(z['pairs_in_scope'] for z in rows))
    compare('new_internal91',combine([c.material([z],saved[i+1:]) for i,z in enumerate(saved)]),combine(want['internal']))
    for label,keys in [('open',['hand','packaging','electrical','arm']),('parent_full',['hand','packaging','electrical','arm']),('parent_mid',['hand'])]:
        case=next(z for z in want['cases'] if z['id']==label);hand,v,f,neutral=k.hand_rows(data,guide,case['angles_deg'])
        posed=[dict(z,shape=v.relocate(z['shape'],f[z['body']],neutral[z['body']])) for z in saved+housing]
        for key in keys:compare(label+'_'+key,c.material(posed,dict(hand=hand,packaging=pack,electrical=data[6],arm=data[7])[key]),case['native'][key])
    clean=not git('status','--porcelain=v1');result=dict(completed=True,target_commit=args.commit,passed=not differences and guard() and clean,differences=differences,key_samples=native,geometry_comparisons=geometry,manifest_files=len(manifest['files']),frozen_files=len(frozen),input_hash_guard=guard(),checkout_clean=clean,OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),independent_audit=False,candidate_adopted=False,whole_hand_drive_approved=False,scope='14new/2short housing original-to-STEP adaptive metric signatures;new91pairs;open/full4groups and parent-mid hand:10nativegroups.Total8poses/render/assembly/contacts/long housing,forces and actual motion not rerun or approved.')
    p.write(args.output,result);assert result['passed'],differences
if __name__=='__main__':main()
