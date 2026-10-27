"""Exact-SHA bounded replay of the new integrated-support candidate, including failure."""
import argparse
import importlib.util
import subprocess
import sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm059_review_model',Path(__file__).with_name('model.py'))
a=importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)
n,np,d,ROOT,HERE=a.n,a.np,a.d,a.ROOT,a.HERE
measure=n.m.drive.module('_arm059_metric',ROOT/'cad/long_housing_route_v1/review_core.py')
follow=n.m.drive.module('_arm059_follow',HERE/'mechanical_followup.py')
assessment=n.m.drive.module('_arm059_assess',HERE/'assessment.py')
from OCP.OSD import OSD_ThreadPool


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--commit',required=True)
    ap.add_argument('--output',required=True,type=Path)
    args=ap.parse_args()
    def git(*xs): return subprocess.check_output(['git',*xs],cwd=ROOT,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW).strip()
    assert git('rev-parse','HEAD')==args.commit and not git('status','--porcelain=v1')
    assert not args.output.exists() and not args.output.resolve().is_relative_to(ROOT)
    manifest=n.p.read(ROOT/'checkpoints/ARM059R.manifest.json')
    frozen={}
    for k in ['geometry_sha256','code_sha256','report_sha256']: frozen.update(n.p.read(ROOT/'AUDIT_EVIDENCE_INDEX.json')[k])
    def guard(): return all(n.m.drive.sha(ROOT/f)==v['sha256'] for f,v in manifest['files'].items()) and all(n.m.drive.sha(ROOT/f)==v for f,v in frozen.items())
    assert guard() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    raw=n.p.read(HERE/'reviews/check.json')
    rows,membership,prior,delta,source=a.context()
    fresh,moved,fixed,design=a.build(rows,prior)
    shift=np.array(design['mechanical_translation_from_ARM057'])
    fresh_cords,routes=a.cords(prior,shift)
    saved=n.c.load(n.p.read(ROOT/raw['scene'])['parts'])
    cords=n.c.load(n.p.read(ROOT/raw['cord_scene'])['parts'])
    changed=saved+moved
    result=dict(completed=False,target_commit=args.commit,differences=[],native_samples=[],geometry_comparisons=[],attachment_samples=[],
        manifest_files=len(manifest['files']),frozen_files=len(frozen),OCC_pool_threads=1,
        current862_installation_replaced=False,proposal871_adopted=False,same_agent_not_independent_audit=True)
    def save(): n.a.save(args.output,result)
    def geometry(xs,ys):
        for x,y in zip(xs,ys):
            assert x['name']==y['name']
            got=dict(name=x['name'],**measure.match(measure.metrics(x['shape']),measure.metrics(y['shape'])))
            result['geometry_comparisons'].append(got)
            if not got['matched']: result['differences'].append('geometry_'+x['name'])
        save()
    geometry(fresh,saved)
    geometry(fresh_cords,cords)
    error=max(np.max(abs(np.array(design['mechanical_translation_from_ARM057'])-raw['design']['mechanical_translation_from_ARM057'])),
        max(np.max(abs(np.array(x['Q'])-y['Q'])) for x,y in zip(design['ports'],raw['design']['ports'])),
        abs(design['carrier']['dogleg_Y_mm'][0]-raw['design']['carrier']['dogleg_Y_mm'][0]))
    route_error=max(np.max(abs(np.array(x[k])-y[k])) for x,y in zip(routes,raw['local_routes']) for k in ['tangent_exit_world_mm','R5_entry_world_mm','R5_exit_world_mm','mouth_world_mm','shoulder_world_mm','local_cord_length_mm','new_piece_endpoint_gap_mm'])
    result.update(placement_contract_matched=bool(error<1e-10 and design['moved_names']==raw['design']['moved_names']),route_contract_matched=bool(route_error<1e-10),placement_max_error=float(error),route_max_error=float(route_error))
    if not result['placement_contract_matched']: result['differences'].append('placement')
    if not result['route_contract_matched']: result['differences'].append('routes')
    def compare(label,got):
        want=raw['native'][label]
        l,r=d.overlap_map(got),d.overlap_map(want)
        err=max([abs(l[k]['material_intersection_mm3']-r[k]['material_intersection_mm3']) for k in l.keys()&r.keys()]+[0.])
        ok=l.keys()==r.keys() and err<1e-5 and got['operation_failures']==want['operation_failures'] and got['pairs_in_scope']==want['pairs_in_scope']
        result['native_samples'].append(dict(id=label,native=got,matched=bool(ok),maximum_volume_difference_mm3=err))
        if not ok: result['differences'].append(label)
        save()
        print(label,bool(ok),flush=True)
    def internal(objects): return d.k.flat([n.c.material([z],objects[i+1:]) for i,z in enumerate(objects)])
    compare('changed20__fixed851_neutral',n.c.material(changed,fixed))
    compare('new4__moved16',n.c.material(saved,moved))
    compare('new4_internal',internal(saved))
    compare('new_cords12__hardware871',n.c.material(cords,changed+fixed))
    compare('new_cords12_internal',internal(cords))
    old={z['name']:z['shape'] for z in rows};new={z['name']:z['shape'] for z in saved}
    def query(label,kind,left,right):
        sh=left.cut(right,tol=1e-6) if kind=='cut' else left.intersect(right,tol=1e-6)
        volume=sh.Volume() if sh.Solids() else 0.
        want=next(z for z in raw['attachment_queries'] if z['id']==label)
        ok=sh.isValid() and len(sh.Solids())==want['solids'] and abs(volume-want['volume_mm3'])<1e-5
        result['attachment_samples'].append(dict(id=label,volume_mm3=volume,solids=len(sh.Solids()),matched=bool(ok)))
        if not ok: result['differences'].append(label)
        save()
    query('original_electrical_base__outside_integrated_base','cut',old['thumb_ip_actuator_wire_guide_base'],new[a.FRAME])
    survey=n.p.read(ROOT/'cad/motor_end_integration_basis_v1/reviews/survey.json')
    groove=n.g.original.transform(a.a.box([-3,-3.75,-69.75],[3,3.75,-67.35]),np.array(survey['electrical_frame_proposal']))
    query('integrated_base__protected_electrical_groove','common',new[a.FRAME],groove)
    for z in [-13.,13.]:
        pad=a.a.cy(3.,-127.,-126.,71.,z).cut(a.a.cy(1.7,-127.01,-125.99,71.,z))
        query('original_M3_pad_'+str(z)+'__outside_integrated_base','cut',pad,new[a.FRAME])
    R=d.around((0,0,0),(0,1,0),-45.)
    pad=n.g.original.transform(a.a.box([66.5,-162,-17],[75.4,-159,-9]),R).intersect(old['arm054_eight_bank_carrier'],tol=1e-6)
    query('original_middle_dip_M3_seat__outside_dogleg_carrier','cut',pad,new[a.CARRIER])
    output=[z for z in fixed if z['name'] in a.a.CLOCK_NAMES|{'thumb_ip_stock_output_horn','thumb_ip_stock_central_retainer'}]
    for state in raw['output_command_checks']:
        moving=[dict(z,shape=n.g.original.transform(z['shape'],np.array(state['transform']))) for z in output]
        compare('command_'+str(int(state['command_deg']))+'__output9__changed20',n.c.material(moving,changed))
    arm=n.p.read(ROOT/'cad/modular_roots_v9/design.json')['arm']
    groups={k:[z for z in fixed if membership.get(z['name'],'forearm')==k] for k in ['fixed','elbow','forearm','wrist','hand']}
    for state in raw['arm_motion']:
        F=d.transforms(state['angles_deg'],arm)
        assert max(np.max(abs(F[k]-state['transforms'][k])) for k in F)<1e-10
        moving=[dict(z,shape=n.g.original.transform(z['shape'],F['forearm'])) for z in changed+cords]
        for target in state['compared_frames']:
            obstacles=[dict(z,shape=n.g.original.transform(z['shape'],F[target])) for z in groups[target]]
            compare(state['id']+'__new20_local12__'+target,n.c.material(moving,obstacles))
    extra,fresh_common=follow.calculate(raw)
    wanted=n.p.read(HERE/'reviews/mechanical_followup.json')
    result['mechanical_followup_samples']=[]
    for x,y in zip(extra['native_queries'],wanted['native_queries']):
        ok=x['id']==y['id'] and x['solids']==y['solids'] and abs(x['volume_mm3']-y['volume_mm3'])<1e-5
        result['mechanical_followup_samples'].append(dict(id=x['id'],native=x,matched=bool(ok)))
        if not ok: result['differences'].append(x['id'])
    geometry(fresh_common,n.c.load(n.p.read(ROOT/wanted['scene'])['parts']))
    result['saved_assessment_matched']=assessment.evaluate(raw,prior)==n.p.read(HERE/'reviews/assessment.json')
    if not result['saved_assessment_matched']: result['differences'].append('assessment')
    result.update(completed=True,native_groups=len(result['native_samples']),native_matches=sum(z['matched'] for z in result['native_samples']),
        geometry_shapes=len(result['geometry_comparisons']),geometry_matches=sum(z['matched'] for z in result['geometry_comparisons']),
        attachment_matches=sum(z['matched'] for z in result['attachment_samples']),mechanical_followup_matches=sum(z['matched'] for z in result['mechanical_followup_samples']),
        input_hash_guard=guard(),checkout_clean=not git('status','--porcelain=v1'),
        replay_scope='31new native groups/14raw records;5attachment queries plus1seat-construct common;4focused mechanical followup queries;20 source-STEP metrics at adaptive1e-10/absolute1e-5(new4+local12+common4);16placement/twoactualR10R5routes/dogleg/assessment metadata. No all old geometry/surfaces trim Boolean equivalence,pixels/armvisualunits,oldARM05812/ARM05736 or oldseam/PCD replay. Not assembly/tool/preload/friction/strength/wholelongmaterial/wholearm/20torque/physical/exterior approval.')
    result['initial_core_passed']=not result['differences'] and result['input_hash_guard'] and result['checkout_clean']
    save()
    print('core',result['initial_core_passed'],'differences',result['differences'],flush=True)


if __name__=='__main__': main()
