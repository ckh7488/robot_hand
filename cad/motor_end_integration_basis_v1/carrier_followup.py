"""Locate the obstruction after the thumb-bridge hypothesis was rejected."""
import importlib.util
import sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm058_survey',Path(__file__).with_name('survey.py'))
s=importlib.util.module_from_spec(spec)
spec.loader.exec_module(s)
n,np,cq,ROOT,HERE=s.n,s.np,s.cq,s.ROOT,s.HERE


def calculate():
    raw,lookup,sh,G,paths,source,_=s.context()
    prior=n.p.read(HERE/'reviews/survey.json')
    source['cad/motor_end_integration_basis_v1/reviews/survey.json']=n.m.drive.sha(HERE/'reviews/survey.json')
    saved=n.c.load(n.p.read(ROOT/prior['scene'])['parts'],source)
    common=next(z['shape'] for z in saved if z['name']=='arm058_spool_carrier_common')
    carrier=sh['arm054_eight_bank_carrier']
    meta=lookup['thumb_ip_tail_clamp_1']
    tail=n.c.load([meta],source)[0]['shape']
    # Same explicit ARM057 custom-output clock, no motor command change.
    *_,delta,_,_,_,clock=s.p.metadata(raw)
    tail=n.g.original.transform(tail,np.array(clock)@np.array(delta[meta['name']]))
    R=s.a.d.around((0.,0.,0.),(0.,1.,0.),-45.)
    spoke=n.g.original.transform(s.a.box([34.,-162.,-17.],[75.4,-159.,-9.]),R)
    free=n.g.original.transform(s.a.box([38.,-162.,-17.],[66.5,-159.,-9.]),R)
    rows=[]
    queries=[]
    def q(name,kind,left,right):
        print(name,flush=True)
        got=left.intersect(right,tol=1e-6) if kind=='common' else left.cut(right,tol=1e-6)
        assert got.isValid(),name
        m=s.metrics(got)
        queries.append(dict(id=name,operation=kind,volume_mm3=m['global_values'][0],solids=m['solids'],bbox_mm=m['bbox_mm']))
        return got
    q('saved_spool_common__outside_middle_dip_minus13_spoke','cut',common,spoke)
    c=q('actual_clocked_tailclamp1__carrier','common',tail,carrier)
    rows.append(dict(name='arm058_tail_carrier_common',shape=c,body='forearm',evidence_only=True,color=[.98,.04,.03]))
    q('tail_common__outside_middle_dip_minus13_spoke','cut',c,spoke)
    cut=q('counterfactual_carrier_without_middle_dip_spoke_free_span','cut',carrier,free)
    islands=[]
    for i,solid in enumerate(sorted(cut.Solids(),key=lambda z:-z.Volume())):
        name='arm058_counterfactual_middle_dip_solid_'+str(i)
        rows.append(dict(name=name,shape=solid,body='forearm',evidence_only=True,color=[.3,.5,.75] if i==0 else [.95,.52,.08]))
        islands.append(dict(name=name,bbox_mm=n.c.m.pack._bbox(solid),volume_mm3=s.metrics(solid)['global_values'][0]))
    spoke_local=dict(module='middle_dip_actuator',row=1,bank=1,azimuth_deg=45.,tangent_mm=-13.,
        source_radial_box_mm=dict(lo=[34.,-162.,-17.],hi=[75.4,-159.,-9.]),
        M3_axis_world_mm=(R@np.array([71.,-162.,-13.,1.]))[:3],
        free_span_radial_mm=[38.,66.5],hub_outer_radius_mm=38.,retained_pad_radial_mm=[66.5,75.4])
    result=dict(completed=True,native_queries=queries,native_boolean_operations=4,operation_failures=[],
        source_sha256=source,identified_spoke=spoke_local,counterfactual_islands=islands,
        initial_thumb_bridge_hypothesis_rejected=True,
        initial_survey_required_changes_carrier_is_superseded=True,
        reason='All neutral spool-common material lies outside both thumb axial bridges and inside the neighboring middle_dip -13mm spoke. Tail boss contact is checked separately.',
        required_carrier_change='Preserve middle_dip M3 axis/pad and hub,reroute or replace its -13mm spoke support around the actual thumb output,with full neighboring packet checks. Do not cut thumb bridges;their removal disconnects two thumb seats and leaves the actual contact untouched.',
        free_span_cut_is_diagnostic_not_a_proposed_carrier=True,new_installed_geometry_rows=0,
        supplier_or_hardware_motion_approved=False,whole_assembly_or_strength_approved=False)
    return result,rows


def main():
    out=HERE/'reviews/carrier_followup.json'
    assert not out.exists()
    result,rows=calculate()
    result['scene']=n.a.a.prior.export(rows,HERE/'generated/carrier_diagnostic')
    result['source_sha256'][Path(__file__).relative_to(ROOT).as_posix()]=n.m.drive.sha(Path(__file__))
    result['input_hash_guard']=all(n.m.drive.sha(ROOT/f)==h for f,h in result['source_sha256'].items())
    n.a.save(out,result)
    print([(z['id'],z['volume_mm3'],z['solids']) for z in result['native_queries']],flush=True)


if __name__=='__main__': main()
