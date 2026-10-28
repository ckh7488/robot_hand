"""Preserve original components while closing the electrical guide in the world."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('_arm079_prior',ROOT/'cad/front_packet_restoration_v1/model.py')
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
n,np,d=a.n,a.np,a.d
OLD,NAME=a.OLD,a.NAME;MODULE='middle_dip_actuator';ROOT_PART='arm065_shared_output_beam_rack_root'
translate,moved,bbox,project_bounds,outside_start,inward_steps,bench_vectors,corners,enrich_vectors,replay_samples=a.translate,a.moved,a.bbox,a.project_bounds,a.outside_start,a.inward_steps,a.bench_vectors,a.corners,a.enrich_vectors,a.replay_samples


def context():
    rows,membership,source,packets=a.context()
    for f in [Path(__file__),ROOT/'cad/front_packet_restoration_v1/reviews/check.json',ROOT/'cad/front_packet_restoration_v1/reviews/survey.json',ROOT/'cad/front_packet_restoration_v1/reviews/evidence.json']:
        source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    return rows,membership,source,packets


def inheritance(lookup):
    q=n.p.read(ROOT/'cad/front_packet_restoration_v1/reviews/check.json');paths=q['paths'][:6];name=MODULE
    core={name+s for s in ['_motor','_rotor','_body_screws','_spool']};unit=core|{name+'_cradle',name+'_base_screws',name+'_plug_outer'}
    expected=[({name+'_base_screws'},{name+'_cradle'}),(core,{name+'_cradle',name+'_base_screws'}),({name+'_wire_guide_base'},{name+'_adapter'}),({name+'_guide_spacer_1'},{name+'_adapter',name+'_wire_guide_base'}),({name+'_guide_spacer_2'},{name+'_adapter',name+'_wire_guide_base',name+'_guide_spacer_1'}),(unit,{name+'_adapter',name+'_wire_guide_base',name+'_guide_spacer_1',name+'_guide_spacer_2'})]
    assert len(paths)==6 and sum(len(p['samples']) for p in paths)==69
    for p,(moving,fixed) in zip(paths,expected):
        assert p['completed'] and set(p['moving_names'])==moving and set(p['present_names'])==fixed
        assert all(not q['native'][s['id']]['overlaps'] and not q['native'][s['id']]['operation_failures'] for s in p['samples'])
        assert np.max(abs(np.array(p['samples'][-1]['transform'])-np.eye(4)))<1e-10
    assert all(n.m.drive.sha(ROOT/f)==h for f,h in q['source_sha256'].items())
    names=set().union(*(moving|fixed for moving,fixed in expected));assert len(names)==11
    return dict(source_report='cad/front_packet_restoration_v1/reviews/check.json',paths=paths,sample_scope=69,new_native_groups=0,all_inputs_registered_geometry_hashes_unchanged=True,exact_moving_and_present_sets_unchanged=True,carrier11_names=sorted(names),registered_bounds={k:n.c.m.pack._bbox(lookup[k]['shape']) for k in sorted(names)})
