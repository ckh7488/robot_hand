"""Read saved failures and actual rack-nut bore/seat properties, without Boolean."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm076_survey',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,ROOT,HERE=m.n,m.np,m.ROOT,m.HERE
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.OSD import OSD_ThreadPool


def main():
    out=HERE/'reviews/survey.json';assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    q=n.p.read(ROOT/'cad/branched_module_assembly_v1/reviews/check.json');s=n.p.read(ROOT/'cad/branched_module_assembly_v1/reviews/selected_check.json');e=n.p.read(ROOT/'cad/branched_module_assembly_v1/reviews/evidence_selected.json');old=n.p.read(ROOT/'cad/branched_module_assembly_v1/reviews/survey.json')
    render=n.m.drive.module('_arm076_metadata',ROOT/'cad/ordered_roll_root_assembly_v1/render_evidence.py');metas,matrix=render.metadata();meta={z['name']:z for z in metas};source={};properties=[]
    for name in ['thumb_ip_actuator_rack_nut_1','thumb_ip_actuator_rack_nut_2','thumb_ip_actuator_adapter']:
        z=dict(meta[name])
        if 'step' not in z:
            rel='cad/modular_roots_v9/parts/'+name+'.step';z.update(step=rel,sha256_step=n.m.drive.sha(ROOT/rel))
        sh=n.g.original.transform(n.c.load([z],source)[0]['shape'],matrix(name));faces=[]
        for i,f in enumerate(sh.Faces()):
            a=BRepAdaptor_Surface(f.wrapped);kind=f.geomType();v=dict(index=i,type=kind,bbox=n.c.m.pack._bbox(f))
            if kind=='CYLINDER':
                c=a.Cylinder();v.update(radius_mm=c.Radius(),axis_point=c.Location().Coord(),axis_direction=c.Axis().Direction().Coord())
            elif kind=='PLANE':
                pl=a.Plane();direction=np.array(pl.Axis().Direction().Coord())
                if abs(direction[1])<.999:continue
                v.update(point=pl.Location().Coord(),normal=direction)
            else:continue
            faces.append(v)
        properties.append(dict(name=name,bbox=n.c.m.pack._bbox(sh),faces=faces))
    saved_common=[]
    for z in n.c.load(n.p.read(ROOT/e['common_scene'])['parts'],source):saved_common.append(dict(name=z['name'],volume_mm3=z['shape'].Volume(),solids=len(z['shape'].Solids()),bbox=n.c.m.pack._bbox(z['shape'])))
    proofs=m.inheritance();body=set(q['body43_names']);firstnut='thumb_ip_actuator_rack_nut_1';separated=body|{firstnut};initial=set(s['packet_final_present_names'])-separated
    assert len(separated)==44 and len(initial)==670 and initial==set(q['initial_world_present_names']) and not initial&set(q['mapped_special_packet']['names'])
    for f in [Path(__file__),HERE/'model.py',ROOT/'cad/branched_module_assembly_v1/reviews/check.json',ROOT/'cad/branched_module_assembly_v1/reviews/selected_check.json',ROOT/'cad/branched_module_assembly_v1/reviews/survey.json',ROOT/'cad/branched_module_assembly_v1/reviews/evidence_selected.json']:source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    n.a.save(out,dict(completed=True,source_sha256=source,actual_rack_nut_and_adapter_properties=properties,saved_prior_common_properties=saved_common,prior_common_queries=e['common_queries'],old_Boolean_repeated=0,independent_inherited_proofs=proofs,inherited_samples=162,inherited_native_groups_repeated=0,
        preserved_previous_world714_names=s['packet_final_present_names'],explicitly_separated_own44_names=sorted(separated),independent_world670_start_names=sorted(initial),mapped_special_packet=q['mapped_special_packet'],body43_names=q['body43_names'],body45_names=sorted(body|{f'thumb_ip_actuator_rack_nut_{i}' for i in [1,2]}),
        initial_carrier25_names=q['paths'][14]['present_names'],original_port0_kit_names=q['port0_kit7_names'],original_port1_kit_names=q['port1_kit7_names'],radial=q['radial'],normal=q['normal'],preserved_upper_M3_whole_cut=q['preserved_upper_M3_whole_cut'],preserved_M3_point_count=old['preserved_M3_point_count'],
        scope='Saved714 remains a separate partial result. Explicitly separate only ownbody43+firstnut1=44 to independent670;rear8/hand/arm retained,reverse disassembly and bench transport not tested. Reuse exact14+42+42+64 with no changed obstacles;all nut-preloaded port entries/worldbody45/newfasteners freshly tested. Read actualnut bore/captureplanes and savedcommon only;no old collision/inclusion/point/615PCD Boolean repeat.',input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items())))
    print('read actual nuts/adapter',len(properties),'oldcommon read only;reuse162;separate44=>670',flush=True)


if __name__=='__main__':main()
