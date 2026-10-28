"""Read original four nuts, four screws and receiving pockets without Boolean."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm077_survey',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,np,ROOT,HERE=m.n,m.np,m.ROOT,m.HERE
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.OSD import OSD_ThreadPool


def properties(sh):
    faces=[]
    for i,f in enumerate(sh.Faces()):
        a=BRepAdaptor_Surface(f.wrapped);kind=f.geomType();z=dict(index=i,type=kind,bbox=n.c.m.pack._bbox(f))
        if kind=='CYLINDER':
            c=a.Cylinder();z.update(radius_mm=c.Radius(),axis_point=c.Location().Coord(),axis_direction=c.Axis().Direction().Coord())
        elif kind=='PLANE':
            pl=a.Plane();z.update(point=pl.Location().Coord(),normal=pl.Axis().Direction().Coord())
        else:continue
        faces.append(z)
    return dict(volume_mm3=sh.Volume(),center_mm=sh.Center().toTuple(),bbox=n.c.m.pack._bbox(sh),faces=faces)


def main():
    out=HERE/'reviews/survey.json';assert not out.exists() and OSD_ThreadPool.DefaultPool_s().NbThreads()==1
    q=n.p.read(ROOT/'cad/preloaded_rack_nut_module_v1/reviews/selected_check.json');ev=n.p.read(ROOT/'cad/preloaded_rack_nut_module_v1/reviews/evidence_selected.json');old=n.m.drive.module('_arm077_metadata',ROOT/'cad/ordered_roll_root_assembly_v1/render_evidence.py');metas,matrix=old.metadata();meta={z['name']:z for z in metas};source={};shapes={}
    for name in [m.BASE,'thumb_ip_actuator_base_screws','thumb_ip_actuator_adapter','thumb_ip_actuator_cradle']:
        z=dict(meta[name])
        if 'step' not in z:
            rel='cad/modular_roots_v9/parts/'+name+'.step';z.update(step=rel,sha256_step=n.m.drive.sha(ROOT/rel))
        shapes[name]=n.g.original.transform(n.c.load([z],source)[0]['shape'],matrix(name))
    assert len(shapes[m.BASE].Solids())==len(shapes['thumb_ip_actuator_base_screws'].Solids())==4
    props={name:dict(whole=properties(sh),solids=[dict(index=i,**properties(s)) for i,s in enumerate(sh.Solids())]) for name,sh in shapes.items()}
    saved=[dict(name=z['name'],**properties(z['shape'])) for z in n.c.load(n.p.read(ROOT/ev['common_scene'])['parts'],source)]
    special=set(q['mapped_special_packet']['names']);separate=special-{m.BASE};initial=set(q['packet_final_present_names'])-separate;assert len(separate)==47 and len(initial)==670
    for f in [Path(__file__),HERE/'model.py',ROOT/'cad/preloaded_rack_nut_module_v1/reviews/selected_check.json',ROOT/'cad/preloaded_rack_nut_module_v1/reviews/evidence_selected.json']:source[f.relative_to(ROOT).as_posix()]=n.m.drive.sha(f)
    n.a.save(out,dict(completed=True,source_sha256=source,actual_properties=props,saved_failed_common_properties=saved,old_common_Boolean_repeated=0,preserved_previous_world717_names=q['packet_final_present_names'],explicit_alternate_separated47_names=sorted(separate),alternate_world670_start_names=sorted(initial),current_world717_start_names=q['packet_final_present_names'],original_special_packet=q['original_special_packet'],mapped_special_packet=q['mapped_special_packet'],original16packets=q['original16packets'],base_nuts_compound_name=m.BASE,nut_solid_count=4,screw_solid_count=4,nut_total_volume_difference_mm3=abs(shapes[m.BASE].Volume()-sum(s.Volume() for s in shapes[m.BASE].Solids())),old_M3_inclusion_or_points_repeated=0,new_hardware_geometry=0,
        scope='Read each original nut and screw solid,adapter/cradle receiving faces and saved failedcommon only. Four solids remain original geometry,not four new installed rows. Test independent nut entries against all717 plus any completed originalnuts;when all4 identity positions recovered,restore the originalcompound row718. Alternateunit preassembly,if needed,uses explicit own47 separation to670 with rear8/hand/arm retained;reverse disassembly/transport unverified. Old106/162/9railfaces/7events/M3cut216points/615PCD not rerun.',input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items())))
    print('four originalnut/screw solids;adapter/cradle;oldcommon read,noBoolean',flush=True)


if __name__=='__main__':main()
