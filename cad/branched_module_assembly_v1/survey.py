"""Read explicit replacement,packet lists and independent bench proof only."""
import importlib.util,sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm075_survey',Path(__file__).with_name('model.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
n,ROOT,HERE=m.n,m.ROOT,m.HERE


def main():
    out=HERE/'reviews/survey.json';assert not out.exists()
    old=n.p.read(ROOT/'cad/special_thumb_module_service_v1/reviews/check.json');kit=n.p.read(ROOT/'cad/captured_adjuster_kit_v1/reviews/check.json');hold=n.p.read(ROOT/'cad/branched_terminal_support_v1/reviews/check.json');datum=n.p.read(ROOT/'cad/branched_terminal_support_v1/reviews/mounting_datum.json')
    proofs=m.inheritance();packet=old['original_special_packet'];mapped={k:(m.mapped(v) if k in ['names','body_names','mounting_names'] else v) for k,v in packet.items()}
    source={str(f.relative_to(ROOT).as_posix()):n.m.drive.sha(f) for f in [Path(__file__),HERE/'model.py',ROOT/'cad/special_thumb_module_service_v1/reviews/check.json',ROOT/'cad/captured_adjuster_kit_v1/reviews/check.json',ROOT/'cad/branched_terminal_support_v1/reviews/check.json',ROOT/'cad/branched_terminal_support_v1/reviews/mounting_datum.json']}
    n.a.save(out,dict(completed=True,source_sha256=source,original_special_packet=packet,mapped_special_packet=mapped,explicit_replacement={m.OLD:m.NAME},independent_inherited_proofs=proofs,inherited_samples=56,inherited_new_native_groups=0,
        guide_opening_axis=old['guide_opening_axis'],radial=old['radial'],normal=old['normal'],guide_actual_frame=old['guide_actual_frame'],original_port0_kit_names=kit['kit7_names'],original_port1_kit_names=['thumb_ip_barrel_adjuster_1','arm059_straight_jaw_1','motor_housing_clamp_1_jaw_1',*[f'motor_housing_clamp_1_{tag}_{i}' for tag in ['nut','bolt'] for i in [0,1]]],
        preserved_upper_M3_whole_cut=hold['attachment_queries'][-1],preserved_M3_point_count=datum['point_count'],preserved_all_points_inside_both=datum['all_selected_points_inside_both'],old_inclusion_Boolean_or_points_repeated=0,new_hardware_geometry=0,
        old_full49_or92_inherited_to_changed_support=False,reference_motor_output12_internal_assembly_new_approval=False,
        scope='Only motorbench14 and independent port0kit42 have the changed support absent from every moving/present set and identical source/registered placement chain. Reuse those exact independent paths without native rerun. All support-dependent carrier/electrical/jamnut/kitentry paths are new;port1 uses its actual original7 on an independent bench. Original48 mapping changes exactly1support name/geometry,other47 retained. UpperM3 cutfalse/216points contradiction read only.',input_hash_guard=all(n.m.drive.sha(ROOT/f)==h for f,h in source.items())))
    print('independent proof14+42=56,no oldfull49/92;explicitmapped48,holdread0Boolean',flush=True)


if __name__=='__main__':main()
