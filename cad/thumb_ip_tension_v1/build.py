"""Build a separately saved adjustable fixture; never modify source hand CAD."""
from pathlib import Path
import argparse
import copy
import traceback
import numpy as np
from common import CAD,BENCH,SOURCE,BASE_SHA,Inputs,digest,write_json,rigid,bounds,source_rows,cord_segments,inherited_path

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--repo',type=Path,default=Path(__file__).resolve().parents[2])
    ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args(); repo=a.repo.resolve(); out=a.output.resolve()
    if out.exists(): raise FileExistsError('Preserve prior build outputs')
    (out/'parts').mkdir(parents=True)
    inputs=Inputs(repo)
    report=dict(schema='thumb_ip_tension_build.v1',completed=False,errors=[],source_sha256=inputs.hashes,
        baseline_sha=BASE_SHA,manufacturing_approved=False,latency_measured=False)
    try:
        import cadquery as cq
        from cadquery.occ_impl.shapes import setThreads
        setThreads(1)
        inputs.bind(__file__); inputs.bind(repo/CAD/'common.py')
        geo=inputs.module('_tension_geometry',inherited_path(inputs,SOURCE+'/geometry.py'))
        carrier=inputs.module('_tension_carrier',repo/CAD/'carrier.py')
        adjuster=inputs.module('_tension_adjuster',repo/CAD/'adjuster.py')
        mount_module=inputs.module('_tension_mounts',repo/CAD/'mounts.py')
        key_module=inputs.module('_tension_key',repo/CAD/'adjustment_key.py')
        features=inputs.module('_tension_manufacturing_features',repo/CAD/'manufacturing_features.py')
        routing=inputs.module('_tension_routing',inherited_path(inputs,SOURCE+'/routing.py'))
        joint=inputs.module('_tension_joint',inherited_path(inputs,SOURCE+'/joint_drive.py'))
        for p in sorted((repo/BENCH/'source_docs').rglob('*')):
            if p.is_file(): inherited_path(inputs,p.relative_to(repo).as_posix())
        for p in sorted((repo/SOURCE/'source_docs/anchor_fastener').rglob('*')):
            if p.is_file(): inherited_path(inputs,p.relative_to(repo).as_posix())
        rows,old_scene,old_build=source_rows(inputs)
        inherited=copy.deepcopy(rows)
        named={r['name']:r for r in rows}
        def native(name):
            row=named[name]
            return rigid(cq,cq.importers.importStep(str(inputs.resolve_part(row))).val(),row['neutral_matrix'])
        c=carrier.build(cq,native('drive_bench_baseplate'),features)
        cp=next(p for p in c['parts'] if p['name']=='drive_bench_baseplate')
        j=adjuster.build(cq,cp['shape'],features)
        mounts=mount_module.build(cq,geo,j['required_common_holes'])
        # The front interface belongs to the adjuster; replace this generated
        # copy once, retaining the old motor file as an immutable input.
        if 'modified_carrier_plate' in j: cp['shape']=j['modified_carrier_plate']
        common=native('bench_common_plate')
        holes=[]
        for module in (c,j):
            for spec in module.get('required_common_holes',[]):
                x,y=spec['xy']; radius=spec.get('diameter_mm',4.5)/2
                if any(abs(x-a)<1e-8 and abs(y-b)<1e-8 for a,b in holes): raise ValueError('Duplicate common hole')
                holes.append((x,y))
                common=geo.boolean(cq,'cut',common,cq.Solid.makeCylinder(radius,8.02,cq.Vector(x,y,-36.01),cq.Vector(0,0,1)))
        removed=set(c.get('remove_names',[]))|set(j.get('remove_names',[]))
        replaced=set(c.get('replace_names',[]))|set(j.get('replace_names',[]))|{'bench_common_plate'}
        if not (removed|replaced)<=set(named): raise ValueError('Unknown removed source part')
        updated_names=set()
        updates={}
        for module in (c,j):
            for name,update in module.get('row_updates',{}).items():
                if name in updated_names or name not in named or not set(update)<={'body','neutral_translation_delta_mm'}:
                    raise ValueError('Duplicate, unknown or shape-changing inherited row update')
                updated_names.add(name); updates[name]=copy.deepcopy(update)
                row=named[name]
                fields=copy.deepcopy(update)
                delta=fields.pop('neutral_translation_delta_mm',None)
                row.update(fields)
                if delta is not None:
                    if len(delta)!=3 or not np.isfinite(delta).all(): raise ValueError('Invalid row translation')
                    t=np.asarray(row['neutral_matrix']); t[:3,3]+=np.asarray(delta); row['neutral_matrix']=t.tolist()
        rows=[r for r in rows if r['name'] not in removed|replaced]
        added=[*c['parts'],*j['parts'],*mounts['parts'],dict(name='bench_common_plate',shape=common,body='fixed',
            role='modified_copy_common_plate_with_independent_guide_and_adjuster_mounts',color=[.24,.30,.36])]
        def export(part):
            name,shape=part['name'],part['shape']; facts=geo.require_solid(shape,name)
            step=out/'parts'/(name+'.step'); mesh=step.with_suffix('.stl')
            if step.exists(): raise ValueError('Duplicate generated name '+name)
            cq.exporters.export(shape,str(step)); cq.exporters.export(shape,str(mesh),tolerance=.035,angularTolerance=.12)
            row=dict(name=name,body=part.get('body','fixed'),role=part['role'],origin_group='tension_fixture',
                location='generated',step=step.relative_to(out).as_posix(),mesh=mesh.relative_to(out).as_posix(),
                sha256_step=digest(step),sha256_stl=digest(mesh),neutral_matrix=np.eye(4).tolist(),
                color=part.get('color',[.55,.6,.65]),volume_mm3=facts['volume_mm3'],solid_count=1,
                native_bbox_mm=bounds(shape),bbox_coordinates='Native STEP; apply neutral_matrix and body placement')
            for k in ('adjuster_axis','travel_mm_per_revolution'):
                if k in part: row[k]=part[k]
            return row
        rows.extend(export(p) for p in added)
        if len({r['name'] for r in rows})!=len(rows): raise ValueError('Duplicate physical part')
        replace_contacts=removed|set(c.get('remove_contact_parts',[]))|set(j.get('remove_contact_parts',[]))
        contacts=[r for r in old_scene['contacts'] if not ({r['a'],r['b']}&replace_contacts)]
        replace_assembly=removed|set(c.get('replace_assembly_parts',[]))|set(j.get('replace_assembly_parts',[]))
        assembly=[r for r in old_scene['assembly'] if r['part'] not in replace_assembly]
        contacts+=c.get('contacts',[])+j.get('contacts',[])+mounts['contacts']
        assembly+=c.get('assembly',[])+j.get('assembly',[])+mounts['assembly']
        # All table feet are attached before the complete motor/adjuster/shoe
        # subassembly is placed on the common plate. Check that later placement
        # separately; do not claim in-place access through the moving plate.
        table_stage={'bench_common_plate'}|{r['name'] for r in rows if r['name'].startswith('bench_table_foot_')}
        for d in assembly:
            if d['kind']=='lateral_jaw':
                d['stage']='parent_fixture_before_motor_module'
            if d['part'].startswith('bench_table_foot_'):
                d['stage_excluded']=sorted((set(r['name'] for r in rows)-table_stage)|set(d.get('stage_excluded',[])))
                d['stage']='table_feet_before_complete_tension_module'
        group_plan=inputs.read(repo/CAD/'assembly_groups.json')
        groups=[]
        for d in group_plan['declarations']:
            row=copy.deepcopy(d)
            row['moving_parts']=row.pop('group_parts'); row['installed_obstacles']=row.pop('included_obstacles'); row['absent_at_stage']=row.pop('stage_absent')
            groups.append(row)
        # Cap fasteners are fitted to the motor/post module before the parent
        # fixture, guide shoes or common plate. The complete module's subsequent
        # placement is a separate actual-parts path, not an exclusion waiver.
        cap_stage=set(groups[1]['installed_obstacles'])
        for d in assembly:
            if d['part'] in {'tension_adjuster_head_cap_screw_1','tension_adjuster_head_cap_screw_2'}:
                d['stage_excluded']=sorted((set(r['name'] for r in rows)-cap_stage)|set(d.get('stage_excluded',[])))
                d['stage']='cap_fasteners_before_shoes_common_plate_and_parent_fixture'
        scene=dict(schema='thumb_ip_tension_scene.v1',parts=rows,contacts=contacts,assembly=assembly,
            coordinates='IP-local mm; axis +X; D120 reference motor origin (-6,-120,0). s=120-D.',
            detached_coupon=True,hand_geometry_modified=False,full_hand_integration=False,
            source_bench_part_count=124,removed_source_parts=sorted(removed),replaced_source_parts=sorted(replaced),
            source_row_updates=updates,
            motor_rotation_per_ip_rotation=.77,comparison_distance_range_mm=[118.,122.],
            thread_zones_motor_frame=old_scene['thread_zones_motor_frame'],
            supplemental_checks=c.get('supplemental_checks',[])+j.get('supplemental_checks',[]),
            group_assembly=groups,
            central_horn_retainer_missing=True,preload_measured=False,individual_cord_microadjusters_complete=False)
        old=inputs.read(inherited_path(inputs,SOURCE+'/generated/build_report.json'))
        key_meta=key_module.metadata()
        key=export(dict(name=key_meta['name'],shape=key_module.build(cq),body=key_meta['body'],
            role=key_meta['role'],adjuster_axis=key_meta['adjuster_axis'],
            travel_mm_per_revolution=key_meta['travel_mm_per_revolution'],color=[.88,.66,.16]))
        key['metadata']=key_meta
        scene['assembly_tools']=[key]
        scene['machining_blanks']=[]
        for blank in j.get('machining_blanks',[]):
            scene['machining_blanks'].append(export(dict(blank,body='fixed',role='nonassembly_machining_reference')))
        scene['display_cords']={}
        for q in (0.,-15.):
            display=[]
            for cord in cord_segments(cq,routing,joint,old['motor_interfaces']['anchors'],q):
                display.append(export(dict(cord,name=f'display_q{q:g}_'+cord['name'],body='fixed',
                    role='nominal_undeformed_cord_display',color=[.9,.26,.12])))
            scene['display_cords'][str(q)]=display
        coverage=inputs.module('_tension_coverage',repo/CAD/'coverage.py')
        report['coverage']=coverage.validate(scene,inherited,old_scene)
        write_json(out/'scene.json',scene)
        report.update(completed=True,part_count=len(rows),new_saved_parts=len(added),
            preserved_source_parts=124-len(removed|replaced),source_row_updates=scene['source_row_updates'],
            removed_source_parts=sorted(removed),replaced_source_parts=sorted(replaced),
            carrier_spec=c['spec'],adjuster_spec=j['spec'],common_new_holes_xy_mm=holes,
            generated_sha256={p.relative_to(out).as_posix():digest(p) for p in sorted(out.rglob('*')) if p.is_file()},
            limitations=['Detached one-axis fixture; accepted hand geometry and whole-hand assembly are unchanged.',
                'Nominal supplier dimension envelopes, no helical thread or tolerance/strength approval.',
                'Central horn retainer, actual pretension and joint angle sensing remain unresolved.',
                'No powered test or measured robot-only 50/100ms result.'])
    except Exception as error:
        report['errors'].append(dict(error=repr(error),traceback=traceback.format_exc()))
    finally:
        report['input_hash_guard']=inputs.guard()
        if not report['input_hash_guard']['passed']: report['completed']=False
        write_json(out/'build_report.json',report)
    print({k:report[k] for k in ('completed','errors')},flush=True)
    return 0 if report['completed'] and not report['errors'] else 1

if __name__=='__main__': raise SystemExit(main())
