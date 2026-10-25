"""Build the one-joint CAD candidate without changing the accepted hand design.

Run explicitly with --output pointing to a new directory. Exported new parts
are in neutral world millimetres; unchanged parts remain hash-bound references.
The external motor bench is separate from the hand assembly and is not a claim
of a complete test fixture, whole-hand routing or measured response time.
"""
from pathlib import Path
import argparse,copy,hashlib,importlib.util,json,traceback,os


def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--repo',type=Path,default=Path(__file__).resolve().parents[2])
    ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args();repo=a.repo.resolve();out=a.output.resolve()
    if out.exists():raise FileExistsError('Use a new output directory')
    out.mkdir(parents=True);(out/'parts').mkdir();(out/'motor').mkdir()
    import cadquery as cq
    from cadquery.occ_impl.shapes import setThreads
    setThreads(1)
    base=repo/'cad/manufacturing_v2';here=repo/'cad/thumb_ip_drive_v1';hashes={}
    def bind(p,expected=None):
        p=Path(p).resolve()
        if not p.is_relative_to(repo):raise ValueError('Source outside repository')
        key=p.relative_to(repo).as_posix();h=digest(p)
        if expected is not None and h!=expected:raise ValueError('Stale source '+key)
        if key in hashes and hashes[key]!=h:raise ValueError('Changed source '+key)
        hashes[key]=h;return p
    def read(p):return json.loads(bind(p).read_text(encoding='utf-8-sig'))
    def module(n,p):
        s=importlib.util.spec_from_file_location(n,bind(p));m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
    report=dict(schema='thumb_ip_drive_build.v1',completed=False,errors=[],source_sha256=hashes,
                manufacturing_approved=False,latency_measured=False)
    def save():
        report['input_hash_guard']=dict(passed=all(digest(repo/k)==h for k,h in hashes.items()),start=dict(hashes),end={k:digest(repo/k) for k in hashes})
        if not report['input_hash_guard']['passed']:report['completed']=False
        (out/'build_report.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    try:
        bind(__file__)
        for p in sorted((here/'source_docs').rglob('*')):
            if p.is_file():bind(p)
        geometry=module('_tip_geom',here/'geometry.py');drive=module('_tip_drive',here/'joint_drive.py')
        routing=module('_tip_routing',here/'routing.py');passages=module('_tip_passages',here/'parent_passages.py')
        motor=module('_tip_motor',here/'motor_adapter.py')
        frames=module('_tip_frames',repo/'cad/gesture_hand_v3/fabrication_repairs.py')
        scene=read(base/'generated/scene.json');design=read(base/'generated/design.json')
        frame=frames._frame(design['thumb'],2);rows={r['name']:r for r in scene['hand']}
        child_name='thumb_segment3_tip_core';parent_name='thumb_segment2_segment_shell'
        def native(name):
            row=rows[name];return frames._local(cq.importers.importStep(str(bind(base/row['step'],row['sha256_step']))).val(),frame)
        joint=drive.build_joint(cq,native(child_name));parent=passages.build_parent(cq,native(parent_name),routing,geometry)
        if drive.PARAMETERS['anchor_angle_deg']!=routing.JOINT_ANCHOR_DEG:raise ValueError('Joint and routing anchor mismatch')
        added=joint['parts'];modified={child_name:joint['child'],parent_name:parent['parent']}
        colors={child_name:[.11,.58,.61],parent_name:rows[parent_name]['color'][:3]}
        hand=[]
        def export(name,shape,body,folder,color,role):
            facts=geometry.require_solid(shape,name)
            step=out/folder/(name+'.step');stl=out/folder/(name+'.stl')
            cq.exporters.export(shape,str(step));cq.exporters.export(shape,str(stl),tolerance=.035,angularTolerance=.12)
            b=shape.BoundingBox()
            return dict(name=name,body=body,role=role,location='generated',step=step.relative_to(out).as_posix(),mesh=stl.relative_to(out).as_posix(),
                sha256_step=digest(step),sha256_stl=digest(stl),color=list(color),mesh_units='mm',volume_mm3=facts['volume_mm3'],
                bbox_mm=[[b.xmin,b.ymin,b.zmin],[b.xmax,b.ymax,b.zmax]],solid_count=facts['solid_count'])
        for row in scene['hand']:
            name=row['name']
            if name in modified:
                hand.append(export(name,frames._world(modified[name],frame),row['body'],'parts',colors[name],'modified_existing_link'))
            else:
                r=copy.deepcopy(row);r['location']='repo'
                for key,hkey in [('step','sha256_step'),('mesh','sha256_stl')]:r[key]=bind(base/row[key],row[hkey]).relative_to(repo).as_posix()
                hand.append(r)
        for row in added:
            color=[.98,.48,.08] if row['name']=='thumb_ip_tendon_clamp' else [.67,.71,.75]
            hand.append(export(row['name'],frames._world(row['shape'],frame),row['body'],'parts',color,row['role']))
        # Motor parts use their own fixed bench coordinates, not the hand FK.
        mb=motor.build_motor_module(cq,repo);motor_rows=[]
        for k,h in mb['input_hashes'].items():bind(repo/k,h)
        for row in mb['parts']:
            shape=row['shape'];d=geometry.describe(shape)
            if not(d['valid'] and d['bounded'] and d['positive_solids']):raise ValueError('Invalid motor geometry')
            if len(shape.Solids())==1:
                saved=export(row['name'],shape,row['body'],'motor',row.get('color',[.28,.36,.47]),row['role'])
            else:
                # The unchanged vendor motor compound contains separate solids.
                step=out/'motor'/(row['name']+'.step');stl=step.with_suffix('.stl')
                cq.exporters.export(shape,str(step));cq.exporters.export(shape,str(stl),tolerance=.035,angularTolerance=.12)
                b=shape.BoundingBox()
                saved=dict(name=row['name'],body=row['body'],role=row['role'],location='generated',step=step.relative_to(out).as_posix(),mesh=stl.relative_to(out).as_posix(),sha256_step=digest(step),sha256_stl=digest(stl),color=row.get('color',[.28,.36,.47]),
                    mesh_units='mm',bbox_mm=[[b.xmin,b.ymin,b.zmin],[b.xmax,b.ymax,b.zmax]],**d)
            motor_rows.append(saved)
        result=dict(schema='thumb_ip_drive_scene.v1',hand=hand,motor_bench=motor_rows,
            motor_to_ip_local_translation_mm=[-6,-120,0],motor_rotation_axis='+X',motor_rotation_per_ip_rotation=.77,
            thumb_ip_frame=frame,whole_hand_tendon_routing_complete=False,bench_parent_fixture_complete=False)
        (out/'scene.json').write_text(json.dumps(result,indent=2)+'\n');(out/'design.json').write_text(json.dumps(design,indent=2)+'\n')
        report.update(completed=True,hand_part_count=len(hand),motor_part_rows=len(motor_rows),modified_names=sorted(modified),added_names=[r['name'] for r in added],
            unchanged_names=sorted(set(rows)-set(modified)),joint_parameters=joint['parameters'],parent_parameters=parent['parameters'],
            joint_operation_journal=joint['operation_journal'],parent_operation_journal=parent['operation_journal'],motor_interfaces=mb['interfaces'],
            limitations=joint['limitations']+parent['limitations']+mb['limitations'],generated_sha256={p.relative_to(out).as_posix():digest(p) for p in sorted(out.rglob('*')) if p.is_file()})
    except Exception as e:report['errors'].append(dict(error=repr(e),traceback=traceback.format_exc()))
    finally:save()
    print(json.dumps(dict(completed=report['completed'],errors=report['errors'])),flush=True)
    return 0 if report['completed'] and not report['errors'] else 1


if __name__=='__main__':raise SystemExit(main())
