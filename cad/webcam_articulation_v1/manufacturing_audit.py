"""Manufacturing screen of the saved CAD; no geometry is silently repaired.

All 120 components are included in interference checks. Separately exported
parts with common material are reported even if assigned to one simulator body.
The results distinguish printable solids from an assembled, retained mechanism.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time
import numpy as np
import cadquery as cq
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
from verify import frames, parent, move, bounds

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
OUT=HERE/'generated/manufacturing_audit'


def save(name,value):
    OUT.mkdir(parents=True,exist_ok=True)
    (OUT/name).write_text(json.dumps(value,indent=2),encoding='utf-8')


def load():
    design=json.loads((HERE/'design.json').read_text())
    scene=json.loads((HERE/'generated/scene.json').read_text())['hand']
    solids=cq.importers.importStep(str(HERE/'generated/hand_articulation_candidate.step')).val().Solids()
    assert len(solids)==len(scene)==120
    parts={}
    for row,solid in zip(scene,solids):
        assert abs(solid.Volume()-row['volume_mm3'])<max(.05,row['volume_mm3']*1e-5),row['name']
        assert np.max(np.abs(np.array(bounds(solid))-row['bbox_mm']))<.002,row['name']
        parts[row['name']]=solid
    hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in
        (HERE/'design.json',HERE/'generated/scene.json',HERE/'generated/hand_articulation_candidate.step',
         HERE/'build.py',HERE/'digits.py',HERE/'thumb_shell.py',HERE/'manufacturing_audit.py')}
    return design,scene,parts,hashes


def cylinders(shape):
    values=[]
    for face in shape.Faces():
        surface=BRepAdaptor_Surface(face.wrapped)
        if surface.GetType()==GeomAbs_Cylinder:
            c=surface.Cylinder();axis=c.Axis();p=axis.Location();d=axis.Direction()
            values.append(dict(radius_mm=c.Radius(),axis=[d.X(),d.Y(),d.Z()],point_mm=[p.X(),p.Y(),p.Z()]))
    return values


def mesh_topology(row):
    path=HERE/row['mesh'];raw=path.read_bytes()
    assert hashlib.sha256(raw).hexdigest()==row['sha256_stl']
    dtype=np.dtype([('normal','<f4',(3,)),('v','<f4',(3,3)),('attribute','<u2')])
    triangles=np.frombuffer(raw[84:],dtype=dtype)['v'].astype(float)
    assert len(triangles)==int.from_bytes(raw[80:84],'little')
    # Exact exported float coordinates: no healing, welding tolerance or hole fill.
    vertices,index=np.unique(triangles.reshape(-1,3),axis=0,return_inverse=True)
    index=index.reshape(-1,3)
    edges=np.concatenate((index[:,[0,1]],index[:,[1,2]],index[:,[2,0]]))
    _,counts=np.unique(np.sort(edges,axis=1),axis=0,return_counts=True)
    area2=np.linalg.norm(np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]),axis=1)
    return dict(name=row['name'],triangles=len(triangles),vertices=len(vertices),
        boundary_edges=int(np.sum(counts==1)),nonmanifold_edges=int(np.sum(counts>2)),
        degenerate_triangles=int(np.sum(area2<1e-10)),coordinate_welding='exact float32 coordinates')


def overlap_pairs(parts,angles,design,include_same_body=True):
    neutral=frames(design,np.zeros(20));current=frames(design,angles)
    moved={name:move(shape,current[parent(name)],neutral[parent(name)]) for name,shape in parts.items()}
    boxes={name:bounds(shape) for name,shape in moved.items()};hits=[];candidates=0
    names=list(parts)
    for i,a in enumerate(names):
        for b in names[i+1:]:
            same=parent(a)==parent(b)
            if same and not include_same_body:continue
            if np.any(np.minimum(boxes[a][1],boxes[b][1])-np.maximum(boxes[a][0],boxes[b][0])<=1e-6):continue
            candidates+=1
            common=moved[a].intersect(moved[b],tol=1e-6)
            volume=common.Volume()
            if volume>.02:
                hits.append(dict(a=a,b=b,volume_mm3=volume,same_simulator_body=same,
                    center_mm=list(common.Center().toTuple())))
    return dict(angles_deg=list(angles),broad_phase_candidates=candidates,overlaps=hits)


def geometry(design,scene,parts,hashes):
    report=dict(input_sha256=hashes,completed=False,scope='All 120 separate hand components; no pair exemptions',cases={})
    report['parts']=[dict(name=n,valid=s.isValid(),solid_count=len(s.Solids()),volume_mm3=s.Volume()) for n,s in parts.items()]
    report['mesh_topology']=[mesh_topology(row) for row in scene]
    save('assembly.json',report)
    poses={'open':np.zeros(20),'fist':np.array(design['fist_deg'])}
    counter=np.zeros(20);counter[0]=-22
    poses['index_spread_minus22_others_open']=counter
    for label,angles in poses.items():
        t=time.perf_counter();case=overlap_pairs(parts,angles,design)
        report['cases'][label]=case;save('assembly.json',report)
        print(label,'overlaps',len(case['overlaps']),'seconds',round(time.perf_counter()-t,1),flush=True)
        for hit in case['overlaps']:print(json.dumps(hit),flush=True)
    report['completed']=True
    save('assembly.json',report)


def interfaces(design,scene,parts,hashes):
    report=dict(input_sha256=hashes,roots=[],attachments=[],hinges=[],cover_fasteners=[])
    palm_names=[n for n in parts if n.startswith(('Palm_','Cover_post_'))]
    palm=cq.Compound.makeCompound([parts[n] for n in palm_names])
    for finger in ('index','middle','ring','little','thumb'):
        housing=parts[finger+'_spread_housing'];ring=parts[finger+'_spread_ring']
        row=dict(finger=finger,housing_to_palm_gap_mm=housing.distance(palm),
            housing_palm_overlap_mm3=housing.intersect(palm,tol=1e-6).Volume(),forks=[])
        for side in (-13,13):
            fork=parts[f'{finger}_root_fork_{side}']
            row['forks'].append(dict(side=side,ring_to_fork_gap_mm=ring.distance(fork),
                ring_fork_overlap_mm3=ring.intersect(fork,tol=1e-6).Volume()))
        report['roots'].append(row)
        for segment in (1,2,3):
            name=f'{finger}_segment{segment}_proximal_bushing'
            bushing=parts[name]
            report['hinges'].append(dict(finger=finger,segment=segment,
                pin_cylinders=cylinders(parts[f'{finger}_joint_pin_{segment-1}']),
                proximal_bushing_cylinders=cylinders(bushing),
                shaft_nominal_diameter_mm=4,bushing_id_mm=4.4,bushing_od_mm=6.7,housing_bore_mm=6.8,
                diametral_shaft_clearance_mm=.4,diametral_bushing_housing_clearance_mm=.1,
                positive_pin_retention_defined=False,bushing_retention_defined=False))
        for segment in (1,2,3):
            base=f'{finger}_segment{segment}_'
            body_name=base+('thenar_shell' if finger=='thumb' and segment==1 else 'tip_core' if segment==3 else 'segment_shell')
            accessory=base+('rounded_tip_cap' if segment==3 else 'dark_faceplate')
            if accessory in parts:
                report['attachments'].append(dict(part=accessory,host=body_name,
                    gap_mm=parts[accessory].distance(parts[body_name]),
                    overlap_mm3=parts[accessory].intersect(parts[body_name],tol=1e-6).Volume(),
                    fastening_definition_present=False))
    for name,shape in parts.items():
        if name.startswith('Cover_screw_'):
            box=bounds(shape)
            report['cover_fasteners'].append(dict(name=name,z_bounds_mm=[box[0][2],box[1][2]],
                cover_z_bounds_mm=[10.5,14],modeled_shank_length_mm=0,thread_or_nut_defined=False))
    drive=json.loads((ROOT/'cad/form_drive_v1/generated/dynamics_contract.json').read_text())
    report['drive_integration']=dict(current_installed_actuators=design['actuator_packaging']['installed_actuators_in_this_cad'],
        separate_drive_source=list(drive['source_sha256']),
        separate_drive_thumb=drive['design']['thumb'],current_thumb=design['thumb'],
        separate_drive_ranges={j['name']:j['range_deg'] for j in drive['joints']},
        current_fist_deg=design['fist_deg'],transmission_status=drive['transmission']['status'],
        limitations=drive['limitations'])
    save('interfaces.json',report)
    print('Interfaces saved',flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage',choices=('geometry','interfaces','all'),default='all',nargs='?')
    args=parser.parse_args();design,scene,parts,hashes=load()
    if args.stage in ('geometry','all'):geometry(design,scene,parts,hashes)
    if args.stage in ('interfaces','all'):interfaces(design,scene,parts,hashes)


if __name__=='__main__':main()
