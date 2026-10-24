"""Rebuild approved exterior solids without executing its exporter or changing it.

The approved module has top-level export side effects. This loader executes only
its shape-construction statements and captures its add() calls. Result volumes
must match the approved scene before mass properties can be used.
"""
from pathlib import Path
import ast, hashlib, json, math, sys
import cadquery as cq
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib

ROOT=Path(__file__).resolve().parents[2]
FORM=ROOT/'cad/form_candidate_v1'

def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def rebuild_exterior():
    source=FORM/'build.py'
    reference=json.loads((FORM/'generated/geometry_review.json').read_text(encoding='utf-8'))
    for name,value in reference['input_sha256'].items():
        if digest(FORM/name)!=value:
            raise ValueError('Approved form source has changed: '+name)
    sys.path.insert(0,str(FORM))
    from lightweight_digits import build_segment
    from lightweight_thumb import build_thumb_metacarpal
    parts=[]
    def capture(name,shape,color,role,pos=(0,0,0),yaw=0):
        if yaw:shape=shape.rotate((0,0,0),(0,0,1),yaw)
        shape=shape.translate(pos)
        if not shape.isValid() or len(shape.Solids())!=1:
            raise ValueError('Exterior part invalid: '+name)
        parts.append(dict(name=name,shape=shape,color=color,role=role))
    namespace=dict(cq=cq,math=math,add=capture,build_segment=build_segment,
        build_thumb_metacarpal=build_thumb_metacarpal,
        C=json.loads((FORM/'design.json').read_text(encoding='utf-8')))
    tree=ast.parse(source.read_text(encoding='utf-8'))
    selected=[];constructing=False
    colors={'BLUE','DARK','TEAL','WHITE','METAL'}
    for node in tree.body:
        if isinstance(node,ast.Assign) and any(isinstance(x,ast.Name) and x.id=='outline' for x in node.targets):
            constructing=True
        if constructing and isinstance(node,ast.Expr) and isinstance(node.value,ast.Call) and isinstance(node.value.func,ast.Name) and node.value.func.id=='print':
            break
        if isinstance(node,ast.FunctionDef):
            if node.name!='add':selected.append(node)
        elif constructing:
            selected.append(node)
        elif isinstance(node,ast.Assign) and any(isinstance(x,ast.Name) and x.id in colors for x in node.targets):
            selected.append(node)
    exec(compile(ast.Module(body=selected,type_ignores=[]),str(source),'exec'),namespace)
    scene=json.loads((FORM/'generated/scene.json').read_text(encoding='utf-8'))
    old={p['name']:p for p in scene['hand']}
    if set(old)!={p['name'] for p in parts}:raise ValueError('Exterior part set changed')
    max_volume_error=max(abs(p['shape'].Volume()-old[p['name']]['volume_mm3']) for p in parts)
    if max_volume_error>1e-5:raise ValueError('Exterior volume mismatch: '+str(max_volume_error))
    return parts,dict(part_count=len(parts),max_volume_difference_mm3=max_volume_error,
        approved_source_sha256=reference['input_sha256'],scope='Same approved exterior solids, no geometry edits')

def mass_properties(shape,density_kg_per_mm3):
    g=GProp_GProps();BRepGProp.VolumeProperties_s(shape.wrapped,g)
    volume=g.Mass();c=g.CentreOfMass();I=g.MatrixOfInertia()
    # Unit-density integration is mm^5; density makes kg*mm^2, then 1e-6 gives SI.
    tensor=[[I.Value(i+1,j+1)*density_kg_per_mm3*1e-6 for j in range(3)] for i in range(3)]
    box=Bnd_Box();BRepBndLib.AddOptimal_s(shape.wrapped,box,False,False)
    ext=box.Get()
    return dict(volume_mm3=volume,mass_kg=volume*density_kg_per_mm3,
        com_world_mm=[c.X(),c.Y(),c.Z()],inertia_com_world_kg_m2=tensor,
        bbox_mm=[list(ext[:3]),list(ext[3:])],density_kg_per_mm3=density_kg_per_mm3)

def material_density(part):
    name=part['name']
    if 'joint_pin' in name or name.startswith('Cover_screw'):
        return 7.85e-6,'steel_assumption'
    # A fully dense polymer assumption is conservative for a deliberately hollow
    # solid, but is not a prediction of a specific printed infill or material.
    return 1.25e-6,'polymer_assumption'

def export_exterior_properties(output):
    parts,check=rebuild_exterior();rows=[]
    for p in parts:
        rho,material=material_density(p)
        rows.append(dict(name=p['name'],role=p['role'],material=material,
                         **mass_properties(p['shape'],rho)))
    report=dict(schema_version='form_drive_v1.exterior_mass.v1',parts=rows,
        total_exterior_mass_kg=sum(p['mass_kg'] for p in rows),rebuild_check=check,
        limitations=['Density assumptions, not measured weights.',
                     'Only approved exterior solids: motors, drive pulleys, tendons, bearings, wiring and missing fasteners are excluded.',
                     'Part volume sums do not prove absence of overlaps or physical assembly.'])
    Path(output).parent.mkdir(parents=True,exist_ok=True)
    Path(output).write_text(json.dumps(report,indent=2),encoding='utf-8')
    return report

if __name__=='__main__':
    report=export_exterior_properties(Path(__file__).parent/'generated/exterior_mass_inertia.json')
    print(json.dumps({k:v for k,v in report.items() if k!='parts'}),flush=True)
