"""Export valid positive-intersection witnesses and retain a failed Boolean retry."""
import argparse,hashlib,importlib.util,json,math
from pathlib import Path
import cadquery as cq
import numpy as np

HERE=Path(__file__).resolve().parent
def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--audit',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    root=HERE.parents[1];source=read(a.audit)
    for k,h in source['source_sha256'].items():
        if sha(root/k)!=h:raise ValueError('Changed source '+k)
    p=root/'cad/articulation_drive_v2/verify.py';s=importlib.util.spec_from_file_location('_witness_fk',p);v=importlib.util.module_from_spec(s);s.loader.exec_module(v)
    design=source['requests']['design'];neutral=v.frames(design,np.zeros(20));scene=read(root/'cad/manufacturing_v2/generated/scene.json');rows={r['name']:r for r in scene['hand']};loaded={}
    def moved_pair(c,pair):
        current=v.frames(design,c['angles_deg']);answer=[]
        for name in [pair['a'],pair['b']]:
            if name not in loaded:loaded[name]=cq.importers.importStep(str(root/'cad/manufacturing_v2'/rows[name]['step'])).val()
            answer.append(v.relocate(loaded[name],current[rows[name]['body']],neutral[rows[name]['body']]))
        return answer
    a.output.mkdir(parents=True);witnesses=[];retry=[]
    wanted=[d+'_ordinary_extension_mcp' for d in ['index','middle','ring','little']]+['fan_1','index_pip_+135','thumb_mcp_+90']
    cases={c['id']:c for c in source['cases']}
    for ident in wanted:
        c=cases[ident];pair=max(c['result']['collision_details'],key=lambda r:r['material_intersection_mm3']);one,two=moved_pair(c,pair)
        common=one.intersect(two,tol=1e-6);volume=float(common.Volume());valid=common.isValid()
        if not valid or not common.Solids() or any(s.Volume()<=0 for s in common.Solids()) or abs(volume-pair['material_intersection_mm3'])>1e-5:raise ValueError('Invalid positive witness '+ident)
        path=a.output/(ident+'_overlap_only.step');cq.exporters.export(common,str(path))
        saved=cq.importers.importStep(str(path)).val()
        # STEP export of trimmed surfaces perturbs volume slightly. This export
        # comparison budget is 200x smaller than the .02mm3 collision threshold;
        # it does not change collision classification or the Boolean tolerance.
        roundtrip_error=abs(saved.Volume()-volume)
        if not saved.isValid() or roundtrip_error>1e-4:raise ValueError('Invalid witness round trip')
        witnesses.append(dict(id=ident,a=pair['a'],b=pair['b'],angles_deg=c['angles_deg'],valid=True,solids=len(common.Solids()),overlap_mm3=volume,
            path=path.name,sha256=sha(path),roundtrip_absolute_volume_error_mm3=roundtrip_error,roundtrip_limit_mm3=1e-4,
            manufactured_part=False,meaning='Intersection material only; visualization/diagnostic witness, NOT a replacement part'))
    for c in source['cases']:
        for pair in c['result']['operation_failures']:
            one,two=moved_pair(c,pair);attempts=[]
            for tol in [1e-6,1e-7,1e-8,0.]:
                try:
                    common=one.intersect(two,tol=tol);attempts.append(dict(tolerance_mm=tol,volume_mm3=float(common.Volume()),valid=common.isValid(),solids=len(common.Solids())))
                except Exception as e:attempts.append(dict(tolerance_mm=tol,error=repr(e)))
            retry.append(dict(case=c['id'],a=pair['a'],b=pair['b'],attempts=attempts,
                status='Original operation failure retained as unresolved; tolerance retries are diagnostics only and do not approve this pose'))
    (a.output/'witness_report.json').write_text(json.dumps(dict(completed=True,audit_sha256=sha(a.audit),script_sha256=sha(__file__),witnesses=witnesses,boolean_retries=retry),indent=2),encoding='utf-8')
if __name__=='__main__':main()
