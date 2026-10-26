"""Read only the neutral arm references before choosing temporary terminations."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from model import cq,p,b,m,HERE,ROOT

def main():
    out=HERE/'reviews/source_survey.json'
    if out.exists():raise FileExistsError('Preserve source survey')
    rows=[];source={}
    for z in p.read(ROOT/'cad/form_candidate_v1/generated/scene.json')['arm_reference']:
        rel='cad/modular_roots_v9/parts/'+z['asset']+'.step';source[rel]=b.sha(ROOT/rel)
        shape=cq.importers.importStep(str(ROOT/rel)).val()
        rows.append(dict(name=z['name'],step=rel,bbox_mm=m.pack._bbox(shape)))
    p.write(out,dict(completed=True,source_sha256=source,neutral_arm=rows))
    print(rows,flush=True)

if __name__=='__main__':main()
