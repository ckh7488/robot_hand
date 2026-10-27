"""Read saved actual common material and the real guide/terminal directions."""
import sys
from pathlib import Path
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'cad/elastic_housing_link_v1'))
import arc_rod as prior
a,m,g,c,p,np,cq=prior.a,prior.m,prior.g,prior.c,prior.p,prior.np,prior.cq
def main():
    r=p.read(ROOT/'cad/elastic_housing_link_v1/reviews/check.json');d=p.read(ROOT/'cad/elastic_housing_link_v1/reviews/details.json');source={};common=c.load(p.read(ROOT/d['common_scene'])['parts'],source);locations=[]
    for z in common:locations.append(dict(name=z['name'],bbox_mm=c.m.pack._bbox(z['shape']),center_mm=z['shape'].Center().toTuple(),default_volume_mm3=z['shape'].Volume()))
    arcs=[]
    for s in r['native_cases'][0]['solutions']:
        for side,indices in [('upstream_arcs',[9,10,11]),('downstream_arcs',[3,4,5,6])]:
            for j in indices:arcs.append(dict(line=s['physical_line'],span=side,arc_index=j,**s[side][j]))
    source['cad/elastic_housing_link_v1/reviews/check.json']=m.drive.sha(ROOT/'cad/elastic_housing_link_v1/reviews/check.json')
    p.write(HERE/'reviews/survey.json',dict(completed=True,source_sha256=source,actual_common_locations=locations,nearby_arc_geometry=arcs,new_native_interference_groups=0,source_scope='Saved actual intersections and arc-end geometry only;not rerun of old six groups,not actual installed flexible routing.'))
    print(locations,flush=True)
if __name__=='__main__':main()
