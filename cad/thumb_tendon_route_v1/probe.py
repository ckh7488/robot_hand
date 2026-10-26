"""Bounded route-shape/length study, without claiming guide installation."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import route_model as g


def main():
    output=g.HERE/'reviews/length_probe.json';assert not output.exists()
    target=next(z for z in g.p.read(g.ROOT/'cad/palm_motor_layout_v1/reviews/check2.json')['cases'] if z['id']=='pinch25')['angles_deg']
    report=dict(completed=False,geometric_candidates=[],whole_drive_complete=False,installed_guides=False,tensioner_installed=False)
    for entry in [-12.,-8.]:
        samples=[];errors=[]
        for fraction in [i/16 for i in range(17)]:
            q=(g.np.asarray(target)*fraction).tolist()
            try:
                rows,kin=g.coordinated(q,entry);samples.append(dict(fraction=fraction,angles_deg=q,kinematics=kin))
            except Exception as exc:errors.append(dict(fraction=fraction,error=repr(exc)))
        report['geometric_candidates'].append(dict(entry_parent_y_mm=entry,samples=samples,errors=errors))
        g.p.write(output,report)
    report['completed']=True;g.p.write(output,report)
    print([(z['entry_parent_y_mm'],len(z['samples']),z['errors']) for z in report['geometric_candidates']],flush=True)


if __name__=='__main__':main()
