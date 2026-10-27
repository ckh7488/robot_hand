"""Front-face axial pickups retain the original spoke range and whole clamp."""
import importlib.util
import sys
from pathlib import Path
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('_arm063_initial',Path(__file__).with_name('model.py'))
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
n,np,cq,d,ROOT,HERE=base.n,base.np,base.cq,base.d,base.ROOT,base.HERE
CARRIER,CAP,OLD=base.CARRIER,base.CAP,base.OLD
context,front_pad_inputs=base.context,base.front_pad_inputs


def build(before):
    geom,box=base.primitives();old=next(z['shape'] for z in before if z['name']==base.p.p.NAME)
    sh=old.cut(geom.prior.cyl_y(42.,48.5,-166.5),tol=1e-6)
    for sign in [-1.,1.]:
        sh=sh.cut(box([min(sign*24,sign*27),-200.5,-27],
            [max(sign*24,sign*27),-152.5,-24]),tol=1e-6)
    frame=box([-30,-155.5,-29],[30,-152.5,29]).cut(box([-26,-155.51,-25],[26,-152.49,25]))
    sh=sh.fuse(frame)
    for bank,stem in enumerate(geom.ORDER[1]):
        angle=geom.AZIMUTHS[bank]
        y=-121. if stem=='thumb_ip' else -162.
        for t in [-13.,13.]:
            if stem=='middle_dip' and t==-13.:continue
            # Recreate the original complete front spoke and original bore,
            # including the inner portion removed with the old clamp roots.
            spoke=geom.radial_box([34,y,t-4],[75.4,y+3,t+4],angle)
            pt=(d.around((0,0,0),(0,1,0),-angle)@np.array([71,y-1,t,1.]))[:3]
            spoke=spoke.cut(cq.Solid.makeCylinder(1.7,5.,cq.Vector(*pt),cq.Vector(0,1,0)))
            sh=sh.fuse(spoke)
            if stem=='thumb_ip':
                sh=sh.fuse(box([26,-158.5,t-4],[30,-118,t+4]))
                sh=sh.fuse(box([26,-121,t-4],[38,-118,t+4]))
            else:
                sh=sh.fuse(geom.radial_box([28,-158.5,t-4],[38,-152.5,t+4],angle))
                sh=sh.fuse(geom.radial_box([36,-162,t-4],[38,-152.5,t+4],angle))
    sh=sh.fuse(box([-27,-208.5,-28],[27,-200.5,-24]))
    for sign in [-1.,1.]:
        sh=sh.fuse(box([min(sign*10,sign*13),-208.5,-28],
            [max(sign*10,sign*13),-152.5,-24]))
    sh=sh.fuse(base.clamp_half(False)).clean()
    assert sh.isValid() and len(sh.Solids())==1,(sh.isValid(),len(sh.Solids()))
    row=dict(name=CARRIER,shape=sh,body='forearm',color=[.3,.5,.75],role='whole_recessed_clamp_roots_with_original_spokes_and_front_face_pickups')
    initial=n.p.read(HERE/'reviews/check.json')
    design=dict(initial['design'])
    design.update(front_normal_root_web_radial_mm=[28,38],front_root_web_Y_mm=[-158.5,-152.5],
        root_datum='Actual new removable cap front faceY=-158.5. Original spoke range restored;axial pickups connect it to front-face roots without widening into neighboring rotors.',
        original_front_spoke_radial_mm=[34,75.4],axial_pickup_radial_mm=[36,38],axial_pickup_Y_mm=[-162,-152.5],
        thumb_bridge_Y_mm=[-158.5,-118],thumb_top_bridge_X_mm=[26,38],
        first_candidate_code_results_exports_preserved=True,unchanged_initial_cap_fasteners5_and_entry4_reused=True,
        selected_hardware_scene_contains_changed_carrier_only=True)
    return row,design
