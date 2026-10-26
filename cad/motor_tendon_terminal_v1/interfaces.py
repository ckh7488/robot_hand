"""Additional lower-face support evidence and explicit unconnected endpoints."""
import sys,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import terminal_selected as model
g,c,p,np,cq,HERE,ROOT=model.g,model.c,model.p,model.np,model.cq,model.HERE,model.ROOT
def main():
    out=HERE/'reviews/interfaces.json';assert not out.exists();static=p.read(HERE/'reviews/check.json');source={}
    parts=c.load(p.read(ROOT/static['scene'])['parts'],source);frame=next(z['shape'] for z in parts if 'terminal_frame' in z['name'])
    meta=p.read(ROOT/'cad/motor_spool_coupling_v1/selected/generated/scene.json')['retained_parts'];heads=c.load([z for z in meta if z['name'] in [model.NAME+'_rack_screw_1',model.NAME+'_rack_screw_2']],source)
    headcontacts=[]
    for i,(xy,head) in enumerate(zip(static['design']['mounting_axes_xy_mm'],heads),1):
        for k,a in enumerate([0,math.pi/2,math.pi,3*math.pi/2]):
            point=np.array([xy[0]+2.2*math.cos(a),xy[1]+2.2*math.sin(a),-82.]);above=point+[0,0,.02];below=point-[0,0,.02]
            headcontacts.append(dict(id='M3_head_'+str(i)+'_'+str(k),part_b=head['name'],distance_mm=frame.distance(head['shape']),point_mm=point.tolist(),axis=[0,0,1],a_above=bool(frame.isInside(cq.Vector(*above),1e-6)),a_below=bool(frame.isInside(cq.Vector(*below),1e-6)),b_above=bool(head['shape'].isInside(cq.Vector(*above),1e-6)),b_below=bool(head['shape'].isInside(cq.Vector(*below),1e-6))))
    ports=[]
    for port,distal in zip(model.channels(),g.original.layout()):
        point=(g.original.WORLD@np.r_[distal['entry'],1.])[:3];seat=np.r_[port['xy'],-158.]
        ports.append(dict(cord=port['cord'],motor_housing_seat_mm=seat.tolist(),motor_housing_mouth_mm=[*port['xy'],-163.],departure_direction=[0,0,-1],housing_OD_ID_candidate_mm=[3.,1.3],
            distal_cord_guide_inlet_neutral_mm=point.tolist(),distal_body='thumb_mcp',distal_sheath_stop_installed=False,endpoint_straight_separation_lower_bound_mm=float(np.linalg.norm(seat-point)),
            scope='No long housing/cord path is drawn here.Endpoint distance is only a lower bound,not a selected cable cut length.The distal1.2mm formed cord guide is not a3mm housing abutment.'))
    seams=[]
    for port in model.channels():
        arc,line=port['pieces'];t0=np.asarray(arc['edge'].tangentAt(0).toTuple());t1=np.asarray(arc['edge'].tangentAt(1).toTuple())
        seams.append(dict(cord=port['cord'],free_entry_tangent_error=float(np.linalg.norm(t0-[0,1,0])),arc_straight_tangent_error=float(np.linalg.norm(t1-line['tangent'])),adjuster_tangent_error=float(np.linalg.norm(line['tangent']-[0,0,-1]))))
    ownmeta=p.read(ROOT/'cad/motor_spool_coupling_v1/selected/generated/scene.json');ownrows=[z for z in ownmeta['retained_parts']+ownmeta['parts'] if z.get('module')==model.NAME or z['name'].startswith(model.NAME+'_')]
    prior=c.load(ownrows,source);oldmin=min(c.m.pack._bbox(z['shape'])[0][2] for z in prior);newmin=min(c.m.pack._bbox(z['shape'])[0][2] for z in parts)
    source['cad/motor_tendon_terminal_v1/interfaces.py']=model.drive.sha(Path(__file__))
    result=dict(completed=True,source_sha256=source,M3_head_contacts=headcontacts,nominal_lower_support_verified=all(z['distance_mm']<1e-5 and [z['a_above'],z['a_below'],z['b_above'],z['b_below']]==[True,False,False,True] for z in headcontacts),ports=ports,channel_tangent_seams=seams,
        new_terminal_min_z_mm=newmin,original_representative_module_min_z_mm=oldmin,extra_projection_below_original_module_mm=oldmin-newmin,
        human_form_envelope_accepted=False,long_motor_to_thumb_housing_connected=False,distal_sheath_stop_installed=False,thread_preload_or_strength_verified=False,
        original_anchor_tail_tangent_kink_preserved_deg=26.56505117707799,
        limitations='Custom closed0.8mm curved channels and1mm mounting pads require an actual manufacturing/finish/material/load decision.M5x0.8barrels/nuts,3/1.3mmhousing,R5are nominal engineering candidates,not selected supplied components or allowable bending/lifetime ratings.')
    result['input_hash_guard']=all(model.drive.sha(ROOT/z)==h for z,h in source.items());p.write(out,result);assert result['input_hash_guard'] and result['nominal_lower_support_verified']
if __name__=='__main__':main()
