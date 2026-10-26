"""Explicit terminal assembly, face support and bounded static adjustment."""
import sys,math,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import terminal_selected as model
g,c,p,np,cq,HERE,ROOT=model.g,model.c,model.p,model.np,model.cq,model.HERE,model.ROOT
check=model.drive.module('_terminal022_inputs',Path(__file__).resolve().parent/'build_check.py');check.model=model;check.HERE=HERE
from OCP.OSD import OSD_ThreadPool
def main():
    out=HERE/'reviews/service.json';assert not out.exists();start=time.monotonic();static=p.read(HERE/'reviews/check.json');assert static['static_candidate_clear']
    design=static['design'];data,retained,guide=check.inputs(design);parts=c.load(p.read(ROOT/static['scene'])['parts'],data[0]);by={z['name']:z for z in parts};fixedby={z['name']:z for z in retained}
    source=data[0];source['cad/motor_tendon_terminal_v1/service.py']=model.drive.sha(Path(__file__))
    source['cad/motor_tendon_terminal_v1/selected/reviews/check.json']=model.drive.sha(HERE/'reviews/check.json')
    coupling_meta=p.read(ROOT/'cad/motor_spool_coupling_v1/selected/generated/scene.json')['parts'];coupling_names=[z['name'] for z in coupling_meta]
    unit=[model.NAME+suffix for suffix in ['_motor','_cradle','_body_screws','_base_screws']]+coupling_names
    screws=[model.NAME+'_rack_screw_'+str(i) for i in [1,2]];base_nuts=model.NAME+'_base_nuts'
    absent=unit+screws+[base_nuts];new_installed=[];wire_absent=[z['name'] for z in data[6] if z['name'].startswith(model.NAME+'_')];assert len(wire_absent)==4
    hand_fixed=[dict(name=z['name'],shape=data[4][z['name']]) for z in data[3]['hand'] if z['name'] not in ['thumb_segment2_dark_faceplate_clamp1_bolt','thumb_segment2_dark_faceplate_clamp2_bolt']]+guide
    record=dict(completed=False,source_sha256=source,paths=[],adjustments=[],cord_cases=[],contacts=[],OCC_pool_threads=OSD_ThreadPool.DefaultPool_s().NbThreads(),reporting_floor_mm3=.001,boolean_tolerance_mm=1e-6,
        electrical_connection_insertion_verified=False,physical_jig_or_tools_verified=False,whole_thumb_connection_complete=False)
    def save():p.write(out,record)
    def path(label,names,translations,scope):
        obstacles=[z for z in retained if z['name'] not in absent+names]+[z for z in data[6] if z['name'] not in wire_absent]+data[7]+hand_fixed+[by[n] for n in new_installed if n not in names]
        rows=[]
        for vector in translations:
            moved=[dict(name=n,shape=(by[n] if n in by else fixedby[n])['shape'].translate(vector)) for n in names]
            rows.append(dict(translation_mm=vector,native=c.material(moved,obstacles)))
        got=dict(id=label,moving=list(names),not_installed_packaging=list(absent),not_connected_electrical=list(wire_absent),already_installed_new=list(new_installed),scope=scope,samples=rows,clear=all(model.drive.clear(z['native']) for z in rows));record['paths'].append(got);save();print(label,got['clear'],flush=True)
    frame='thumb_ip_twin_sheath_terminal_frame'
    path('frame_from_rear',[frame],[[0,y,0] for y in [-35,-25,-15,-5,0]],'Existing adapter/wire guide is held on rack;own motor/cradle/output unit,base nuts and twoM3bolts are not installed.Twoold spacers permanently replaced by frame pads.Wires of this motor unconnected.Jig/holding and rack-to-arm installation not approved.')
    new_installed.append(frame)
    path('M3_before_motor',screws,[[0,0,z] for z in [-12,-9,-6,-3,0]],'Original twoM3x12 envelopes,stack unchanged;own motor/baseM2hardware absent.Rack nuts manually held in original seats.')
    for n in screws:absent.remove(n)
    path('preassembled_motor_and_output',unit,[[x,0,0] for x in [8,6,4,2,0]],'Motor/cradle and captive lowerM2screws preassembled on bench,plusunchangedDRIVE018output10at0degree.Own lowerM2nuts still absent.Samehorizontalright-column entry,now tested againstnew frame.Internalvendor/bench assembly not newly approved.')
    for n in unit:absent.remove(n)
    path('base_M2_nuts_after_unit',[base_nuts],[[x,0,0] for x in [-8,-6,-4,-2,0]],'Actual original compound of four lowerM2nuts approaches after unit seated;no missing motor/body screws excluded.')
    absent.remove(base_nuts);assert not absent
    for port in model.channels():
        i=port['cord'];top='thumb_ip_barrel_top_nut_'+str(i);barrel='thumb_ip_barrel_adjuster_'+str(i);bottom='thumb_ip_barrel_bottom_nut_'+str(i)
        sign=-1 if i==0 else 1
        path('upper_nut_side_'+str(i),[top],[[sign*x,0,0] for x in [12,9,6,3,0]],'Uppernut placed laterally aboveframeplate beforebarrel,avoidingintegral fairlead support above.It is held untilnominal M5thread engages;physicalholding/driver access unverified.')
        new_installed.append(top)
        path('barrel_from_below_'+str(i),[barrel,bottom],[[0,0,z] for z in [-25,-18,-12,-6,0]],'Bottom jam nut prethreaded on barrel.Bottomhousing head remains belowframe;onlyM5shaft passes through5.1mmclearance.Threads are nominal envelopes,not helical/supplier/thread-strength approval.')
        new_installed.extend([barrel,bottom])
    assert len(new_installed)==7
    housing=model.witnesses();cords=model.cords()
    for port in model.channels():
        i=port['cord'];n='thumb_ip_barrel_adjuster_'+str(i);w=housing[i]
        obstacles=[by[k] for k in by if k!=n]+retained+data[6]+data[7]
        for travel in [-2.,0.,2.]:
            moved=[dict(name=n,shape=by[n]['shape'].translate((0,0,travel))),dict(name=w['name'],shape=w['shape'].translate((0,0,travel)))]
            got=c.material(moved,obstacles);record['adjustments'].append(dict(cord=i,barrel_translation_z_mm=travel,native=got,clear=model.drive.clear(got)))
        xy=port['xy'];outer=model.cyl(2.5,-159,-135,xy).fuse(model.cyl(3.5,-165,-155,xy)).cut(model.cyl(.65,-165.01,-134.99,xy))
        got=c.material([dict(name='barrel_'+str(i)+'_whole_stroke_bound',shape=outer)],obstacles)
        record.setdefault('full_stroke_bounds',[]).append(dict(cord=i,native=got,clear=model.drive.clear(got),scope='Conservative continuous4mm axial stroke outer bound;central1.3mm passage retained,counterbore fill conservative.Not physical screw-motion/locknut preload or long housing flex.'))
    # Support and housing seating: actual mating faces, not a body label.
    def contact(label,a,b,point,axis):
        axis=np.asarray(axis,float);point=np.asarray(point,float);above=point+.02*axis;below=point-.02*axis
        record['contacts'].append(dict(id=label,distance_mm=a.distance(b),point_mm=point.tolist(),axis=axis.tolist(),a_above=bool(a.isInside(cq.Vector(*above),1e-6)),a_below=bool(a.isInside(cq.Vector(*below),1e-6)),b_above=bool(b.isInside(cq.Vector(*above),1e-6)),b_below=bool(b.isInside(cq.Vector(*below),1e-6))))
    frame_shape=by[frame]['shape'];base=fixedby[model.NAME+'_wire_guide_base']['shape']
    for xy in design['mounting_axes_xy_mm']:
        for k,a in enumerate([0,math.pi/2,math.pi,3*math.pi/2]):
            point=[xy[0]+2.2*math.cos(a),xy[1]+2.2*math.sin(a),-81.]
            contact('rack_pad_'+str(xy[1])+'_'+str(k),base,frame_shape,point,[0,0,1])
    for port in model.channels():
        i=port['cord'];xy=port['xy'];barrel=by['thumb_ip_barrel_adjuster_'+str(i)]['shape']
        for side,z,axis in [('top',-141,[0,0,1]),('bottom',-144,[0,0,-1])]:
            nut=by['thumb_ip_barrel_'+side+'_nut_'+str(i)]['shape'];contact('jam_nut_'+side+'_'+str(i),nut,frame_shape,[xy[0],xy[1]+3.2,z],axis)
        contact('sheath_shoulder_'+str(i),barrel,housing[i]['shape'],[xy[0]+1.,xy[1],-158.],[0,0,1])
    neutral=model.cords();base_lengths=[sum(z['length_mm'] for z in neutral if z['cord']==i) for i in [0,1]]
    for ip in [-15.,0.,45.,85.]:
        theta=ip*7.7/10.;now=model.cords(theta);coupling,_=model.drive.build();rotated=model.drive.moved(coupling,theta)
        obstacles=[z for z in retained if z['name'] not in coupling_names]+rotated+parts+data[6]+data[7]
        native=c.material(now,obstacles);length=[sum(z['length_mm'] for z in now if z['cord']==i) for i in [0,1]]
        between=c.material([z for z in now if z['cord']==0],[z for z in now if z['cord']==1])
        self_tests=[]
        for cord in [0,1]:
            same=[z for z in now if z['cord']==cord]
            self_tests.extend(c.material([z],same[k+2:]) for k,z in enumerate(same))
        self_test=dict(pairs_in_scope=sum(z['pairs_in_scope'] for z in self_tests),overlaps=[v for z in self_tests for v in z['overlaps']],operation_failures=[v for z in self_tests for v in z['operation_failures']])
        q=np.zeros(20);q[19]=ip;distal=g.cord_pieces(q);distal0=g.cord_pieces(np.zeros(20))
        local_changes=[sum(z['length_mm'] for z in distal if z['cord']==i)-sum(z['length_mm'] for z in distal0 if z['cord']==i) for i in [0,1]]
        residual=np.array(length)-base_lengths+local_changes
        record['cord_cases'].append(dict(ip_deg=ip,motor_angle_candidate_deg=theta,native=native,between_cords=between,nonadjacent_same_cord=self_test,motor_terminal_lengths_mm=length,current_distal_length_change_mm=local_changes,ideal_fixed_housing_residual_mm=residual.tolist(),
            scope='0.77derivedherefromcurrentCADradii10and7.7,conditionalonfutureconstantlengthhousing.Not inheritedfixed-parentbenchapproval.Motor-to-inletlonghousing/parentmotion/forceequilibrium absent.'));save();print('cord_angle',theta,len(native['overlaps']),flush=True)
    record.update(completed=True,required_service_samples=sum(len(z['samples']) for z in record['paths']),required_sampled_paths_clear=all(z['clear'] for z in record['paths']),nominal_contacts_verified=all(z['distance_mm']<1e-5 and [z['a_above'],z['a_below'],z['b_above'],z['b_below']]==[True,False,False,True] for z in record['contacts']),
        adjustment_space_clear=all(z['clear'] for z in record['adjustments']+record['full_stroke_bounds']),motor_terminal_cord_angles_clear=all(all(model.drive.clear(z[k]) for k in ['native','between_cords','nonadjacent_same_cord']) for z in record['cord_cases']),
        input_hash_guard=all(model.drive.sha(ROOT/z)==h for z,h in source.items()),elapsed_s=time.monotonic()-start)
    save();assert record['input_hash_guard'];print('service done',record['required_sampled_paths_clear'],record['nominal_contacts_verified'],record['adjustment_space_clear'],record['motor_terminal_cord_angles_clear'],flush=True)
if __name__=='__main__':main()
