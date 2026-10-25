"""Explicit exploratory motion requests. Never clips angles or approves travel."""
import copy,json
from pathlib import Path
import numpy as np

DIGITS=['index','middle','ring','little','thumb']
def make(repo):
    repo=Path(repo)
    old=json.loads((repo/'cad/manufacturing_v2/generated/design.json').read_text())
    design=copy.deepcopy(old)
    ranges=dict(spread=[-27,27],mcp=[-35,100],pip=[-25,135],dip=[-15,95],
                thumb_base_sweep=[-70,60],thumb_opposition=[-5,90],thumb_mcp=[-15,90],thumb_ip=[-25,90])
    design['joint_ranges_deg']=ranges
    source=json.loads((repo/'cad/gesture_hand_v3/poses.json').read_text(encoding='utf-8'))
    names=source['joint_names']; cases=[]
    def add(ident,q,kind,required):
        cases.append(dict(id=ident,angles_deg=list(map(float,q)),kind=kind,required_function=required))
    for g in source['gestures']:
        q=g['angles_deg'][:]
        if g['id']=='thumb_up':q=source['reference']['authored_fist_deg'][:];q[16:]=[0,0,0,-15]
        add('gesture_'+g['id'],q,'accepted_gesture',True)
    for j,name in enumerate(names):
        key=name if j>=16 else name.split('_',1)[1]
        lo,hi=ranges[key]
        for angle in [lo,hi]:
            q=np.zeros(20);q[j]=angle
            add(name+f'_{angle:+g}',q,'isolated_axis_extreme',not(key in ['spread','thumb_base_sweep','thumb_opposition']))
    # Early practical counterexamples inside ordinary motion, before extra margin.
    for suffix,values in [('extension',[-25,-15,-8]),('tabletop',[90,0,0]),
                          ('hook',[0,100,75]),('flat_fist',[80,100,0]),('curl',[85,110,80])]:
        q=np.zeros(20)
        for i in range(4):q[i*4+1:i*4+4]=values
        add('four_fingers_'+suffix,q,'functional',True)
    for i,d in enumerate(DIGITS[:4]):
        for joint,val in [(1,-20),(2,-10),(3,-5)]:
            q=np.zeros(20);q[i*4+joint]=val
            add(d+'_ordinary_extension_'+names[i*4+joint].split('_')[1],q,'ordinary_extension',True)
        q=np.zeros(20);q[i*4+1:i*4+4]=[85,110,80]
        add(d+'_individual_curl',q,'functional',True)
    for scale in [1,1.2]:
        q=np.zeros(20);q[[0,4,8,12]]=np.array([22,6,-8,-22])*scale
        add('fan_'+str(scale),q,'coordinated_spread',True)
    return dict(schema='whole_hand_motion_requests.v1',design=design,names=names,cases=cases,
        original_ranges=old['joint_ranges_deg'],requested_ranges=ranges,
        range_note='Engineering exploration, not a universal anatomical envelope. Ordinary extension checks precede extra margins. Isolated spread/CMC extrema with stationary neighbours are stress probes, not proof of a required human pose. Full Cartesian products are not assumed anatomically feasible.',
        sources=[dict(url='https://pmc.ncbi.nlm.nih.gov/articles/PMC11436331/',
            evidence='390 hands; observed active extension at MCP, PIP and DIP; variation between subjects and digits.',
            use='Motivates extension tests; engineering bounds are our choices, not population coverage claims.')])
