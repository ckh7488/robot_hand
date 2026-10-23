import copy,json,math,tempfile,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from offline_pose_guard import check_request,ROOT
C=json.loads((ROOT/'cad/modular_roots_v9/design.json').read_text(encoding='utf-8'))
def request(pose='Open',error=.5):return {'geometry_basis':'V9','finger_pose_deg':copy.deepcopy(C['poses'][pose]),'arm_deg':[0,0,0,0],'spread_error_bound_deg':error,'other_joint_error_bound_deg':0}
rows=[]
def case(name,q,want):
 r=check_request(q);assert r['offline_geometry_domain_matched']==want,(name,r)
 assert r['physical_actuation_approved'] is False and r['continuous_path_approved'] is False
 rows.append({'name':name,'expected_match':want,'result':r})
case('Open simultaneous error within box',request(error=1),True)
case('Open +/-3deg audit failure is rejected',request(error=3),False)
case('Backbend excluded even with zero error',request('Backbend',0),False)
case('Close +/-3deg at fixed other joints',request('Close',3),True)
q=request();q['finger_pose_deg']['index'][0]+=.6;case('Offset plus error exceeds evidence box',q,False)
q=request();q['finger_pose_deg']['index'][0]+=.4;q['finger_pose_deg']['middle'][0]-=.4;case('Opposing offsets plus errors remain in simultaneous box',q,True)
q=request();q['other_joint_error_bound_deg']=.01;case('Missing flexion uncertainty evidence',q,False)
q=request();q['finger_pose_deg']['thumb'][1]+=.01;case('Thumb changed outside proof domain',q,False)
q=request();q['arm_deg'][0]=10;case('Coupled arm proof unavailable',q,False)
q=request();q['finger_pose_deg']['little'][0]=math.nan;case('NaN rejected',q,False)
q=request();del q['finger_pose_deg']['ring'];case('Missing finger rejected',q,False)
q=request();q['spread_error_bound_deg']=-1;case('Negative uncertainty rejected',q,False)
q=request();q['geometry_basis']='V13';case('Do not transfer V9 evidence to V13',q,False)
# Evidence cannot be replaced or widened accidentally while still returning success.
import shutil
with tempfile.TemporaryDirectory() as temp:
 root=Path(temp);limits=json.loads((ROOT/'safe_joint_limits.json').read_text(encoding='utf-8'))
 for item in limits['evidence']:
  dest=root/item['path'];dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/item['path'],dest)
 (root/'safe_joint_limits.json').write_text(json.dumps(limits))
 assert check_request(request(),root)['offline_geometry_domain_matched']
 limits['domains'][0]['simultaneous_four_spread_halfwidth_deg']=5
 (root/'safe_joint_limits.json').write_text(json.dumps(limits))
 assert not check_request(request(error=3),root)['offline_geometry_domain_matched']
 rows.append({'name':'Widened contract without matching geometry proof rejected','expected_match':False})
 (root/'safe_joint_limits.json').write_text((ROOT/'safe_joint_limits.json').read_text(encoding='utf-8'))
 target=root/limits['evidence'][0]['path'];target.write_bytes(target.read_bytes()+b' ')
 assert not check_request(request(),root)['offline_geometry_domain_matched']
 rows.append({'name':'Stale or changed geometry evidence rejected','expected_match':False})
print(json.dumps({'tests':len(rows),'passed':True,'rows':rows},indent=2))
