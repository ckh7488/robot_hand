"""Audit saved outputs independently of gallery selection."""
import hashlib,importlib.metadata,json,pathlib,sqlite3,zlib
import numpy as np
from data import Samples

ROOT=pathlib.Path(r'E:\hand_recog\vision-results\20261026')
def main():
    expected={r['id'] for r in Samples('eval').rows};audit={}
    for model in ['mediapipe','wilor','hamer']:
        db=sqlite3.connect(f'file:{(ROOT/f"{model}_eval.sqlite").as_posix()}?mode=ro',uri=True)
        assert db.execute('PRAGMA quick_check').fetchone()[0]=='ok'
        ids=set();errors=[];hands=0;gt_count=0
        for sid,status,metrics,payload in db.execute('SELECT id,status,metrics,payload FROM results'):
            ids.add(sid)
            if status!='complete':errors.append(sid);continue
            m=json.loads(metrics);d=json.loads(zlib.decompress(payload))
            assert m['matched_hands']+m['missed_hands']==m['gt_hands']
            assert len(m['matches'])==m['matched_hands']
            assert m['detected_hands']==len(d['hands'])
            hands+=len(d['hands']);gt_count+=m['gt_hands']
            for h in d['hands']:
                for key in ['xyz','uv','vertices','camera_t']:
                    if key in h:assert np.isfinite(np.asarray(h[key])).all(),(sid,key)
                assert np.asarray(h['xyz']).shape==(21,3)
            for match in m['matches']:assert np.isfinite(match['mpjpe_mm']) and match['mpjpe_mm']>=0
        assert ids==expected,(model,len(expected-ids),len(ids-expected))
        assert not errors,(model,errors[:10])
        audit[model]=dict(images=len(ids),error_rows=len(errors),predicted_hands=hands,gt_hands=gt_count,sqlite_quick_check='ok',finite_coordinates=True)
        db.close()
    assert len({v['gt_hands'] for v in audit.values()})==1
    versions={p:importlib.metadata.version(p) for p in ['torch','torchvision','mediapipe','numpy','smplx','timm','ultralytics','numba']}
    source={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in pathlib.Path(__file__).parent.glob('*') if p.suffix in ['.py','.html','.txt','.md']}
    provenance=json.loads(pathlib.Path(r'E:\hand_recog\vision-runtime\models\provenance.json').read_text(encoding='utf8'))
    result=dict(audit=audit,software=versions,source_sha256=source,weights=provenance)
    (ROOT/'verified_run.json').write_text(json.dumps(result,indent=2),encoding='utf8')
    print(json.dumps(audit,indent=2))

if __name__=='__main__':main()
