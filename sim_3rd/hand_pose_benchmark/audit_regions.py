"""Postprocess saved meshes: distinguish crossings between different fingers.

Regions are the dominant summed MANO skinning weights over each face. Palm is
region 0; thumb/index/middle/ring/little are 1..5. This is a geometric diagnostic,
not a physical penetration depth or a hand-labelled bad-pose classification.
"""
import argparse
import inspect
import json
import pathlib
import pickle
import sqlite3
import sys
import time
import zlib

import numpy as np
from numba import njit
from metrics import triangles_cross,mesh_crossings

ROOT=pathlib.Path(r'E:\hand_recog\vision-results\20261026')

def face_regions(faces):
    sys.path.insert(0,r'E:\hand_recog\vision-runtime\chumpy')
    if not hasattr(inspect,'getargspec'):inspect.getargspec=inspect.getfullargspec
    for name,value in dict(bool=bool,int=int,float=float,complex=complex,object=object,unicode=str,str=str).items():
        if name not in np.__dict__:setattr(np,name,value)
    with open(r'E:\hand_recog\vision-runtime\models\wilor\pretrained_models\MANO_RIGHT.pkl','rb') as f:
        mano=pickle.load(f,encoding='latin1')
    weights=np.asarray(mano['weights'])
    groups=[[0],[13,14,15],[1,2,3],[4,5,6],[10,11,12],[7,8,9]]
    scores=np.stack([weights[:,g].sum(axis=1) for g in groups],axis=1)
    regions=scores[faces].sum(axis=1).argmax(axis=1)
    return regions,np.asarray(mano['v_template'],dtype=np.float64)

@njit(cache=True)
def detail(vertices,faces,candidate,regions):
    counts=np.zeros((6,6),np.int64);hit=np.zeros(len(faces),np.uint8)
    for ii in range(len(candidate)):
        a=candidate[ii];ra=regions[a]
        if ra==0:continue
        for jj in range(ii+1,len(candidate)):
            b=candidate[jj];rb=regions[b]
            if rb==0 or ra==rb:continue
            shared=False
            for p in faces[a]:
                for q in faces[b]:
                    if p==q:shared=True
            if shared:continue
            if triangles_cross(vertices[faces[a]],vertices[faces[b]]):
                counts[min(ra,rb),max(ra,rb)]+=1;hit[a]=1;hit[b]=1
    return counts,np.where(hit>0)[0]

def process(model,phase='eval',root=ROOT):
    faces=np.load(root/f'{model}_faces.npy');regions,neutral=face_regions(faces)
    neutral_count,neutral_hit=mesh_crossings(neutral,faces)
    neutral_fingers,_=detail(neutral,faces,neutral_hit,regions)
    src=sqlite3.connect(f'file:{(root/f"{model}_{phase}.sqlite").as_posix()}?mode=ro',uri=True)
    out=sqlite3.connect(root/f'{model}_{phase}_regions.sqlite')
    out.execute('CREATE TABLE IF NOT EXISTS regions (id TEXT PRIMARY KEY, data TEXT)')
    done={r[0] for r in out.execute('SELECT id FROM regions')};count=0
    for sid,payload in src.execute("SELECT id,payload FROM results WHERE status='complete'"):
        if sid in done:continue
        d=json.loads(zlib.decompress(payload));hs=[]
        for h in d.get('hands',[]):
            candidates=np.asarray(h.get('geometry',{}).get('mesh_crossing_faces',[]),dtype=np.int64)
            counts,hit=detail(np.asarray(h['vertices'],dtype=np.float64),faces,candidates,regions)
            hs.append(dict(different_finger_pairs=int(counts.sum()),different_finger_faces=hit.tolist(),region_pairs=counts.tolist()))
        out.execute('INSERT INTO regions VALUES (?,?)',(sid,json.dumps(hs)));count+=1
        if count%100==0:out.commit()
    out.commit();total=out.execute('SELECT count(*) FROM regions').fetchone()[0]
    out.close();src.close()
    result=dict(model=model,phase=phase,processed=total,new=count,neutral_mesh_crossing_pairs=int(neutral_count),neutral_different_finger_pairs=int(neutral_fingers.sum()))
    (root/f'{model}_{phase}_regions.json').write_text(json.dumps(result,indent=2),encoding='utf8')
    print(json.dumps(result),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--model',choices=['wilor','hamer'],required=True);p.add_argument('--phase',default='eval');p.add_argument('--output',type=pathlib.Path,default=ROOT)
    a=p.parse_args();process(a.model,a.phase,a.output)
