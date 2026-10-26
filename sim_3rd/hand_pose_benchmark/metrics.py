"""Explicit GT errors and strict non-adjacent mesh surface crossings.

Surface intersections are geometric flags, not proof of physical tissue overlap.
MediaPipe supplies no surface, so it does not receive this mesh metric.
"""
import numpy as np
from numba import njit
from scipy.optimize import linear_sum_assignment

CHAINS = ((0,1,2,3,4),(0,5,6,7,8),(0,9,10,11,12),(0,13,14,15,16),(0,17,18,19,20))
EDGES = [(a,b) for chain in CHAINS for a,b in zip(chain[:-1],chain[1:])]


def bend_angles(xyz):
    result = []
    for chain in CHAINS:
        bones = np.diff(xyz[list(chain)], axis=0)
        norms = np.linalg.norm(bones, axis=1)
        for j in range(3):
            denom = norms[j]*norms[j+1]
            result.append(np.degrees(np.arccos(np.clip(np.dot(bones[j],bones[j+1])/denom,-1,1))) if denom > 1e-10 else np.nan)
    return np.array(result)


def match_and_measure(predictions, ground_truth):
    costs = np.full((len(ground_truth),len(predictions)), 1e6)
    for g, gt in enumerate(ground_truth):
        mask = gt['match_mask']
        if mask.sum() < 3:
            continue
        diag = max(np.linalg.norm(np.ptp(gt['uv'][mask],axis=0)), 20.0)
        for p, pred in enumerate(predictions):
            costs[g,p] = np.median(np.linalg.norm(pred['uv'][mask]-gt['uv'][mask],axis=1))/diag
    matched = []
    if costs.size:
        rows, cols = linear_sum_assignment(costs)
        for g,p in zip(rows,cols):
            if costs[g,p] > .5:
                continue
            gt, pred = ground_truth[g], predictions[p]
            valid = gt['valid']
            x = pred['xyz'] - pred['xyz'][0]
            y = gt['xyz'] - gt['xyz'][0]
            err = np.linalg.norm(x-y,axis=1)*1000
            uv_error = np.linalg.norm(pred['uv']-gt['uv'],axis=1)
            angle_error = np.abs(bend_angles(x)-bend_angles(y))
            angle_mask=[]
            for chain in CHAINS:
                angle_mask.extend(bool(valid[list(chain[j:j+3])].all()) for j in range(3))
            angle_error[~np.array(angle_mask)] = np.nan
            matched.append(dict(gt=int(g),pred=int(p),mpjpe_mm=float(err[valid].mean()),
                                per_joint_mm=err.tolist(),joint_angle_mae_deg=float(np.nanmean(angle_error)),
                                bad_bends_30deg=int(np.sum(angle_error>30)),
                                per_bend_deg=np.nan_to_num(angle_error,nan=-1).tolist(),
                                visible_2d_error_px=float(uv_error[gt['match_mask']].mean()),
                                handedness_wrong=(bool(pred['side']!=gt['side']) if gt['side'] is not None else None)))
    return dict(gt_hands=len(ground_truth),detected_hands=len(predictions),matched_hands=len(matched),
                missed_hands=len(ground_truth)-len(matched),extra_hands=len(predictions)-len(matched),matches=matched)


@njit(cache=True)
def cross(a,b):
    return np.array([a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]])


@njit(cache=True)
def pierces(p,q,a,b,c):
    direction=q-p; e1=b-a; e2=c-a
    h=cross(direction,e2); det=np.dot(e1,h)
    if abs(det)<1e-15:
        return False
    inv=1.0/det; s=p-a
    u=inv*np.dot(s,h)
    if u<=1e-6 or u>=1-1e-6:
        return False
    v=inv*np.dot(direction,cross(s,e1))
    if v<=1e-6 or u+v>=1-1e-6:
        return False
    t=inv*np.dot(e2,cross(s,e1))
    return t>1e-6 and t<1-1e-6


@njit(cache=True)
def triangles_cross(a,b):
    for i in range(3):
        if pierces(a[i],a[(i+1)%3],b[0],b[1],b[2]):
            return True
        if pierces(b[i],b[(i+1)%3],a[0],a[1],a[2]):
            return True
    return False


@njit(cache=True)
def mesh_crossings(vertices, faces):
    n=len(faces)
    tris=np.empty((n,3,3));lo=np.empty((n,3));hi=np.empty((n,3))
    for i in range(n):
        for j in range(3):
            tris[i,j]=vertices[faces[i,j]]
        for k in range(3):
            lo[i,k]=min(tris[i,0,k],tris[i,1,k],tris[i,2,k])
            hi[i,k]=max(tris[i,0,k],tris[i,1,k],tris[i,2,k])
    order=np.argsort(lo[:,0]);count=0;hit=np.zeros(n,np.uint8)
    for ii in range(n):
        a=order[ii]
        for jj in range(ii+1,n):
            b=order[jj]
            if lo[b,0]>=hi[a,0]-1e-9:
                break
            if lo[a,1]>=hi[b,1]-1e-9 or lo[b,1]>=hi[a,1]-1e-9 or lo[a,2]>=hi[b,2]-1e-9 or lo[b,2]>=hi[a,2]-1e-9:
                continue
            shared=False
            for p in range(3):
                for q in range(3):
                    if faces[a,p]==faces[b,q]:shared=True
            if shared:
                continue
            if triangles_cross(tris[a],tris[b]):
                count+=1;hit[a]=1;hit[b]=1
    return count,np.where(hit>0)[0]


def geometric_flags(pred, faces=None):
    if faces is None or 'vertices' not in pred:
        return dict(mesh_crossing_pairs=None,mesh_crossing_faces=[],mesh_status='not_available')
    count, hit=mesh_crossings(np.ascontiguousarray(pred['vertices'],np.float64),np.ascontiguousarray(faces,np.int64))
    return dict(mesh_crossing_pairs=int(count),mesh_crossing_faces=hit.tolist(),mesh_status='measured')


@njit(cache=True)
def inter_mesh_crossings(a,b,faces):
    n=len(faces);ta=np.empty((n,3,3));tb=np.empty((n,3,3))
    la=np.empty((n,3));ha=np.empty((n,3));lb=np.empty((n,3));hb=np.empty((n,3))
    for i in range(n):
        for j in range(3):ta[i,j]=a[faces[i,j]];tb[i,j]=b[faces[i,j]]
        for k in range(3):
            la[i,k]=min(ta[i,0,k],ta[i,1,k],ta[i,2,k]);ha[i,k]=max(ta[i,0,k],ta[i,1,k],ta[i,2,k])
            lb[i,k]=min(tb[i,0,k],tb[i,1,k],tb[i,2,k]);hb[i,k]=max(tb[i,0,k],tb[i,1,k],tb[i,2,k])
    order=np.argsort(lb[:,0]);count=0;hit_a=np.zeros(n,np.uint8);hit_b=np.zeros(n,np.uint8)
    for i in range(n):
        for jj in range(n):
            j=order[jj]
            if lb[j,0]>=ha[i,0]-1e-9:break
            if la[i,0]>=hb[j,0]-1e-9 or la[i,1]>=hb[j,1]-1e-9 or lb[j,1]>=ha[i,1]-1e-9 or la[i,2]>=hb[j,2]-1e-9 or lb[j,2]>=ha[i,2]-1e-9:continue
            if triangles_cross(ta[i],tb[j]):count+=1;hit_a[i]=1;hit_b[j]=1
    return count,np.where(hit_a>0)[0],np.where(hit_b>0)[0]
