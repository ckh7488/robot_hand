"""All-obstacle triangle proximity, with native near-contact distances/inside tests."""
import numpy as np
from scipy.spatial import cKDTree
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.TopAbs import TopAbs_IN
from OCP.gp import gp_Pnt
from OCP.BRepExtrema import BRepExtrema_DistShapeShape

def triangle_closest(points,tri):
    A,B,C=tri[:,0],tri[:,1],tri[:,2];u=B-A;v=C-A;n=np.cross(u,v);nn=np.sum(n*n,axis=1);Q=points-n*(np.sum((points-A)*n,axis=1)/np.maximum(nn,1e-30))[:,None];w=Q-A;uu=np.sum(u*u,axis=1);uv=np.sum(u*v,axis=1);vv=np.sum(v*v,axis=1);wu=np.sum(w*u,axis=1);wv=np.sum(w*v,axis=1);den=uu*vv-uv*uv;b=(wu*vv-wv*uv)/np.maximum(den,1e-30);d=(wv*uu-wu*uv)/np.maximum(den,1e-30);inside=(b>=0)&(d>=0)&(b+d<=1)
    candidates=[]
    for X,Y in [(A,B),(B,C),(C,A)]:
        D=Y-X;t=np.clip(np.sum((points-X)*D,axis=1)/np.maximum(np.sum(D*D,axis=1),1e-30),0,1);candidates.append(X+t[:,None]*D)
    qs=np.stack(candidates,axis=1);j=np.argmin(np.sum((qs-points[:,None,:])**2,axis=2),axis=1);edge=qs[np.arange(len(points)),j];return np.where(inside[:,None],Q,edge)

class Field:
    def __init__(self,rows,cq,bbox):
        self.rows=rows;self.cq=cq;self.box=np.array([bbox(z['shape']) for z in rows]);self.surfaces=[cq.Compound.makeCompound(z['shape'].Shells()) for z in rows];triangles=[];owners=[];self.classifiers=[];self.classifier_owner=[];self.classifier_boxes=[];self.native_queries=0
        for j,row in enumerate(rows):
            vertices,faces=row['shape'].tessellate(.1,.15);V=np.array([x.toTuple() for x in vertices]);T=V[np.asarray(faces,dtype=int)];T=T[np.linalg.norm(np.cross(T[:,1]-T[:,0],T[:,2]-T[:,0]),axis=1)>1e-12]
            while len(T):
                long=np.max(np.linalg.norm(T-np.roll(T,1,axis=1),axis=2),axis=1)>8.;triangles.append(T[~long]);owners.append(np.full(np.count_nonzero(~long),j,dtype=np.int32));q=T[long]
                if not len(q):break
                A,B,C=q[:,0],q[:,1],q[:,2];AB,BC,CA=(A+B)/2,(B+C)/2,(C+A)/2;T=np.concatenate([np.stack([A,AB,CA],axis=1),np.stack([AB,B,BC],axis=1),np.stack([CA,BC,C],axis=1),np.stack([AB,BC,CA],axis=1)])
            for sh in row['shape'].Solids():self.classifiers.append(BRepClass3d_SolidClassifier(sh.wrapped));self.classifier_owner.append(j);self.classifier_boxes.append(bbox(sh))
        self.tri=np.concatenate(triangles);self.owner=np.concatenate(owners);self.centers=self.tri.mean(axis=1);self.radius=float(np.linalg.norm(self.tri-self.centers[:,None,:],axis=2).max());self.tree=cKDTree(self.centers);self.classifier_boxes=np.asarray(self.classifier_boxes);self.calls=0
        print('field',len(rows),'parts',len(self.tri),'triangles','Rmax',self.radius,flush=True)
    def evaluate(self,P):
        self.calls+=1;_,near=self.tree.query(P,k=1,workers=1);q=triangle_closest(P,self.tri[near]);best=np.linalg.norm(P-q,axis=1);groups=self.tree.query_ball_point(P,best+self.radius+1e-10,workers=1);counts=np.array([len(x) for x in groups]);ids=np.concatenate(groups);who=np.repeat(np.arange(len(P)),counts);closest=triangle_closest(P[who],self.tri[ids]);d2=np.sum((P[who]-closest)**2,axis=1);start=0
        for j,n in enumerate(counts):
            k=start+int(np.argmin(d2[start:start+n]));q[j]=closest[k];near[j]=ids[k];start+=n
        distance=np.linalg.norm(P-q,axis=1);gradient=(P-q)/np.maximum(distance[:,None],1e-12);owner=self.owner[near].copy();inside=[[] for _ in P]
        for j,point in enumerate(P):
            boxes=self.classifier_boxes;candidate=np.flatnonzero(np.all((point>=boxes[:,0]-1e-7)&(point<=boxes[:,1]+1e-7),axis=1))
            for k in candidate:
                cl=self.classifiers[k];cl.Perform(gp_Pnt(*point),1e-7)
                if cl.State()==TopAbs_IN:inside[j].append(self.classifier_owner[k])
        for j,point in enumerate(P):
            if distance[j]>1.7 and not inside[j]:continue
            # 0.2 mm is a backend-switch band, never a required design clearance.
            candidates=set(inside[j]);gap=np.maximum(np.maximum(self.box[:,0]-point,point-self.box[:,1]),0.);candidates.update(np.flatnonzero(np.linalg.norm(gap,axis=1)<=distance[j]+.2).tolist());vertex=self.cq.Vertex.makeVertex(*point);values=[]
            for k in candidates:
                tool=BRepExtrema_DistShapeShape(vertex.wrapped,self.surfaces[k].wrapped);tool.SetMultiThread(False);tool.Perform();self.native_queries+=1
                if not tool.IsDone() or not tool.NbSolution():raise RuntimeError(('native distance failed',j,k))
                value=tool.Value();Q=np.array(tool.PointOnShape2(1).Coord());sgn=-1. if k in inside[j] else 1.;values.append((sgn*value,sgn*(point-Q)/max(value,1e-12),k))
            val,grad,k=min(values,key=lambda z:z[0]);distance[j]=val;gradient[j]=grad;owner[j]=k
        return distance,gradient,owner
