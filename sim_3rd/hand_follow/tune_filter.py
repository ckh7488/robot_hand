"""Offline One Euro tuning on the synthetic hand: lag vs jitter trade-off (no camera, no robot)."""
import sys, numpy as np
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from hand_follow.retarget import raw_angles, NAMES
from hand_follow.synthetic import landmarks_from_angles, demo_angles
from hand_follow.filters import OneEuro
from replay_in_sim import best_lag
fps=120; t=np.arange(0,12.8,1/fps); rng=np.random.default_rng(3)
noise=float(sys.argv[1]) if len(sys.argv)>1 else 0.0015
truth=np.array([demo_angles(x) for x in t])
raw=np.array([raw_angles(landmarks_from_angles(a,noise_m=noise,rng=rng),"Right") for a in truth])
# hold samples: truth not changing
vel=np.abs(np.gradient(truth,axis=0)).max(axis=1)*fps; moving=vel>=1
k=int(0.25*fps); hold=np.array([not moving[max(0,i-k):i+1].any() for i in range(len(t))])
seg=np.cumsum(np.r_[0,np.diff(hold.astype(int))!=0])
def jitter(y):
    out=[]
    for s_ in np.unique(seg[hold]):
        m=(seg==s_)&hold
        if m.sum()>10: out.append((y[m][:,bend]-y[m][:,bend].mean(axis=0)).std(axis=0))
    return float(np.median(np.concatenate(out)))
bend=[i for i,n in enumerate(NAMES) if 'spread' not in n and 'sweep' not in n]
print(f"landmark noise {noise*1000:.1f} mm; raw jitter at rest (bend joints, deg std): {jitter(raw):.2f}")
for mc,beta in [(1.0,0.0),(1.5,0.02),(1.5,0.05),(1.0,0.1),(2.0,0.1),(1.0,0.3),(3.0,0.3)]:
    f=OneEuro(20,mc,beta); y=np.array([f(r,ti) for r,ti in zip(raw,t)])
    lag=np.median([best_lag(truth[:,j],y[:,j],1/fps) for j in bend])
    jit=jitter(y)
    print(f"min_cutoff {mc:3.1f} Hz beta {beta:4.2f}: lag {lag:5.1f} ms, rest jitter {jit:4.2f} deg")
