"""Offline wall-only revisit screening; recommendations never modify the map."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from rigid_room_revisit import ROOT, sample, warp

def walls(rows):
    p = sample(rows, .08)
    if len(p) < 10:
        return p[:0], p[:0]
    d, idx = cKDTree(p).query(p, k=10)
    v = p[idx] - p[idx].mean(axis=1, keepdims=True)
    eig, axes = np.linalg.eigh(v.transpose(0, 2, 1) @ v)
    n = axes[:, :, 0]
    ok = (d[:, -1] < .48) & (eig[:, 0] < .18*eig[:, 1]) & (eig[:, 1] > .12*eig[:, 2]) & (abs(n[:, 2]) < .32)
    return p[ok], n[ok]

def matches(a, b):
    p, n = a
    q, m = b
    if min(len(p),len(q)) == 0:
        return 0., 0
    d, idx = cKDTree(q).query(p)
    use = (d < .10) & (abs((n*m[idx]).sum(1)) > .92)
    # Two directions need distinct evidence, even for normals near a bin boundary.
    dirs = n[use,:2]
    angles = np.arctan2(dirs[:,1],dirs[:,0])
    support = [sum(abs(np.cos(angles-a)) > np.cos(np.deg2rad(15))) for a in np.deg2rad(np.arange(0,180,5))]
    second = max(min(x,y) for i,x in enumerate(support) for j,y in enumerate(support) if 9 <= abs(i-j) <= 27)
    return float(use.mean()), int(second)

def main():
    rows=[]
    for path in sorted((ROOT/'evidence/live_room_processed_20261103').glob('session*.jsonl')):
        with path.open() as f:
            rows.extend(np.array(json.loads(s)['points_xyz']) for s in f)
    starts=list(range(0,170,10)); ends=list(range(len(rows)-170,len(rows),10))
    cache={i:[walls(rows[i+k:i+10:3]) for k in range(3)] for i in starts+ends}
    candidates=[]
    for i in starts:
        p,n=cache[i][0]
        if len(p)<20: continue
        tree=cKDTree(p)
        for j in ends:
            q,m=cache[j][0]
            if len(q)<20: continue
            center=np.median(q[:,:2],axis=0)
            for yaw in (-20,-10,0,10,20):
                x=warp(q,[0,0,yaw],center)
                # Wall pairs vote for translations in 10 cm bins.
                ids=tree.query_ball_point(x,2.25)
                ii=np.repeat(np.arange(len(q)),[len(z) for z in ids])
                jj=np.array([k for z in ids for k in z],dtype=int)
                if not len(jj):continue
                diff=p[jj]-x[ii]
                rot=warp(m,[0,0,yaw],np.zeros(2))
                ok=(abs(diff[:,2])<.10)&(abs(diff[:,:2]).max(1)<=1.5)&(abs((rot[ii]*n[jj]).sum(1))>.92)
                if not ok.any():continue
                bins=np.rint(diff[ok,:2]*10).astype(int)+15
                counts=np.bincount(bins[:,0]*31+bins[:,1],minlength=961)
                for b in counts.argsort()[-2:]:
                    if not counts[b]:continue
                    t=[(b//31-15)/10,(b%31-15)/10,yaw]
                    def score(k):
                        a=cache[i][k]; v,w=cache[j][k]
                        c=(warp(v,t,center),warp(w,[0,0,yaw],np.zeros(2)))
                        return [min(pair) for pair in zip(matches(a,c),matches(c,a))]
                    frac,second=score(1)
                    if second>=10 and frac>=.25:
                        candidates.append((frac,second,i,j,t,center.tolist()))
    best=max(candidates,default=None)
    report=dict(frames=len(rows),window_pairs=len(starts)*len(ends),passing_rank_candidates=len(candidates),best=None,correction_applied=False)
    if best:
        frac,second,i,j,t,c=best
        v,w=cache[j][2]; b=(warp(v,t,c),warp(w,[0,0,t[2]],np.zeros(2)))
        report['best']=dict(first=i+1,last=j+1,transform=t,ranking=[frac,second],held_out=[matches(cache[i][2],b),matches(b,cache[i][2])])
    out=ROOT/'evidence/room_registration_replay/wall_revisit.json'
    out.write_text(json.dumps(report,indent=2));print(json.dumps(report))
if __name__=='__main__': main()
