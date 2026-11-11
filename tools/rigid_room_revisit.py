"""Offline bounded SE(2) revisit search; disjoint fit, ranking and confirmation frames."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.optimize import minimize

ROOT=Path(__file__).resolve().parents[1]
def sample(rows, step):
    points=np.concatenate(rows)
    _, idx=np.unique(np.floor(points/step),axis=0,return_index=True)
    return points[idx]

def warp(points, x, center):
    a=np.deg2rad(x[2])
    rot=np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
    out=points.copy()
    out[:,:2]=(points[:,:2]-center)@rot.T+center+x[:2]
    return out

def overlap(a,b):
    return [float(np.isfinite(cKDTree(target).query(source,distance_upper_bound=.1)[0]).mean())
            for source,target in [(a,b),(b,a)]]

def run():
    rows=[]
    for file in sorted((ROOT/'evidence/live_room_processed_20261103').glob('session*.jsonl')):
        with file.open() as stream:
            rows.extend(np.asarray(json.loads(line)['points_xyz']) for line in stream)
    candidates=[]
    for i in range(0,170,10):
        first=rows[i:i+10]
        a=sample(first[::3],.12)
        tree=cKDTree(a)
        for j in range(len(rows)-170,len(rows),10):
            last=rows[j:j+10]
            b=sample(last[::3],.12)
            center=np.median(b[:,:2],axis=0)
            # Remove directions already impossible under bounded translation.
            seeds=[]
            for yaw in [-20,-10,0,10,20]:
                turned=warp(b,[0,0,yaw],center)
                for dx in [-1,0,1]:
                    for dy in [-1,0,1]:
                        dist=tree.query(turned[::3]+[dx,dy,0],distance_upper_bound=.4)[0]
                        seeds.append((np.minimum(dist,.4).mean(),[dx,dy,yaw]))
            loss,seed=min(seeds,key=lambda pair:pair[0])
            candidates.append((float(loss),i,j,seed))
    ranked=[]
    for _,i,j,seed in sorted(candidates)[:16]:
        a=sample(rows[i:i+10:3],.10)
        b=sample(rows[j:j+10:3],.10)
        tree=cKDTree(a)
        center=np.median(b[:,:2],axis=0)
        def objective(x):
            return np.minimum(tree.query(warp(b,x,center),distance_upper_bound=.3)[0],.3).mean()
        result=minimize(objective,seed,method='Powell',bounds=[(-1.5,1.5),(-1.5,1.5),(-25,25)],options={'maxiter':28})
        x=result.x if result.fun<objective(seed) else np.array(seed)
        aa=sample(rows[i+1:i+10:3],.08)
        bb=sample(rows[j+1:j+10:3],.08)
        score=overlap(aa,warp(bb,x,center))
        ranked.append(dict(first=i+1,last=j+1,transform=x.tolist(),center=center.tolist(),ranking_overlap=score))
    best=max(ranked,key=lambda r:min(r['ranking_overlap']))
    i,j=best['first']-1,best['last']-1
    a=sample(rows[i+2:i+10:3],.08)
    b=sample(rows[j+2:j+10:3],.08)
    best['confirmation_before']=overlap(a,b)
    best['confirmation_after']=overlap(a,warp(b,np.array(best['transform']),np.array(best['center'])))
    report=dict(window_pairs=len(candidates),refined=len(ranked),best=best,correction_applied=False)
    (ROOT/'evidence/room_registration_replay/rigid_revisit.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report))
if __name__=='__main__':
    run()
