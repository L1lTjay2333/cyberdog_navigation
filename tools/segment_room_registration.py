"""Offline bounded wall registration with disjoint fitting and validation frames."""
import argparse,json
from pathlib import Path
import numpy as np
import open3d as o3d
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from reconstruction_core import decode_depth_frame

def sample(rows,voxel=.06):
    p=np.concatenate(rows)
    _,idx=np.unique(np.floor(p/voxel),axis=0,return_index=True)
    return p[idx]

def warp(p,x,center):
    c,s=np.cos(x[2]),np.sin(x[2])
    out=p.copy()
    out[:,:2]=(p[:,:2]-center)@np.array([[c,s],[-s,c]])+center+x[:2]
    return out

def wall_model(points):
    tree=cKDTree(points)
    distance,idx=tree.query(points,k=12)
    patch=points[idx];patch=patch-patch.mean(axis=1,keepdims=True)
    values,vectors=np.linalg.eigh(patch.transpose(0,2,1)@patch)
    normal=vectors[:,:,0]
    valid=(distance[:,-1]<.35)&(values[:,0]<.12*values[:,1])&(values[:,1]>.1*values[:,2])&(abs(normal[:,2])<.35)
    return points[valid],normal[valid]

def error(points,target):
    p,n=target
    if len(p)<30:return .15,0.
    d,idx=cKDTree(p).query(points)
    valid=d<.15
    residual=np.full(len(points),.15)
    residual[valid]=abs(((points[valid]-p[idx[valid]])*n[idx[valid]]).sum(1))
    return float(residual.mean()),float(valid.mean())

def fit(source,target,check,validation):
    p,n=wall_model(target)
    if len(source)<100 or len(check)<100:return None,'few_source_walls'
    if len(p)<100:return None,'few_wall_samples'
    d,idx=cKDTree(p).query(source);valid=d<.15
    if valid.mean()<.4:return None,'weak_overlap'
    a,b,n=source[valid],p[idx[valid]],n[idx[valid]]
    center=np.median(a[:,:2],axis=0)
    design=np.column_stack([n[:,:2],(a[:,0]-center[0])*n[:,1]-(a[:,1]-center[1])*n[:,0]])
    singular=np.linalg.svd(design,compute_uv=False)
    if singular[-1]<max(1.,singular[0]*.08):return None,'wall_direction_unobservable'
    def residual(x):
        return ((warp(a,x,center)-b)*n).sum(1)
    bound=np.array([.12,.12,np.deg2rad(4)])
    result=least_squares(residual,np.zeros(3),bounds=(-bound,bound),loss='huber',f_scale=.02)
    if np.any(abs(result.x)>.95*bound):return None,'correction_at_bound'
    before,overlap=error(check,validation)
    after,new_overlap=error(warp(check,result.x,center),validation)
    if min(overlap,new_overlap)<.3:
        return None,'weak_validation_overlap'
    if after>before*.9 or before-after<.002 or new_overlap<overlap-.02:
        return None,'held_out_no_improvement'
    return dict(transform=result.x.tolist(),center=center.tolist(),before=before,after=after,
                overlap_before=overlap,overlap_after=new_overlap),'accepted'

def run(source,folder):
    folder.mkdir(exist_ok=True)
    records=[json.loads(s) for s in source.open()]
    frames=[decode_depth_frame(r['depth'],stride=8) for r in records]
    blocks=[frames[i:i+6] for i in range(0,len(frames),6)]
    corrected=[];audit=[]
    for i,block in enumerate(blocks):
        correction=None;reason='first_or_short_block'
        if i and len(block)>=4 and len(blocks[i-1])>=4:
            previous=blocks[i-1]
            source_points=wall_model(sample(block[::2]))[0];target=sample(previous[::2])
            check=wall_model(sample(block[1::2]))[0];validation=wall_model(sample(previous[1::2]))
            correction,reason=fit(source_points,target,check,validation)
        audit.append(dict(block=i,first_frame=i*6,frames=len(block),reason=reason,correction=correction))
        corrected.extend([warp(p,correction['transform'],correction['center']) if correction else p for p in block])
    def consensus(batches):
        cells={}
        for points in batches:
            _,idx=np.unique(np.floor(points/.05),axis=0,return_index=True)
            for p in points[idx]:
                key=tuple(np.floor(p/.05).astype(int))
                if key not in cells:cells[key]=[p.copy(),1]
                else:cells[key][0]+=p;cells[key][1]+=1
        p=np.array([v/n for v,n in cells.values() if n>=3])
        cloud=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(p))
        return cloud.remove_statistical_outlier(20,2)[0]
    clouds=[consensus(frames),consensus(corrected)]
    for name,cloud in zip(('baseline','segment_refined'),clouds):
        o3d.io.write_point_cloud(str(folder/(name+'.ply')),cloud)
    points=[np.asarray(c.points) for c in clouds]
    lo=np.minimum(*[p[:,:2].min(0) for p in points])-.1
    hi=np.maximum(*[p[:,:2].max(0) for p in points])+.1
    w,h=np.ceil((hi-lo)/.025).astype(int)+1;images=[]
    for p in points:
        p=p[(p[:,2]>.15)&(p[:,2]<1.4)]
        xy=np.floor((p[:,:2]-lo)/.025).astype(int)
        image=np.full((h,w,3),245,np.uint8);image[h-1-xy[:,1],xy[:,0]]=[30,70,110];images.append(image)
    o3d.io.write_image(str(folder/'comparison.png'),o3d.geometry.Image(np.ascontiguousarray(np.concatenate(images,axis=1))))
    accepted=[r['correction'] for r in audit if r['correction']]
    report=dict(frames=len(frames),blocks=len(blocks),accepted=len(accepted),
                held_out_mean_residual_m=np.mean([[r['before'],r['after']] for r in accepted],axis=0).tolist() if accepted else None,
                max_yaw_deg=max((abs(np.degrees(r['transform'][2])) for r in accepted),default=0),
                limitations=['Local bounded segment adjustments only; corridor parallelism not proven.',
                             'Validation frames disjoint in time; same physical surfaces may appear.',
                             'No automatic motion or changes to factory navigation.'],audit=audit)
    (folder/'report.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='audit'}))
if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('source',type=Path);parser.add_argument('folder',type=Path)
    args=parser.parse_args();run(args.source,args.folder)
