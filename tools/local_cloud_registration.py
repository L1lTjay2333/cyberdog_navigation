"""Small map-display corrections relative to a previous stock-aligned frame."""
import numpy as np
from scipy.spatial import cKDTree


class LocalRegistration:
    def __init__(self):
        self.previous = None
        self.cached = None

    def apply(self, cloud, key):
        if not cloud.get('aligned') or not key:
            return
        identity = (key, cloud['frame'], cloud['stamp'])
        if self.cached and self.cached[0] == identity:
            cloud.update(self.cached[1])
            return
        points = np.asarray(cloud['points_xyz'], dtype=float)
        previous = self.previous
        result = {'registration': {'applied': False}}
        if points.ndim == 2 and points.shape[1] == 3 and len(points) >= 100 and np.isfinite(points).all():
            if previous and previous[0][:2] == identity[:2] and 0 < identity[2]-previous[0][2] <= 1.5:
                correction = estimate(points, previous[1])
                if correction is not None:
                    fitted, info = correction
                    result.update(points_xyz=fitted.tolist(), registration=info)
            self.previous = identity, points
        else:
            self.previous = None
        self.cached = identity, result
        cloud.update(result)


def estimate(points, target):
    tree = cKDTree(target)
    distances, indices = tree.query(target, k=10)
    patches = target[indices]
    patches -= patches.mean(axis=1, keepdims=True)
    values, vectors = np.linalg.eigh(patches.transpose(0,2,1) @ patches)
    good = (distances[:,-1] < .24) & (values[:,1] > .0003) & (values[:,0] < .12*values[:,1])
    distance, match = tree.query(points)
    keep = (distance < .08) & good[match]
    if keep.sum() < max(80, .35*len(points)):
        return None
    source, fixed = points[keep], target[match[keep]]
    normals = vectors[match[keep],:,0]
    center = source.mean(axis=0)
    centered = source-center
    design = np.column_stack((normals[:,:2], centered[:,0]*normals[:,1]-centered[:,1]*normals[:,0]))
    residual = np.sum((fixed-source)*normals,axis=1)
    train = np.arange(len(source))%4 != 0
    weight = np.sqrt(np.minimum(1., .015/np.maximum(abs(residual[train]),1e-9)))
    u,s,vt = np.linalg.svd(design[train]*weight[:,None],full_matrices=False)
    observable = s > max(2.,s[0]*.12)
    if not observable.any():
        return None
    dx,dy,yaw = vt[observable].T @ ((u[:,observable].T @ (residual[train]*weight))/s[observable])
    if np.hypot(dx,dy)>.04 or abs(yaw)>np.deg2rad(2):
        return None
    rotation=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
    fitted=points.copy()
    fitted[:,:2]=(points[:,:2]-center[:2])@rotation.T+center[:2]+[dx,dy]
    # Validate with fresh correspondences on withheld samples, penalizing lost overlap.
    check=fitted[keep][~train]
    dist, idx=tree.query(check)
    valid=(dist<.08)&good[idx]
    errors=np.full(len(check),.08)
    errors[valid]=abs(np.sum((target[idx[valid]]-check[valid])*vectors[idx[valid],:,0],axis=1))
    before=np.mean(np.minimum(abs(residual[~train]),.04))
    after=np.mean(np.minimum(errors,.04))
    if after >= before*.9 or before-after < .001:
        return None
    return fitted,dict(applied=True,dx=float(dx),dy=float(dy),yaw=float(yaw),center=center.tolist(),
                       observable=int(observable.sum()),before=float(before),after=float(after))
