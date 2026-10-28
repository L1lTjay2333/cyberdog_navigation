"""Computer-side bounded online submaps; no ROS/control interfaces."""
import json
import math
import re
import zipfile
from pathlib import Path
import numpy as np
from reconstruction_core import MovingModel, checked_pose, decode_depth_frame
from reconstruction_submaps import DiskSubmaps


class StreamingModel(MovingModel):
    def __init__(self, folder, **kwargs):
        super().__init__(**kwargs)
        self.store = DiskSubmaps(folder, voxel=self.voxel, max_voxels=self.max_voxels)
        self.sealed_preview = []

    def check_motion(self, t, q, stamp):
        previous_t, previous_q = self.last_pose
        dt = stamp-self.stamp
        distance = float(np.linalg.norm(t-previous_t))
        angle = 2*math.acos(min(1., abs(float(np.dot(q, previous_q)))))
        if not 0 < dt <= 2.5:
            raise ValueError('Depth/pose sequence has a time gap; start a new scan')
        if distance > .2 or distance/dt > .45 or angle > math.radians(35) or angle/dt > 1.2:
            raise ValueError('Pose jump or movement too fast; fusion stopped, robot not controlled')

    def add(self, frame, stride=4):
        t,q = checked_pose(frame)
        if self.anchor is not None:
            if frame['frame'] != self.frame or frame['source_frame'] != self.source_frame:
                raise ValueError('Coordinate frame changed; start a new model')
            if frame['stamp'] <= self.stamp:return False
            self.check_motion(t,q,frame['stamp'])
        points = decode_depth_frame(frame,stride)
        self.store.add(points)
        self.cells = self.store.cells
        self.refresh_sealed_preview()
        if self.last_pose is not None:
            self.travel_m += float(np.linalg.norm(t-self.last_pose[0]))
        if self.anchor is None:
            self.anchor,self.frame,self.source_frame = (t,q),frame['frame'],frame['source_frame']
        self.last_pose = (t,q)
        self.stamp = frame['stamp'];self.accepted_frames += 1
        self.trajectory.append({'position':t.tolist(),'quaternion':q.tolist(),'stamp':self.stamp})
        return True

    def load_part(self, part):
        name = part['file']
        if not re.fullmatch(r'submap_[0-9]{3}\.npz', name):
            raise ValueError('Invalid submap name')
        path = self.store.folder/name
        if path.is_symlink() or path.stat().st_size > self.max_voxels*28+4096:
            raise ValueError('Submap file exceeds bounds')
        with zipfile.ZipFile(path) as archive:
            if set(archive.namelist())!={'points.npy','support.npy'}:
                raise ValueError('Unexpected submap archive contents')
            if any(p.file_size>self.max_voxels*24+256 for p in archive.infolist()):
                raise ValueError('Expanded submap arrays exceed bounds')
        with np.load(path,allow_pickle=False) as saved:
            points,support = saved['points'],saved['support']
        if (points.ndim!=2 or points.shape[1]!=3 or len(points)>self.max_voxels
                or support.shape!=(len(points),) or not np.isfinite(points).all()
                or support.dtype.kind not in 'iu'):
            raise ValueError('Invalid saved submap arrays')
        stable = points[support>=2]
        if len(stable)!=part['stable_points']:raise ValueError('Submap count mismatch')
        return stable

    def refresh_sealed_preview(self):
        while len(self.sealed_preview)<len(self.store.parts):
            points = self.load_part(self.store.parts[len(self.sealed_preview)])
            self.sealed_preview.append(points[::max(1,(len(points)+29999)//30000)].copy())

    def active_points(self, minimum_frames=2):
        return np.asarray([c[0]/c[1] for c in self.store.cells.values() if c[2]>=minimum_frames],dtype=float).reshape((-1,3))

    def stable_count(self):
        return sum(p['stable_points'] for p in self.store.parts)+sum(c[2]>=2 for c in self.store.cells.values())

    def preview(self, limit=30000):
        self.refresh_sealed_preview()
        active = self.active_points()
        old = [p for p in self.sealed_preview if len(p)]
        share = max(1,limit//max(1,len(old)+int(bool(len(active)))))
        clouds = [p[::max(1,(len(p)+share-1)//share)] for p in old]
        budget = limit-sum(len(p) for p in clouds)
        if len(active) and budget>0:clouds.append(active[::max(1,(len(active)+budget-1)//budget)])
        if not clouds:return []
        joined = np.concatenate(clouds)
        return joined[::max(1,(len(joined)+limit-1)//limit)].tolist()

    def points(self, minimum_frames=2):
        # Only a bounded preview. Full-resolution output uses export_snapshot.
        if minimum_frames!=2:raise ValueError('Streaming preview requires two-frame support')
        return self.preview()

    def export_snapshot(self):
        parts = list(self.store.parts)
        active = self.active_points().copy()
        count = sum(p['stable_points'] for p in parts)+len(active)
        def batches():
            for part in parts:yield self.load_part(part)
            if len(active):yield active
        return count,batches

    def finish(self):
        self.store.finish();self.cells = self.store.cells
        self.refresh_sealed_preview()

    def stats(self):
        result = super().stats()
        result.update(streaming_submaps=True, voxels=sum(p['voxels'] for p in self.store.parts)+len(self.store.cells),
                      active_voxels=len(self.store.cells), sealed_parts=len(self.store.parts),
                      stored_bytes=self.store.bytes,max_storage_bytes=self.store.max_bytes,
                      max_submaps=self.store.max_parts, stable_points_per_part=True,
                      global_deduplicated=False, max_displacement_m=None,max_travel_m=None)
        return result

    @classmethod
    def restore(cls,folder,summary,source):
        folder = Path(folder)
        if (folder/'manifest.json').stat().st_size>100000:raise ValueError('Manifest exceeds bounds')
        manifest = json.loads((folder/'manifest.json').read_text(encoding='utf-8'))
        if summary['frame']!=source['frame']:raise ValueError('Saved coordinate frame changed')
        parts = manifest['parts'];maximum = manifest['max_voxels_per_part']
        if len(parts)>64 or type(maximum) is not int or not 1<=maximum<=200000:
            raise ValueError('Invalid saved submap bounds')
        model = cls.__new__(cls)
        MovingModel.__init__(model,voxel=float(manifest['voxel_m']),max_voxels=maximum)
        store = DiskSubmaps.__new__(DiskSubmaps)
        store.folder,store.voxel,store.max_voxels = folder,model.voxel,maximum
        store.max_parts,store.max_bytes = 64,512*1024*1024
        store.parts,store.cells,store.previous = parts,{},None
        store.frames,store.bytes = int(manifest['frames']),sum(p['bytes'] for p in parts)
        if store.bytes>store.max_bytes:raise ValueError('Saved submap storage exceeds bounds')
        store.closed,store.manifest = True,manifest
        model.store,model.sealed_preview = store,[]
        model.anchor = checked_pose(source)
        model.frame,model.source_frame = source['frame'],source['source_frame']
        model.accepted_frames,model.stamp = summary['accepted_frames'],summary['last_stamp']
        model.trajectory = summary.get('trajectory',[])[:1000]
        model.travel_m = summary.get('travel_m',0.)
        model.last_pose = model.anchor
        if model.trajectory:
            last = model.trajectory[-1]
            model.last_pose = checked_pose(dict(source,translation=last['position'],quaternion=last['quaternion']))
        model.refresh_sealed_preview()
        return model
