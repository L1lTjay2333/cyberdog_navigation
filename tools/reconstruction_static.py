"""Bounded stationary pixel consensus. PC only; never applied to moving scans."""
import base64
import math
import zlib
from collections import deque
import numpy as np
from reconstruction_core import StaticModel, checked_pose, decode_depth_frame
from static_depth_consensus import consensus


def rotation(q):
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def anchor_depth(frame, anchor, stride):
    """Reproject measured points to first camera; nearest-Z occlusion, no fill."""
    world = decode_depth_frame(frame, stride)
    camera = (world-anchor[0]).dot(rotation(anchor[1]))
    h=(frame['height']+stride-1)//stride;w=(frame['width']+stride-1)//stride
    fx,fy,cx,cy=np.asarray(frame['intrinsics'])/stride
    camera=camera[np.isfinite(camera).all(axis=1)&(camera[:,2]>=.15)&(camera[:,2]<=4.)]
    z=camera[:,2]
    u=np.rint(camera[:,0]*fx/z+cx);v=np.rint(camera[:,1]*fy/z+cy)
    valid=(u>=0)&(u<w)&(v>=0)&(v<h)
    result=np.full(h*w,np.inf)
    np.minimum.at(result,v[valid].astype(int)*w+u[valid].astype(int),z[valid])
    result[~np.isfinite(result)]=np.nan
    return result.reshape(h,w)


class ConsensusStaticModel(StaticModel):
    """Five distinct observations, strict 3/5 support; no hole filling.

    Reproject to one reference camera before comparing pixel depths, compensating
    small standing motion. Fail closed on larger changes from session anchor.
    Thresholds are conservative operating limits, not accuracy claims.
    """
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.history = deque(maxlen=5)
        self.signature = None
        self.consensus_updates = 0
        self.consensus_pixels = 0
        self.valid_pixels = 0

    def add(self, frame, stride=4):
        t, q = checked_pose(frame)
        signature = (frame['frame'], frame['source_frame'], frame.get('width'),
                     frame.get('height'), tuple(frame.get('intrinsics', [])), stride)
        if self.anchor is not None:
            if signature != self.signature:
                raise ValueError('静态相机来源或内参改变；请重新开始采集')
            if frame['stamp'] <= self.stamp:
                return False
            angle = 2 * math.acos(min(1., abs(float(np.dot(q, self.anchor[1])))))
            if np.linalg.norm(t-self.anchor[0]) > .03 or angle > math.radians(5):
                raise ValueError('静态位姿发生变化，已停止像素融合；移动扫描请使用移动模式')
            if frame['stamp']-self.stamp > 2.5:
                raise ValueError('静态深度数据中断；请重新开始采集')
        anchor = self.anchor if self.anchor is not None else (t,q)
        depth = anchor_depth(frame, anchor, stride)
        if not np.any(np.isfinite(depth) & (depth >= .15) & (depth <= 4.)):
            raise ValueError('No valid depth points in supported range')
        candidate = list(self.history)[-4:] + [depth]
        cells = None
        pixels = 0
        if len(candidate) == 5:
            filtered, support, required = consensus(np.stack(candidate))
            pixels = int(np.isfinite(filtered).sum())
            if pixels:
                # Reproject consensus once, using its original pixel centres.
                h, w = filtered.shape
                derived = dict(frame, width=w, height=h, step=w*4,
                               encoding='32FC1', bigendian=False,
                               translation=anchor[0].tolist(), quaternion=anchor[1].tolist(),
                               intrinsics=[v/stride for v in frame['intrinsics']],
                               depth_zlib_b64=base64.b64encode(zlib.compress(
                                   filtered.astype('<f4').tobytes())).decode('ascii'))
                model = StaticModel(voxel=self.voxel, max_voxels=self.max_voxels)
                model.add(derived, stride=1)
                cells = model.cells
            else:
                cells = {}
        # Commit only after all validation/projection/capacity checks pass.
        if self.anchor is None:
            self.anchor, self.frame = (t, q), frame['frame']
            self.source_frame, self.signature = frame['source_frame'], signature
        self.history.append(depth)
        self.stamp, self.last_pose = frame['stamp'], (t, q)
        self.accepted_frames += 1
        if cells is not None:
            # Replace the snapshot: never accumulate obsolete outliers forever.
            self.cells = cells
            self.consensus_updates += 1
            self.consensus_pixels = pixels
            self.valid_pixels = int(np.sum(np.any(np.isfinite(candidate) &
                (np.asarray(candidate) >= .15) & (np.asarray(candidate) <= 4.), axis=0)))
        return True

    def points(self, minimum_frames=2):
        # Support was established per pixel before voxel reduction.
        return super().points(minimum_frames=1)

    def stats(self):
        return dict(super().stats(), temporal_filter='static_reprojected_majority_3_of_5',
                    temporal_window_frames=len(self.history), temporal_required_frames=5,
                    consensus_updates=self.consensus_updates,
                    consensus_pixels=self.consensus_pixels,
                    consensus_valid_pixels=self.valid_pixels,
                    consensus_span_m=.02, static_translation_limit_m=.03,
                    static_rotation_limit_deg=5., absolute_accuracy_verified=False)
