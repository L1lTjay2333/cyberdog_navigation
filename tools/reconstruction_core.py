"""Bounded depth/pose decoding and local static voxel fusion; no ROS controls."""
import base64
import math
import zlib
import numpy as np


def _finite(values, length):
    try:
        if len(values) != length or any(type(x) not in (int, float) or not math.isfinite(x) for x in values):
            raise ValueError('Non-finite or malformed pose/calibration')
        return np.asarray(values, dtype=float)
    except TypeError:
        raise ValueError('Malformed pose/calibration')


def checked_pose(frame):
    stamp = frame.get('stamp')
    if type(stamp) not in (int, float) or not math.isfinite(stamp) or stamp <= 0:
        raise ValueError('Invalid acquisition timestamp')
    if not frame.get('frame') or not frame.get('source_frame'):
        raise ValueError('Missing coordinate frame')
    if 'tf_stamp' in frame:
        tf_stamp = frame['tf_stamp']
        if type(tf_stamp) not in (int, float) or not math.isfinite(tf_stamp) or abs(tf_stamp-stamp) > 1e-6:
            raise ValueError('Depth and pose timestamps do not match')
    translation = _finite(frame.get('translation'), 3)
    quaternion = _finite(frame.get('quaternion'), 4)
    norm = np.linalg.norm(quaternion)
    if not .5 <= norm <= 1.5:
        raise ValueError('Invalid quaternion norm')
    return translation, quaternion / norm


def decode_depth_frame(frame, stride=4):
    translation, quaternion = checked_pose(frame)
    if type(stride) is not int or stride < 1:
        raise ValueError('Invalid sampling stride')
    width, height, step = (frame.get(k) for k in ('width', 'height', 'step'))
    if any(type(x) is not int for x in (width, height, step)) or not 1 <= width <= 1280 or not 1 <= height <= 1024:
        raise ValueError('Invalid image dimensions')
    encoding = frame.get('encoding')
    if encoding not in ('16UC1', '32FC1') or type(frame.get('bigendian')) is not bool:
        raise ValueError('Unsupported metric depth encoding')
    itemsize = 2 if encoding == '16UC1' else 4
    if not width * itemsize <= step <= width * itemsize + 4096:
        raise ValueError('Invalid image row layout')
    expected = step * height
    payload = frame.get('depth_zlib_b64')
    if not isinstance(payload, str) or len(payload) > 24 * 1024 * 1024:
        raise ValueError('Depth payload exceeds bound')
    try:
        decoder = zlib.decompressobj()
        raw = decoder.decompress(base64.b64decode(payload, validate=True), expected + 1)
        if len(raw) != expected or not decoder.eof or decoder.unconsumed_tail or decoder.unused_data:
            raise ValueError('Malformed or oversized depth payload')
    except (zlib.error, TypeError) as exc:
        raise ValueError('Invalid depth compression') from exc
    fx, fy, cx, cy = _finite(frame.get('intrinsics'), 4)
    if fx <= 0 or fy <= 0 or not 0 <= cx < width or not 0 <= cy < height:
        raise ValueError('Invalid intrinsics')
    dtype = ('>' if frame['bigendian'] else '<') + ('u2' if itemsize == 2 else 'f4')
    image = np.ndarray((height, width), dtype=dtype, buffer=raw, strides=(step, itemsize))
    z = image[::stride, ::stride].astype(float) * (.001 if itemsize == 2 else 1.)
    v, u = np.mgrid[0:height:stride, 0:width:stride]
    valid = np.isfinite(z) & (z >= .15) & (z <= 4.)
    with np.errstate(over='ignore', invalid='ignore', divide='ignore'):
        points = np.column_stack(((u[valid]-cx)*z[valid]/fx, (v[valid]-cy)*z[valid]/fy, z[valid]))
    if not np.isfinite(points).all():
        raise ValueError('Calibration produced non-finite projected points')
    x, y, zq, w = quaternion
    rotation = np.array([[1-2*(y*y+zq*zq), 2*(x*y-zq*w), 2*(x*zq+y*w)],
                         [2*(x*y+zq*w), 1-2*(x*x+zq*zq), 2*(y*zq-x*w)],
                         [2*(x*zq-y*w), 2*(y*zq+x*w), 1-2*(x*x+y*y)]])
    with np.errstate(over='ignore', invalid='ignore'):
        transformed = points.dot(rotation.T) + translation
    if not np.isfinite(transformed).all():
        raise ValueError('Pose produced non-finite transformed points')
    return transformed


class StaticModel:
    def __init__(self, voxel=.01, max_voxels=200000):
        if not math.isfinite(voxel) or voxel <= 0 or type(max_voxels) is not int or max_voxels < 1:
            raise ValueError('Invalid model bounds')
        self.voxel, self.max_voxels = voxel, max_voxels
        self.cells = {}
        self.anchor = None
        self.frame = None
        self.source_frame = None
        self.stamp = 0.
        self.accepted_frames = 0
        self.last_pose = None

    def check_motion(self, t, q, stamp):
        at, aq = self.anchor
        angle = 2 * math.acos(min(1., abs(float(np.dot(q, aq)))))
        if np.linalg.norm(t-at) > .03 or angle > math.radians(5):
            raise ValueError('Camera moved beyond static trial limits; fusion stopped')

    def add(self, frame, stride=4):
        t, q = checked_pose(frame)
        if self.anchor is not None:
            if frame['frame'] != self.frame or frame['source_frame'] != self.source_frame:
                raise ValueError('Coordinate frame changed; start a new model')
            if frame['stamp'] <= self.stamp:
                return False
            self.check_motion(t, q, frame['stamp'])
        points = decode_depth_frame(frame, stride)
        if not len(points):
            raise ValueError('No valid depth points in supported range')
        with np.errstate(over='ignore', invalid='ignore', divide='ignore'):
            scaled = np.floor(points/self.voxel)
        if not np.isfinite(scaled).all() or np.any(np.abs(scaled) > np.iinfo(np.int64).max//2):
            raise ValueError('Voxel coordinates exceed representable bounds')
        keys, inverse, counts = np.unique(scaled.astype(np.int64), axis=0, return_inverse=True, return_counts=True)
        additions = sum(tuple(key) not in self.cells for key in keys)
        if len(self.cells) + additions > self.max_voxels:
            raise ValueError('Model capacity reached; fusion stopped without discarding old surfaces')
        sums = np.column_stack([np.bincount(inverse, weights=points[:,i]) for i in range(3)])
        for key, total, count in zip(keys, sums, counts):
            cell = self.cells.setdefault(tuple(key), [np.zeros(3), 0, 0])
            cell[0] += total
            cell[1] += int(count)
            cell[2] += 1  # One supporting frame, regardless of points in this voxel.
        if self.anchor is None:
            self.anchor, self.frame = (t, q), frame['frame']
            self.source_frame = frame['source_frame']
        self.stamp, self.accepted_frames = frame['stamp'], self.accepted_frames + 1
        self.last_pose = (t, q)
        return True

    def points(self, minimum_frames=2):
        return [(cell[0]/cell[1]).tolist() for cell in self.cells.values() if cell[2] >= minimum_frames]

    def stats(self):
        return {'accepted_frames':self.accepted_frames, 'voxels':len(self.cells),
                'frame':self.frame, 'last_stamp':self.stamp, 'voxel_m':self.voxel,
                'max_voxels':self.max_voxels, 'scale_calibrated':False}


class MovingModel(StaticModel):
    """Short-distance odometry-guided fusion, not SLAM or robot control."""
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.trajectory = []
        self.travel_m = 0.

    def check_motion(self, t, q, stamp):
        previous_t, previous_q = self.last_pose
        dt = stamp-self.stamp
        distance = float(np.linalg.norm(t-previous_t))
        angle = 2*math.acos(min(1., abs(float(np.dot(q, previous_q)))))
        if not 0 < dt <= 2.5:
            raise ValueError('Depth/pose sequence has a time gap; start a new scan')
        if distance > .2 or distance/dt > .45 or angle > math.radians(35) or angle/dt > 1.2:
            raise ValueError('Pose jump or movement too fast; fusion stopped, robot not controlled')
        if np.linalg.norm(t-self.anchor[0]) > 2. or self.travel_m+distance > 3.:
            raise ValueError('Short-scan distance limit reached; save this model')

    def add(self, frame, stride=4):
        previous = self.last_pose
        changed = super().add(frame, stride)
        if changed:
            t, q = self.last_pose
            if previous is not None:
                self.travel_m += float(np.linalg.norm(t-previous[0]))
            self.trajectory.append({'position':t.tolist(), 'quaternion':q.tolist(), 'stamp':self.stamp})
        return changed

    def stats(self):
        return dict(super().stats(), mode='moving_local_point_cloud', travel_m=self.travel_m,
                    loop_closure=False, pose_source='vodom_original_depth_stamp',
                    max_displacement_m=2., max_travel_m=3.)


class FrameQueue:
    def __init__(self, capacity=6, max_age=2., wait=.15):
        if type(capacity) is not int or capacity < 1 or not 0 <= wait < max_age:
            raise ValueError('Invalid queue bounds')
        self.capacity, self.max_age, self.wait = capacity, max_age, wait
        self.pending, self.dropped, self.expired = [], 0, 0

    def offer(self, frame, now):
        if len(self.pending) >= self.capacity:
            _, discarded = self.pending.pop(0)
            if callable(getattr(self,'on_discard',None)):self.on_discard(discarded,'queue_full')
            self.dropped += 1
        self.pending.append((now, frame))

    def ready(self, lookup, now):
        waiting, ready = [], []
        for receipt, frame in self.pending:
            age = now - receipt
            if age >= self.max_age:
                self.expired += 1
                if callable(getattr(self,'on_discard',None)):self.on_discard(frame,'pose_unavailable')
                continue
            match = lookup(frame) if age >= self.wait else None
            if match is None:
                waiting.append((receipt, frame))
            else:
                ready.append((frame, match))
        self.pending = waiting
        return ready
