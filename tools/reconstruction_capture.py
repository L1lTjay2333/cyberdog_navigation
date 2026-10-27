"""Transient ROS depth subscriber. No services, publishers or robot controls."""
import base64
import json
import math
import time
import zlib
import rclpy
from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from reconstruction_core import FrameQueue

seconds = int(globals().get('CAPTURE_SECONDS', 30))
if not 1 <= seconds <= 60:
    raise ValueError('Capture duration outside bounds')
namespace = '/mi_desktop_48_b0_2d_7b_0b_4a'
rclpy.init(args=[])
node = rclpy.create_node('readonly_static_reconstruction', enable_rosout=False, start_parameter_services=False)
buffer = Buffer()
listener = TransformListener(buffer, node)
queue = FrameQueue()
calibration = [None]
last_sample = [0.]
counts = {'images': 0, 'sent': 0, 'bad_time_or_calibration': 0, 'tf_retries': 0}


def emit(data):
    print(json.dumps(data, allow_nan=False, separators=(',', ':')), flush=True)


def receive_info(msg):
    calibration[0] = msg


def receive_depth(msg):
    counts['images'] += 1
    now = time.monotonic()
    if now-last_sample[0] < 1./3:
        return
    last_sample[0] = now
    info = calibration[0]
    stamp = Time.from_msg(msg.header.stamp)
    age = (node.get_clock().now().nanoseconds-stamp.nanoseconds)*1e-9
    if (info is None or not msg.header.frame_id or stamp.nanoseconds <= 0 or not -.05 <= age <= .7
            or msg.header.frame_id != info.header.frame_id
            or msg.width != info.width or msg.height != info.height
            or abs(Time.from_msg(info.header.stamp).nanoseconds-stamp.nanoseconds)*1e-9 > .2):
        counts['bad_time_or_calibration'] += 1
        return
    if (msg.encoding not in ('16UC1', '32FC1') or len(msg.data) != msg.step*msg.height
            or not 1 <= msg.width <= 1280 or not 1 <= msg.height <= 1024
            or not 0 < len(msg.data) <= 16*1024*1024):
        counts['bad_time_or_calibration'] += 1
        return
    # Keep the original frame and matching calibration while waiting for TF.
    queue.offer({'image': msg, 'info': info, 'stamp': stamp.nanoseconds/1e9}, now)


def lookup(item):
    msg = item['image']
    stamp = Time.from_msg(msg.header.stamp)
    age = (node.get_clock().now().nanoseconds-stamp.nanoseconds)*1e-9
    if not -.05 <= age <= 2.:
        return None
    try:
        # Local full-pose reconstruction, independent of stock global map ownership.
        tf = buffer.lookup_transform('vodom', msg.header.frame_id, stamp)
        p, q = tf.transform.translation, tf.transform.rotation
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if not all(math.isfinite(v) for v in values):
            return None
        return {'translation': values[:3], 'quaternion': values[3:],
                'tf_stamp': tf.header.stamp.sec+tf.header.stamp.nanosec/1e9}
    except Exception:
        counts['tf_retries'] += 1
        return None


subs = [node.create_subscription(CameraInfo, namespace+'/camera/depth/camera_info', receive_info, qos_profile_sensor_data),
        node.create_subscription(Image, namespace+'/camera/depth/image_rect_raw', receive_depth, qos_profile_sensor_data)]
started = time.monotonic()
last_sent = started
emit({'event': 'started', 'frame': 'vodom', 'seconds': seconds, 'control_requests': 0})
try:
    while time.monotonic()-started < seconds:
        rclpy.spin_once(node, timeout_sec=.02)
        for item, pose in queue.ready(lookup, time.monotonic()):
            msg, info = item['image'], item['info']
            if any(not math.isfinite(float(x)) for x in info.p) or len(info.p) != 12 or info.p[3] != 0 or info.p[7] != 0:
                counts['bad_time_or_calibration'] += 1
                continue
            emit(dict(pose, event='depth', stamp=item['stamp'], frame='vodom',
                      source_frame=msg.header.frame_id, width=int(msg.width), height=int(msg.height),
                      step=int(msg.step), encoding=msg.encoding, bigendian=bool(msg.is_bigendian),
                      intrinsics=[float(info.p[i]) for i in (0,5,2,6)],
                      depth_zlib_b64=base64.b64encode(zlib.compress(bytes(msg.data), 1)).decode('ascii'),
                      source_age_s=(node.get_clock().now().nanoseconds-Time.from_msg(msg.header.stamp).nanoseconds)*1e-9))
            counts['sent'] += 1
            last_sent = time.monotonic()
        if time.monotonic()-started > 6 and time.monotonic()-last_sent > 3:
            emit({'event':'error','message':'Depth/TF stream has no matching frames; stop static capture','counts':counts,'control_requests':0})
            break
    emit({'event': 'finished', 'counts': counts, 'queue_dropped': queue.dropped,
          'queue_expired': queue.expired, 'queue_pending': len(queue.pending), 'control_requests': 0})
finally:
    node.destroy_node()
    rclpy.shutdown()
