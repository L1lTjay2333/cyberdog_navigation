"""Transient ROS depth subscriber. No services, publishers or robot controls."""
import base64
import json
import math
import time
import zlib
from collections import deque
import rclpy
from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from reconstruction_core import FrameQueue

seconds = int(globals().get('CAPTURE_SECONDS', 30))
if seconds < 0:
    raise ValueError('Capture duration outside bounds')
namespace = '/mi_desktop_48_b0_2d_7b_0b_4a'
rclpy.init(args=[])
node = rclpy.create_node('readonly_static_reconstruction', enable_rosout=False, start_parameter_services=False)
buffer = Buffer()
listener = TransformListener(buffer, node)
pose_source=ReconstructionPose(node,buffer,namespace,globals().get('POSE_FRAME'))
queue = FrameQueue()
calibration = [None]
last_sample = [0.]
counts = {'images': 0, 'sent': 0, 'bad_time_or_calibration': 0, 'tf_retries': 0}
colors = deque(maxlen=30)
last_color_saved = [0.]
counts.update(color_images=0, color_pairs=0, color_pair_misses=0)


def receive_color(msg):
    if (msg.encoding not in ('rgb8','bgr8') or not 1 <= msg.width <= 1280
            or not 1 <= msg.height <= 1024 or msg.step < msg.width*3
            or len(msg.data) != msg.step*msg.height or len(msg.data)>4*1024*1024):return
    stamp=Time.from_msg(msg.header.stamp).nanoseconds/1e9
    age=node.get_clock().now().nanoseconds/1e9-stamp
    if not -.05<=age<=.7:return
    colors.append((stamp,msg));counts['color_images']+=1


def color_pair(stamp):
    if stamp-last_color_saved[0]<1.:return None
    candidate=min(colors,key=lambda c:abs(c[0]-stamp)) if colors else None
    if candidate is None or abs(candidate[0]-stamp)>.05:
        counts['color_pair_misses']+=1;return None
    cs,msg=candidate
    last_color_saved[0]=stamp;counts['color_pairs']+=1
    return {'stamp':cs,'depth_stamp':stamp,'difference_s':abs(cs-stamp),
            'topic':namespace+'/image','frame_id':msg.header.frame_id,
            'width':msg.width,'height':msg.height,'step':msg.step,'encoding':msg.encoding,
            'data_zlib_b64':base64.b64encode(zlib.compress(bytes(msg.data),1)).decode('ascii'),
            'hardware_sync_verified':False,'spatial_registration_verified':False}


def emit(data):
    print(json.dumps(data, allow_nan=False, separators=(',', ':')), flush=True)


def retain_unposed(item, reason):
    msg,info=item['image'],item['info']
    emit(dict(event='unposed_depth',reason=reason,stamp=item['stamp'],source_frame=msg.header.frame_id,
              width=int(msg.width),height=int(msg.height),step=int(msg.step),encoding=msg.encoding,
              bigendian=bool(msg.is_bigendian),intrinsics=[float(info.p[i]) for i in (0,5,2,6)],
              depth_zlib_b64=base64.b64encode(zlib.compress(bytes(msg.data),1)).decode('ascii'),
              color_sample=color_pair(item['stamp'])))
queue.on_discard=retain_unposed


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
        tf = pose_source.lookup(msg.header.frame_id,stamp)
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
        node.create_subscription(Image, namespace+'/image', receive_color, qos_profile_sensor_data),
        node.create_subscription(Image, namespace+'/camera/depth/image_rect_raw', receive_depth, qos_profile_sensor_data)]
started = time.monotonic()
last_sent = started
last_warning = started
emit({'event': 'started', 'frame': pose_source.frame, 'seconds': seconds, 'control_requests': 0})
try:
    while seconds == 0 or time.monotonic()-started < seconds:
        rclpy.spin_once(node, timeout_sec=.02)
        for item, pose in queue.ready(lookup, time.monotonic()):
            msg, info = item['image'], item['info']
            if any(not math.isfinite(float(x)) for x in info.p) or len(info.p) != 12 or info.p[3] != 0 or info.p[7] != 0:
                counts['bad_time_or_calibration'] += 1
                continue
            emit(dict(pose, event='depth', stamp=item['stamp'], frame=pose_source.frame,
                      source_frame=msg.header.frame_id, width=int(msg.width), height=int(msg.height),
                      step=int(msg.step), encoding=msg.encoding, bigendian=bool(msg.is_bigendian),
                      intrinsics=[float(info.p[i]) for i in (0,5,2,6)],
                      depth_zlib_b64=base64.b64encode(zlib.compress(bytes(msg.data), 1)).decode('ascii'),
                      color_sample=color_pair(item['stamp']),
                      source_age_s=(node.get_clock().now().nanoseconds-Time.from_msg(msg.header.stamp).nanoseconds)*1e-9))
            counts['sent'] += 1
            last_sent = time.monotonic()
        if time.monotonic()-started > 6 and time.monotonic()-last_sent > 3 and time.monotonic()-last_warning > 5:
            message = ('深度相机没有输出图像；请检查原厂相机状态，先不要移动' if counts['images'] == 0 else
                       '原始深度持续保存；尚无同时间建图/视觉里程计位姿，融合等待恢复')
            emit({'event':'waiting','message':message,'counts':counts,'control_requests':0})
            last_warning = time.monotonic()
    for _,item in queue.pending:retain_unposed(item,'capture_finished_before_pose')
    queue.pending=[]
    emit({'event': 'finished', 'counts': counts, 'queue_dropped': queue.dropped,
          'queue_expired': queue.expired, 'queue_pending': len(queue.pending), 'control_requests': 0})
finally:
    node.destroy_node()
    rclpy.shutdown()
