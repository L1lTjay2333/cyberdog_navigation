"""Transient ROS subscriber: raw depth and genuine RGB/BGR images every 5 s.

Python 3.6 compatible. No services, motion, lifecycle changes or robot files.
This is a sensor sample, not synchronized RGB-D or a registered room map.
"""
import base64
import json
import time
from collections import deque
import rclpy
from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import qos_profile_sensor_data


def main():
    duration = float(globals().get('CAPTURE_SECONDS', 16))
    if not 1 <= duration <= 120:
        raise ValueError('Sample duration must be 1..120 seconds')
    ns = '/mi_desktop_48_b0_2d_7b_0b_4a'
    rclpy.init(args=[])
    node = rclpy.create_node('rgbd_five_second_sample', enable_rosout=False,
                             start_parameter_services=False)
    counts = {}; saved = {}; last = {}; calibration = {}; errors = {}
    subscriptions = []
    paired = bool(globals().get('PAIR_RGBD', False))
    depths = deque(maxlen=20)
    pending = []
    pair_stats = {'matched': 0, 'rejected': 0, 'max_difference_s': .05}

    def stamp_of(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

    def info(topic, msg):
        calibration[topic] = {'frame_id': msg.header.frame_id, 'width': msg.width,
            'height': msg.height, 'p': list(msg.p), 'k': list(msg.k), 'd': list(msg.d),
            'distortion_model': msg.distortion_model,
            'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9}

    def prepare_row(topic, msg, extra=None):
        stamp = stamp_of(msg)
        age = node.get_clock().now().nanoseconds / 1e9 - stamp
        if not -.1 <= age <= 1.0:
            errors['stale_or_invalid_stamp'] = errors.get('stale_or_invalid_stamp', 0) + 1
            return
        if msg.encoding not in ('rgb8', 'bgr8', 'rgba8', 'bgra8', '16UC1', '32FC1'):
            errors['encoding:' + msg.encoding] = errors.get('encoding:' + msg.encoding, 0) + 1
            return
        if not 0 < len(msg.data) <= 8 * 1024 * 1024:
            return
        row = {'kind': 'image', 'topic': topic, 'stamp': stamp, 'age_s': age,
            'frame_id': msg.header.frame_id, 'width': msg.width, 'height': msg.height,
            'encoding': msg.encoding, 'step': msg.step, 'is_bigendian': msg.is_bigendian,
            'calibration': [v for v in calibration.values() if v['frame_id'] == msg.header.frame_id
                and v['width'] == msg.width and v['height'] == msg.height],
            'data': base64.b64encode(bytes(msg.data)).decode('ascii')}
        row.update(extra or {})
        return row

    def write_row(row):
        print(json.dumps(row), flush=True)
        topic = row['topic']
        saved[topic] = saved.get(topic, 0) + 1

    def emit(topic, msg, extra=None):
        row = prepare_row(topic, msg, extra)
        if row is not None:
            write_row(row)

    def receive(topic, msg):
        counts[topic] = counts.get(topic, 0) + 1
        now = time.monotonic()
        if paired and topic.endswith('/depth/image_rect_raw'):
            depths.append(msg)
            return
        if now - last.get(topic, -1e9) < 5:
            return
        if paired:
            if msg.encoding not in ('rgb8', 'bgr8', 'rgba8', 'bgra8'):
                return
            pending.append((now, topic, msg))
        else:
            emit(topic, msg)
        last[topic] = now

    def flush_pairs(force=False):
        while pending and (force or time.monotonic() - pending[0][0] >= .15):
            _, topic, color = pending.pop(0)
            depth = min(depths, key=lambda m: abs(stamp_of(m)-stamp_of(color))) if depths else None
            delta = abs(stamp_of(depth)-stamp_of(color)) if depth is not None else float('inf')
            if delta > pair_stats['max_difference_s']:
                pair_stats['rejected'] += 1
                continue
            pair_id = pair_stats['matched']
            extra = {'pair_id': pair_id, 'pair_difference_s': delta,
                     'pair_method': 'nearest_buffered_timestamp_not_hardware_sync'}
            color_row = prepare_row(topic, color, extra)
            depth_row = prepare_row(ns + '/camera/depth/image_rect_raw', depth, extra)
            if color_row is None or depth_row is None:
                pair_stats['rejected'] += 1
                continue
            write_row(color_row)
            write_row(depth_row)
            pair_stats['matched'] += 1

    try:
        until = time.monotonic() + 3
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
        topics = node.get_topic_names_and_types()
        # CameraInfo is small; collect actual calibration topics without changing drivers.
        for topic, types in topics:
            if topic.startswith(ns + '/') and 'sensor_msgs/msg/CameraInfo' in types:
                subscriptions.append(node.create_subscription(CameraInfo, topic,
                    lambda m, t=topic: info(t, m), qos_profile_sensor_data))
        candidates = [ns + '/camera/depth/image_rect_raw', ns + '/image',
                      ns + '/image_rgb', ns + '/camera/color/image_raw']
        for topic in candidates:
            subscriptions.append(node.create_subscription(Image, topic,
                lambda m, t=topic: receive(t, m), qos_profile_sensor_data))
        until = time.monotonic() + duration
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.03)
            flush_pairs()
        flush_pairs(force=True)
        print(json.dumps({'kind': 'summary', 'counts': counts, 'saved': saved,
            'errors': errors, 'interval_s': 5, 'motion_requests': 0,
            'spatial_registration_verified': False, 'calibration': calibration,
            'paired_mode': paired, 'pair_stats': pair_stats}), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
