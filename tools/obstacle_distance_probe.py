#!/usr/bin/env python3
"""Read-only ROS 2 LaserScan/PointCloud2 distance diagnostics, Python 3.6+."""
import argparse
import json
import math
import time

from cloud_geometry import decode_xyz, transform_xyz


def scan_points(msg):
    if not all(math.isfinite(v) for v in (msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)):
        raise ValueError('Invalid scan metadata')
    if msg.angle_increment == 0 or msg.range_min < 0 or msg.range_max <= msg.range_min:
        raise ValueError('Invalid scan limits')
    out = []
    for i, r in enumerate(msg.ranges):
        if math.isfinite(r) and r > 0 and msg.range_min <= r <= msg.range_max:
            a = msg.angle_min + i * msg.angle_increment
            out.append([r * math.cos(a), r * math.sin(a), 0.0])
    return out


def quantile(values, q):
    ordered = sorted(values)
    at = (len(ordered)-1) * q
    lo = int(at)
    hi = min(lo+1, len(ordered)-1)
    return ordered[lo] + (ordered[hi]-ordered[lo])*(at-lo)


def summarize(points, bounds, minimum_points):
    selected = [p for p in points if len(p) == 3 and all(math.isfinite(v) for v in p)
                and any(v != 0 for v in p)
                and all(bounds[2*i] <= p[i] <= bounds[2*i+1] for i in range(3))]
    report = {'valid': len(selected) >= minimum_points, 'roi_points': len(selected),
              'minimum_points': minimum_points, 'roi_m': list(bounds)}
    if not report['valid']:
        report['reason'] = 'insufficient_points_in_roi'
        return report
    xs = [p[0] for p in selected]
    report.update(forward_x_min_m=min(xs), forward_x_p10_m=quantile(xs, .1),
                  forward_x_median_m=quantile(xs, .5), forward_x_p90_m=quantile(xs, .9),
                  horizontal_range_min_m=min(math.hypot(p[0], p[1]) for p in selected),
                  spread_p90_minus_p10_m=quantile(xs, .9)-quantile(xs, .1),
                  interpretation='ROI statistics; no obstacle identity or accuracy guarantee')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', required=True, choices=('scan', 'cloud'))
    parser.add_argument('--topic', required=True)
    parser.add_argument('--target-frame', default='base_link', help='Actual body frame with X forward; exact-stamp TF required')
    parser.add_argument('--roi', type=float, nargs=6, default=[.1, 5., -.3, .3, -.1, 1.], metavar=('XMIN','XMAX','YMIN','YMAX','ZMIN','ZMAX'))
    parser.add_argument('--minimum-points', type=int, default=3)
    parser.add_argument('--max-points', type=int, default=250000)
    parser.add_argument('--seconds', type=float, default=30.)
    parser.add_argument('--max-age', type=float, default=1.)
    args = parser.parse_args()
    if not args.topic.startswith('/') or not args.target_frame:
        parser.error('Use an absolute topic and explicit target frame')
    if not all(math.isfinite(v) for v in args.roi) or any(args.roi[i] >= args.roi[i+1] for i in (0,2,4)):
        parser.error('Invalid ROI bounds')
    if args.roi[0] < 0 or args.minimum_points < 1 or args.max_points < 1 or not all(math.isfinite(v) and v > 0 for v in (args.seconds,args.max_age)):
        parser.error('Positive limits and a forward ROI are required')

    import rclpy
    from rclpy.duration import Duration
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan, PointCloud2
    from tf2_ros import Buffer, TransformListener
    rclpy.init(args=[])
    node = rclpy.create_node('readonly_obstacle_distance_probe')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    observed = [None]
    printed = [0.]

    def emit(data):
        print(json.dumps(data, allow_nan=False), flush=True)

    def callback(msg):
        observed[0] = time.monotonic()
        if observed[0]-printed[0] < .5:
            return
        printed[0] = observed[0]
        data = {'topic': args.topic, 'mode': args.mode, 'source_frame': msg.header.frame_id,
                'target_frame': args.target_frame, 'units_assumption': 'ROS SI meters',
                'source_stamp': {'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec},
                'motion_compensated': False, 'control_requests': 0}
        try:
            stamp = Time.from_msg(msg.header.stamp)
            age = (node.get_clock().now().nanoseconds-stamp.nanoseconds)*1e-9
            data['source_age_s'] = age
            if not msg.header.frame_id or stamp.nanoseconds == 0 or age < -.05 or age > args.max_age:
                raise ValueError('Missing frame, zero stamp, stale data or clock mismatch')
            pts = scan_points(msg) if args.mode == 'scan' else decode_xyz(msg, args.max_points)
            pts = [p for p in pts if any(v != 0 for v in p)]
            if msg.header.frame_id != args.target_frame:
                tf = buffer.lookup_transform(args.target_frame, msg.header.frame_id, stamp, timeout=Duration(seconds=.05)).transform
                pts = transform_xyz(pts, [tf.translation.x,tf.translation.y,tf.translation.z],
                                    [tf.rotation.x,tf.rotation.y,tf.rotation.z,tf.rotation.w])
            data.update(summarize(pts, args.roi, args.minimum_points))
        except Exception as exc:
            data.update(valid=False, reason=str(exc))
        emit(data)

    subscription = node.create_subscription(LaserScan if args.mode == 'scan' else PointCloud2,
                                            args.topic, callback, qos_profile_sensor_data)
    start = time.monotonic()
    warned = start
    try:
        while rclpy.ok() and time.monotonic()-start < args.seconds:
            rclpy.spin_once(node, timeout_sec=.1)
            now = time.monotonic()
            if now-warned >= 2. and (observed[0] is None or now-observed[0] > args.max_age):
                emit({'valid': False, 'reason': 'no_fresh_messages', 'topic': args.topic, 'control_requests': 0})
                warned = now
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
