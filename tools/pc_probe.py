#!/usr/bin/env python3
"""Passive PointCloud2 diagnostic. Python 3.6+, stdlib geometry, no controls."""
import argparse
import json
import math
import struct
import sys
import time


def inspect_cloud(msg, max_points=10000):
    """Inspect evenly sampled XYZ, respecting endianness and row padding.

    Coordinates remain in source units/frame. Variation is NOT proof of stairs
    or of volumetric geometry; a tilted plane can vary along all three axes.
    """
    if max_points < 1:
        raise ValueError('max_points must be positive')
    width, height = int(msg.width), int(msg.height)
    step, row = int(msg.point_step), int(msg.row_step)
    if min(width, height, row) < 0 or step <= 0:
        raise ValueError('invalid dimensions/point_step')
    if row < width * step or len(msg.data) != row * height:
        raise ValueError('inconsistent row_step/data length')
    fields = {}
    for field in msg.fields:
        if field.name in fields:
            raise ValueError('duplicate field name')
        fields[field.name] = field
    readers = []
    for name in ('x', 'y', 'z'):
        field = fields.get(name)
        if field is None or field.count != 1 or field.datatype not in (7, 8):
            raise ValueError('XYZ must each be scalar FLOAT32 or FLOAT64')
        reader = struct.Struct(('>' if msg.is_bigendian else '<') +
                               ('f' if field.datatype == 7 else 'd'))
        if field.offset < 0 or field.offset + reader.size > step:
            raise ValueError('XYZ field exceeds point_step')
        readers.append((field.offset, reader))
    count = width * height
    sampled = min(count, max_points)
    finite = 0
    minimum, maximum = [float('inf')] * 3, [-float('inf')] * 3
    buf = memoryview(msg.data)
    for i in range(sampled):
        index = i * count // sampled
        base = (index // width) * row + (index % width) * step
        xyz = [reader.unpack_from(buf, base + offset)[0]
               for offset, reader in readers]
        if not all(math.isfinite(value) for value in xyz):
            continue
        finite += 1
        for axis, value in enumerate(xyz):
            minimum[axis] = min(minimum[axis], value)
            maximum[axis] = max(maximum[axis], value)
    return dict(frame_id=msg.header.frame_id,
                stamp_ns=int(msg.header.stamp.sec) * 1000000000 +
                int(msg.header.stamp.nanosec),
                width=width, height=height, point_count=count,
                point_step=step, row_step=row, data_bytes=len(msg.data),
                is_bigendian=bool(msg.is_bigendian), is_dense=bool(msg.is_dense),
                fields=[dict(name=f.name, offset=f.offset, datatype=f.datatype,
                             count=f.count) for f in msg.fields],
                sampled_points=sampled, sampled_finite_points=finite,
                sampled_invalid_points=sampled - finite,
                xyz_min=minimum if finite else None,
                xyz_max=maximum if finite else None,
                xyz_span=[b-a for a, b in zip(minimum, maximum)] if finite else None,
                units='UNVERIFIED_SOURCE_UNITS', coordinates='UNCHANGED_SOURCE_FRAME')


def summarize(frames):
    good = [f for f in frames if 'error' not in f]
    stamps = [f['stamp_ns'] for f in good]
    elapsed = frames[-1]['receive_monotonic'] - frames[0]['receive_monotonic'] if len(frames) > 1 else 0
    return dict(received_frames=len(frames), valid_layout_frames=len(good),
                frames_with_finite_xyz=sum(f['sampled_finite_points'] > 0 for f in good),
                receive_hz=(len(frames)-1)/elapsed if elapsed > 0 else None,
                duplicate_stamps=sum(b == a for a, b in zip(stamps, stamps[1:])),
                backwards_stamps=sum(b < a for a, b in zip(stamps, stamps[1:])),
                note='Receipt rate is not nominal sensor FPS. No TF/units/geometry acceptance implied.')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', required=True, help='Actual absolute PointCloud2 topic')
    parser.add_argument('--seconds', type=float, default=15.0)
    parser.add_argument('--frames', type=int, default=10)
    parser.add_argument('--max-points', type=int, default=10000)
    parser.add_argument('--output', help='New JSON file; refuses overwrite')
    args = parser.parse_args(argv)
    if not args.topic.startswith('/') or not math.isfinite(args.seconds) or not 0 < args.seconds <= 60:
        parser.error('absolute topic and seconds in (0, 60] required')
    if not 1 <= args.frames <= 100 or not 1 <= args.max_points <= 307200:
        parser.error('frames must be 1..100; max-points 1..307200')
    # Lazy ROS imports: offline tests and --help do not need ROS installed.
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import PointCloud2
    output = open(args.output, 'x') if args.output else None
    node = None
    frames = []
    report = dict(topic=args.topic, requested_seconds=args.seconds,
                  requested_frames=args.frames, subscriber_qos='BEST_EFFORT/VOLATILE/depth=1',
                  publisher_qos='UNKNOWN; obtain separately with ros2 topic info --verbose',
                  frames=frames)
    initialized = False
    try:
        rclpy.init(args=[])
        initialized = True
        node = rclpy.create_node('m1r_readonly_pc_probe', enable_rosout=False,
                                 start_parameter_services=False)
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE

        def receive(msg):
            started = time.monotonic()
            wall = time.time()
            try:
                result = inspect_cloud(msg, args.max_points)
                result['receipt_minus_stamp_seconds'] = wall - result['stamp_ns'] / 1e9
            except Exception as exc:
                result = dict(error=str(exc))
            result.update(receive_monotonic=started, receive_wall_unix=wall,
                          inspect_ms=(time.monotonic()-started)*1000)
            frames.append(result)

        subscription = node.create_subscription(PointCloud2, args.topic, receive, qos)
        started = time.monotonic()
        while len(frames) < args.frames and time.monotonic()-started < args.seconds:
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0, args.seconds-(time.monotonic()-started))))
        report['status'] = 'CAPTURED' if frames else 'NO_MESSAGES'
    except KeyboardInterrupt:
        report['status'] = 'INTERRUPTED'
    except Exception as exc:
        report.update(status='ERROR', error=str(exc))
    finally:
        if node is not None:
            node.destroy_node()
        if initialized:
            rclpy.shutdown()
        report['summary'] = summarize(frames)
        encoded = json.dumps(report, indent=2, allow_nan=False)
        print(encoded)
        if output is not None:
            try:
                output.write(encoded + '\n')
            finally:
                output.close()
    if report['status'] == 'INTERRUPTED':
        return 130
    return 0 if report['status'] == 'CAPTURED' and frames and all(
        'error' not in f and f['sampled_finite_points'] > 0 for f in frames) else 2


if __name__ == '__main__':
    sys.exit(main())
