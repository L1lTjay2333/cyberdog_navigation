#!/usr/bin/env python3
"""Republish the read-only SSH map snapshot into WSL-local ROS for RViz2."""
import json
import math
from pathlib import Path

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


SNAPSHOT = Path('/mnt/d/cyberdog/cyberdog_navigation/evidence/latest_map_snapshot.json')


def main():
    rclpy.init(args=None)
    node = rclpy.create_node('cyberdog_readonly_map_bridge')
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    publisher = node.create_publisher(OccupancyGrid, '/cyberdog_live_map', qos)
    free_publisher = node.create_publisher(Marker, '/cyberdog_live_free', qos)
    obstacle_publisher = node.create_publisher(Marker, '/cyberdog_live_obstacles', qos)
    last_signature = None

    def publish_snapshot():
        nonlocal last_signature
        try:
            raw = SNAPSHOT.read_text(encoding='utf-8')
            state = json.loads(raw)
            grid = state.get('map')
            if not grid:
                return
            width, height = int(grid['width']), int(grid['height'])
            resolution = float(grid['resolution'])
            data = grid['data']
            signature = (grid.get('received_unix'), width, height, len(data))
            if signature == last_signature:
                return
            if width <= 0 or height <= 0 or len(data) != width * height:
                node.get_logger().error('Rejected inconsistent occupancy grid')
                return
            if not math.isfinite(resolution) or resolution <= 0:
                node.get_logger().error('Rejected invalid map resolution')
                return
            msg = OccupancyGrid()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = str(grid.get('frame') or 'laser_odom')
            msg.info.map_load_time = msg.header.stamp
            msg.info.resolution = resolution
            msg.info.width = width
            msg.info.height = height
            origin = grid.get('origin') or {}
            msg.info.origin.position.x = float(origin.get('x', 0.0))
            msg.info.origin.position.y = float(origin.get('y', 0.0))
            msg.info.origin.orientation.z = float(origin.get('qz', 0.0))
            msg.info.origin.orientation.w = float(origin.get('qw', 1.0))
            msg.data = [int(value) for value in data]
            publisher.publish(msg)
            yaw = 2.0 * math.atan2(msg.info.origin.orientation.z,
                                   msg.info.origin.orientation.w)
            cosine, sine = math.cos(yaw), math.sin(yaw)
            free = Marker()
            free.header = msg.header
            free.ns = 'cyberdog_map'
            free.id = 1
            free.type = Marker.POINTS
            free.action = Marker.ADD
            free.pose.orientation.w = 1.0
            free.scale.x = resolution * 0.92
            free.scale.y = resolution * 0.92
            free.color.r = 0.82
            free.color.g = 0.86
            free.color.b = 0.90
            free.color.a = 1.0
            occupied = Marker()
            occupied.header = msg.header
            occupied.ns = 'cyberdog_map'
            occupied.id = 2
            occupied.type = Marker.POINTS
            occupied.action = Marker.ADD
            occupied.pose.orientation.w = 1.0
            occupied.scale.x = resolution
            occupied.scale.y = resolution
            occupied.color.r = 0.04
            occupied.color.g = 0.08
            occupied.color.b = 0.12
            occupied.color.a = 1.0
            origin_x = msg.info.origin.position.x
            origin_y = msg.info.origin.position.y
            for index, value in enumerate(data):
                if value < 0:
                    continue
                col, row = index % width, index // width
                local_x = (col + 0.5) * resolution
                local_y = (row + 0.5) * resolution
                point = Point()
                point.x = origin_x + cosine * local_x - sine * local_y
                point.y = origin_y + sine * local_x + cosine * local_y
                point.z = 0.0
                if value >= 65:
                    occupied.points.append(point)
                else:
                    free.points.append(point)
            free_publisher.publish(free)
            obstacle_publisher.publish(occupied)
            last_signature = signature
            node.get_logger().info(
                'Published live map: {}x{} at {:.3f} m/cell, {} free, {} occupied'.format(
                    width, height, resolution, len(free.points), len(occupied.points)))
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
            node.get_logger().warning('Waiting for valid map snapshot: {}'.format(exc))

    node.create_timer(0.5, publish_snapshot)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
