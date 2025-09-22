#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

from custom_interface.msg import TrackedConeArray


class ConeVisualizer(Node):
    def __init__(self):
        super().__init__('cone_visualizer')

        # QoS aligned with upstream topic (often BEST_EFFORT for perception)
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        # Parameters
        self.declare_parameter('topic', '/cone/fused')
        self.declare_parameter('frame_id_override', '')  # if non-empty, override incoming frame
        self.declare_parameter('scale', 0.3)             # sphere diameter
        self.declare_parameter('alpha', 0.9)             # marker alpha
        self.declare_parameter('publish_text_ids', False)

        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.sub_ = self.create_subscription(TrackedConeArray, topic, self.on_cones, qos)
        self.pub_ = self.create_publisher(MarkerArray, '/cones/markers', 10)

        self.get_logger().info(f'ConeVisualizer subscribing {topic} -> /cones/markers')

    def on_cones(self, msg: TrackedConeArray):
        frame = self.get_parameter('frame_id_override').get_parameter_value().string_value or msg.header.frame_id or 'base_link'
        scale = float(self.get_parameter('scale').get_parameter_value().double_value)
        alpha = float(self.get_parameter('alpha').get_parameter_value().double_value)
        show_ids = bool(self.get_parameter('publish_text_ids').get_parameter_value().bool_value)

        # Group cones by color string
        groups = {
            'blue cone': [],
            'yellow cone': [],
            'red cone': [],
            'unknown': [],
        }
        for c in msg.cones:
            color = getattr(c, 'color', '')
            pt = Point(x=float(c.position.x), y=float(c.position.y), z=float(c.position.z))
            if color in groups:
                groups[color].append((pt, getattr(c, 'track_id', 0)))
            else:
                groups['Other'].append((pt, getattr(c, 'track_id', 0)))

        arr = MarkerArray()

        def add_spheres(ns: str, mid: int, points_with_id, rgba: ColorRGBA):
            m = Marker()
            m.header.frame_id = frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = ns
            m.id = mid
            if points_with_id:
                m.type = Marker.SPHERE_LIST
                m.action = Marker.ADD
                m.scale.x = scale
                m.scale.y = scale
                m.scale.z = scale
                m.pose.orientation.w = 1.0
                m.color = rgba
                m.points = [p for (p, _) in points_with_id]
            else:
                # Clear previously drawn marker if now empty
                m.type = Marker.SPHERE_LIST
                m.action = Marker.DELETE
            arr.markers.append(m)

        def col(r, g, b, a):
            return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))

        add_spheres('cones_blue',   0, groups['blue cone'],   col(0.10, 0.60, 1.00, alpha))
        add_spheres('cones_yellow', 1, groups['yellow cone'], col(1.00, 0.85, 0.10, alpha))
        add_spheres('cones_red',    2, groups['red cone'],    col(1.00, 0.20, 0.20, alpha))
        add_spheres('cones_other',  3, groups['unknown'],       col(0.70, 0.70, 0.70, alpha))

        if show_ids:
            # Publish a few text labels (optional, can be heavy if too many)
            tid = 1000
            for key, points in groups.items():
                for p, track_id in points:
                    t = Marker()
                    t.header.frame_id = frame
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.ns = 'cone_ids'
                    t.id = tid; tid += 1
                    t.type = Marker.TEXT_VIEW_FACING
                    t.action = Marker.ADD
                    t.scale.z = 0.35
                    t.pose.position = p
                    t.pose.position.z += 0.5
                    t.pose.orientation.w = 1.0
                    t.color = col(1.0, 1.0, 1.0, 0.9)
                    t.text = str(track_id)
                    arr.markers.append(t)

        self.pub_.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = ConeVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

