#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Visualise /cone/lidar (TrackedConeArray) in RViz2.

* frame      : os_sensor (header.frame_id)
* coordinates: (x, y, z) triplets in msg.data
* colour     : all cones → solid grey
"""

import rclpy
from rclpy.node import Node
from rclpy.qos  import qos_profile_sensor_data          # BEST_EFFORT (depth = 10)

from visualization_msgs.msg import Marker, MarkerArray
from custom_interface.msg import TrackedConeArray
from builtin_interfaces.msg import Duration


class ConeVisualizer(Node):
    def __init__(self) -> None:
        super().__init__('cone_visualizer_no_color')

        # ── parameters ──────────────────────────────────
        self.declare_parameter('topic_name', '/cone/lidar')
        self.declare_parameter('frame_id',    'os_sensor')
        self.declare_parameter('lifetime_sec', 0.3)
        self.declare_parameter('scale',        0.50)      # sphere diameter [m]

        topic_name = self.get_parameter('topic_name').value
        self.frame = self.get_parameter('frame_id').value
        self.scale = float(self.get_parameter('scale').value)

        life_f = float(self.get_parameter('lifetime_sec').value)
        self.lifetime = Duration(
            sec=int(life_f),
            nanosec=int((life_f - int(life_f)) * 1e9))

        # ── publishers / subscribers ────────────────────
        self.pub = self.create_publisher(
            MarkerArray, '/cones_marker_array', 10)

        self.create_subscription(
            TrackedConeArray,
            topic_name,
            self.cb_cones,
            qos_profile_sensor_data)

        self.get_logger().info(
            f'Subscribed to {topic_name}  →  publishing /cones_marker_array '
            f'(frame “{self.frame}”).')

    # --------------------------------------------------
    def cb_cones(self, msg: TrackedConeArray) -> None:
        if not msg.cones:
            return                           # nothing to show
        n_cones = len(msg.cones)

        now = self.get_clock().now().to_msg()
        arr = MarkerArray()

        for idx, cone in enumerate(msg.cones):
            x = cone.position.x
            y = cone.position.y
            z = cone.position.z

            m = Marker()
            m.header.stamp    = now
            m.header.frame_id = self.frame
            m.ns   = 'cones_gray'
            m.id   = idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = float(z)
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = self.scale
            # solid grey
            m.color.r = m.color.g = m.color.b = 0.6
            m.color.a = 1.0
            m.lifetime = self.lifetime

            arr.markers.append(m)

        # DELETEALL → 새 ADD 마커로 덮어쓰기 (잔상 방지)
        clear = Marker()
        clear.action = Marker.DELETEALL
        self.pub.publish(MarkerArray(markers=[clear]))  # clear first
        self.pub.publish(arr)                           # publish current cones


# ------------------------------------------------------
def main(args=None) -> None:
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