#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Visualise /sorted_cones_time_ukf (TrackedConeArray) in RViz2.

* frame      : msg.header.frame_id (fallback → 파라미터 frame_id)
* coordinates: cones[i].position (geometry_msgs/Point; x,y,z는 float64)
* colour     : cones[i].color (Unknown → 회색, 그 외 간단 매핑)
* id         : cones[i].track_id (0 또는 미지정 시 인덱스로 대체)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data  # BEST_EFFORT(depth=10)

from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration

# NEW: 새로운 커스텀 메시지 사용
from custom_interface.msg import TrackedConeArray


class ConeVisualizer(Node):
    def __init__(self) -> None:
        super().__init__('cone_visualizer_no_color')

        # ── parameters ──────────────────────────────────
        # 토픽명 기본값을 새 토픽으로 변경
        self.declare_parameter('topic_name', '/sorted_cones_time_ukf')
        self.declare_parameter('frame_id', 'os_sensor')     # fallback frame
        self.declare_parameter('lifetime_sec', 0.3)
        self.declare_parameter('scale', 0.50)               # sphere diameter [m]

        topic_name = self.get_parameter('topic_name').value
        self.frame = self.get_parameter('frame_id').value
        self.scale = float(self.get_parameter('scale').value)

        life_f = float(self.get_parameter('lifetime_sec').value)
        self.lifetime = Duration(
            sec=int(life_f),
            nanosec=int((life_f - int(life_f)) * 1e9)
        )

        # ── publishers / subscribers ────────────────────
        self.pub = self.create_publisher(MarkerArray, '/cones_marker_array', 10)

        # 새 메시지 타입으로 구독
        self.create_subscription(
            TrackedConeArray,
            topic_name,
            self.cb_cones,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            f'Subscribed to {topic_name}  →  publishing /cones_marker_array '
            f'(fallback frame “{self.frame}”).'
        )

    # --------------------------------------------------
    def cb_cones(self, msg: TrackedConeArray) -> None:
        cones = msg.cones
        if not cones:
            return  # nothing to show

        # 메시지 헤더의 프레임이 있으면 우선 사용
        frame_id = msg.header.frame_id if msg.header.frame_id else self.frame
        now = self.get_clock().now().to_msg()

        # 현재 프레임을 한 번 비우고(잔상 방지) 새 마커 발행
        clear = Marker()
        clear.action = Marker.DELETEALL
        self.pub.publish(MarkerArray(markers=[clear]))

        arr = MarkerArray()

        for idx, c in enumerate(cones):
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = frame_id
            m.ns = 'cones'
            # track_id가 0이거나 미정일 수 있으니 인덱스로 보정
            m.id = int(c.track_id) if getattr(c, 'track_id', 0) != 0 else idx

            m.type = Marker.SPHERE
            m.action = Marker.ADD

            # 위치
            m.pose.position.x = float(c.position.x)
            m.pose.position.y = float(c.position.y)
            m.pose.position.z = float(c.position.z)
            m.pose.orientation.w = 1.0

            # 스케일
            m.scale.x = m.scale.y = m.scale.z = self.scale

            # 색상: Unknown → 회색, 간단 매핑(필요시 확장)
            name = (getattr(c, 'color', 'Unknown') or 'Unknown').lower()
            if name == 'unknown':
                m.color.r = m.color.g = m.color.b = 0.6
            elif 'blue' in name:
                m.color.r, m.color.g, m.color.b = 0.2, 0.4, 1.0
            elif 'yellow' in name:
                m.color.r, m.color.g, m.color.b = 1.0, 1.0, 0.2
            elif 'orange' in name:
                m.color.r, m.color.g, m.color.b = 1.0, 0.5, 0.0
            else:
                # 알려지지 않은 지정 색상 문자열 → 기본 회색
                m.color.r = m.color.g = m.color.b = 0.6

            m.color.a = 1.0
            m.lifetime = self.lifetime

            arr.markers.append(m)

        self.pub.publish(arr)


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
