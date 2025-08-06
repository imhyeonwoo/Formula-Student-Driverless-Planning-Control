#!/usr/bin/env python3
# file: car_marker_publisher.py
import math
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.duration import Duration
from visualization_msgs.msg import Marker, MarkerArray


class CarMarkerPublisher(Node):
    def __init__(self) -> None:
        super().__init__('car_marker_publisher')

        # ===== 프레임 =====
        self.declare_parameter('frame_id', 'base_link')  # base_link = 후륜축 중심
        self.frame_id: str = self.get_parameter('frame_id').get_parameter_value().string_value

        # ===== 차체/섀시 치수 (m) =====
        self.declare_parameter('body_length', 4.2)   # 범퍼~범퍼
        self.declare_parameter('body_width',  1.8)
        self.declare_parameter('body_height', 1.4)
        self.body_length: float = float(self.get_parameter('body_length').value)
        self.body_width:  float = float(self.get_parameter('body_width').value)
        self.body_height: float = float(self.get_parameter('body_height').value)

        self.declare_parameter('wheel_base',  2.6)   # 앞/뒤 축 간 거리
        self.declare_parameter('track_width', 1.6)   # 좌/우 바퀴 간 거리
        self.wheel_base:  float = float(self.get_parameter('wheel_base').value)
        self.track_width: float = float(self.get_parameter('track_width').value)

        # ===== 오버행 (m) =====
        # body_length = rear_overhang + wheel_base + front_overhang
        self.declare_parameter('front_overhang', None)
        self.declare_parameter('rear_overhang',  None)
        fo_param = self.get_parameter('front_overhang').value
        ro_param = self.get_parameter('rear_overhang').value

        if fo_param is None and ro_param is None:
            default_overhang = max(0.0, (self.body_length - self.wheel_base) / 2.0)
            self.front_overhang = default_overhang
            self.rear_overhang  = default_overhang
        elif fo_param is None:
            self.rear_overhang  = float(ro_param)
            self.front_overhang = max(0.0, self.body_length - self.wheel_base - self.rear_overhang)
        elif ro_param is None:
            self.front_overhang = float(fo_param)
            self.rear_overhang  = max(0.0, self.body_length - self.wheel_base - self.front_overhang)
        else:
            self.front_overhang = float(fo_param)
            self.rear_overhang  = float(ro_param)
            expected = self.rear_overhang + self.wheel_base + self.front_overhang
            if abs(expected - self.body_length) > 1e-3:
                self.get_logger().warn(
                    f'front_overhang + wheel_base + rear_overhang ({expected:.3f}) '
                    f'!= body_length ({self.body_length:.3f}). 표시만 진행합니다.'
                )

        # ===== 바퀴(실린더) 크기 (m) =====
        self.declare_parameter('wheel_radius', 0.30)
        self.declare_parameter('wheel_width',  0.20)
        self.wheel_radius: float = float(self.get_parameter('wheel_radius').value)
        self.wheel_width:  float = float(self.get_parameter('wheel_width').value)

        # ===== 높이 배치 (m) =====
        # 기본 규칙: body_z = wheel_radius + ground_clearance + body_height/2
        self.declare_parameter('ground_clearance', 0.12)
        gc = float(self.get_parameter('ground_clearance').value)

        self.declare_parameter('body_z',  None)   # 명시 시 우선
        self.declare_parameter('wheel_z', None)   # 명시 시 우선
        body_z_param = self.get_parameter('body_z').value
        wheel_z_param = self.get_parameter('wheel_z').value
        self.wheel_z: float = (self.wheel_radius) if wheel_z_param is None else float(wheel_z_param)
        self.body_z:  float = (self.wheel_radius + gc + self.body_height/2.0) \
                              if body_z_param is None else float(body_z_param)

        # ===== 색상 (0~1) =====
        self.declare_parameter('body_color_r', 0.0)
        self.declare_parameter('body_color_g', 0.45)
        self.declare_parameter('body_color_b', 1.0)
        self.declare_parameter('body_color_a', 0.5)
        self.declare_parameter('wheel_color_r', 0.05)
        self.declare_parameter('wheel_color_g', 0.05)
        self.declare_parameter('wheel_color_b', 0.05)
        self.declare_parameter('wheel_color_a', 1.0)

        self.body_color = (
            float(self.get_parameter('body_color_r').value),
            float(self.get_parameter('body_color_g').value),
            float(self.get_parameter('body_color_b').value),
            float(self.get_parameter('body_color_a').value),
        )
        self.wheel_color = (
            float(self.get_parameter('wheel_color_r').value),
            float(self.get_parameter('wheel_color_g').value),
            float(self.get_parameter('wheel_color_b').value),
            float(self.get_parameter('wheel_color_a').value),
        )

        # ===== 퍼블리시 주기 =====
        self.declare_parameter('rate_hz', 1.0)
        rate_hz: float = float(self.get_parameter('rate_hz').value)

        # QoS: Transient Local(신규 구독자에게 마지막 메시지 전달) + Reliable
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability  = DurabilityPolicy.TRANSIENT_LOCAL

        self.pub   = self.create_publisher(MarkerArray, 'car_marker', qos)
        self.timer = self.create_timer(1.0 / max(rate_hz, 1e-3), self._on_timer)

        # ===== 배치/겹침 검증 =====
        self._sanity_checks()

        self.get_logger().info(
            f'frame={self.frame_id} (rear-axle origin), '
            f'LxWxH={self.body_length:.2f}x{self.body_width:.2f}x{self.body_height:.2f}, '
            f'WB={self.wheel_base:.2f}, TW={self.track_width:.2f}, '
            f'OH(front/rear)={self.front_overhang:.2f}/{self.rear_overhang:.2f}, '
            f'body_z={self.body_z:.2f}, wheel_z={self.wheel_z:.2f}, gc={gc:.2f}'
        )

    # -------- Helpers --------
    def _sanity_checks(self):
        # z-겹침 체크: 차체 바닥 vs 바퀴 꼭대기
        body_bottom = self.body_z - self.body_height / 2.0
        wheel_top   = self.wheel_z + self.wheel_radius
        if body_bottom <= wheel_top + 1e-3:
            self.get_logger().warn(
                f'차체 바닥({body_bottom:.3f}) ≤ 바퀴 꼭대기({wheel_top:.3f}). '
                f'body_z(혹은 ground_clearance)를 키우세요.'
            )

        # 폭 여유 체크: 바퀴가 차체 밖으로 보이는지
        if (self.track_width + self.wheel_width) <= (self.body_width + 1e-3):
            self.get_logger().warn(
                '바퀴가 차체와 겹쳐 보일 수 있습니다. '
                f'track_width({self.track_width:.2f}) + wheel_width({self.wheel_width:.2f}) '
                f'<= body_width({self.body_width:.2f}). '
                'track_width 또는 wheel_width를 늘리거나 body_width를 줄이세요.'
            )

    def _quat_from_rpy(self, roll: float, pitch: float, yaw: float):
        cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5);   sy = math.sin(yaw * 0.5)
        w = cr*cp*cy + sr*sp*sy
        x = sr*cp*cy - cr*sp*sy
        y = cr*sp*cy + sr*cp*sy
        z = cr*cp*sy - sr*sp*cy
        return (x, y, z, w)

    def _make_body_marker(self, center_x: float) -> Marker:
        now = self.get_clock().now().to_msg()
        r, g, b, a = self.body_color

        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = now
        m.ns = 'car'
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD

        m.pose.position.x = center_x    # ★ 차체 중심 x (rear axle 기준)
        m.pose.position.y = 0.0
        m.pose.position.z = self.body_z
        m.pose.orientation.w = 1.0

        m.scale.x = self.body_length
        m.scale.y = self.body_width
        m.scale.z = self.body_height

        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, a
        m.lifetime = Duration(seconds=0.0).to_msg()
        m.frame_locked = True
        return m

    def _make_wheel_marker(self, mid: int, x: float, y: float, z: float) -> Marker:
        now = self.get_clock().now().to_msg()
        r, g, b, a = self.wheel_color

        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = now
        m.ns = 'car'
        m.id = mid
        m.type = Marker.CYLINDER
        m.action = Marker.ADD

        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z

        # RViz 실린더 축(Z) → 바퀴 축(Y)에 맞추기 위해 X=+90°
        qx, qy, qz, qw = self._quat_from_rpy(math.pi/2.0, 0.0, 0.0)
        m.pose.orientation.x = qx
        m.pose.orientation.y = qy
        m.pose.orientation.z = qz
        m.pose.orientation.w = qw

        # CYLINDER: scale.x/scale.y=지름, scale.z=축 방향 길이(=휠 폭)
        m.scale.x = self.wheel_radius * 2.0
        m.scale.y = self.wheel_radius * 2.0
        m.scale.z = self.wheel_width

        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, a
        m.lifetime = Duration(seconds=0.0).to_msg()
        m.frame_locked = True
        return m

    # -------- Timer --------
    def _on_timer(self) -> None:
        arr = MarkerArray()
        markers: List[Marker] = []

        # ★ base_link = rear axle center
        rx = 0.0                 # rear axle x
        fx = self.wheel_base     # front axle x
        ly = +self.track_width / 2.0
        ry = -self.track_width / 2.0

        # 차체 중심 x (rear axle 기준)
        body_center_x = (self.body_length / 2.0) - self.rear_overhang

        # 1) 차체
        markers.append(self._make_body_marker(body_center_x))

        # 2) 바퀴 4개
        markers.append(self._make_wheel_marker(1, fx, ly, self.wheel_z))  # FL
        markers.append(self._make_wheel_marker(2, fx, ry, self.wheel_z))  # FR
        markers.append(self._make_wheel_marker(3, rx, ly, self.wheel_z))  # RL
        markers.append(self._make_wheel_marker(4, rx, ry, self.wheel_z))  # RR

        arr.markers = markers
        self.pub.publish(arr)


def main():
    rclpy.init()
    node = CarMarkerPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
