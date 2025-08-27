#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from rviz_2d_overlay_msgs.msg import OverlayText


def quat_to_yaw_deg(q) -> float:
    """Quaternion → yaw(deg) (ENU 기준, z-축 회전)"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw_rad = math.atan2(siny_cosp, cosy_cosp)
    yaw_deg = math.degrees(yaw_rad)
    # [-180, 180]로 정규화
    if yaw_deg > 180.0:
        yaw_deg -= 360.0
    elif yaw_deg < -180.0:
        yaw_deg += 360.0
    return yaw_deg


class HudOverlayNode(Node):
    def __init__(self):
        super().__init__('hud_overlay_node')

        # ===== Publishers =====
        self.pub_text = self.create_publisher(OverlayText, '/hud/overlay_text', 10)

        # ===== Subscribers (Sensor QoS로 호환성 확보) =====
        qos = qos_profile_sensor_data
        self.create_subscription(Float32, '/current_speed', self.cb_current_speed, qos)
        self.create_subscription(Float32, '/cmd/speed', self.cb_target_speed, qos)
        self.create_subscription(Float32, '/ctrl/steer', self.cb_current_steer, qos)
        self.create_subscription(Float32, '/cmd/steer', self.cb_target_steer, qos)
        self.create_subscription(Imu, '/imu/processed', self.cb_imu, qos)  # yaw는 여기서 계산하지 않음
        self.create_subscription(Odometry, '/odometry/filtered', self.cb_odom, qos)

        # (선택) 보정 yaw Float32(rad)를 직접 쓰고 싶다면 주석 해제
        # self.create_subscription(Float32, '/global_yaw/complementary', self.cb_yaw_rad, qos)

        # ===== State =====
        self.current_speed = 0.0
        self.target_speed = 0.0
        self.current_steer = 0.0
        self.target_steer = 0.0
        self.yaw_deg = 0.0
        self.position = (0.0, 0.0)

        # ===== Timer =====
        self.create_timer(0.10, self.publish_overlay)  # 10Hz

        self.get_logger().info('HUD overlay node started (uses /odometry/filtered orientation for yaw).')

    # ---------- Callbacks ----------
    def cb_current_speed(self, msg: Float32):
        self.current_speed = float(msg.data)

    def cb_target_speed(self, msg: Float32):
        self.target_speed = float(msg.data)

    def cb_current_steer(self, msg: Float32):
        self.current_steer = float(msg.data)

    def cb_target_steer(self, msg: Float32):
        self.target_steer = float(msg.data)

    def cb_imu(self, msg: Imu):
        # IMU orientation은 초기화/보정 상태에 따라 신뢰도가 낮을 수 있으므로
        # yaw 계산은 /odometry/filtered의 orientation을 사용한다.
        pass

    def cb_odom(self, msg: Odometry):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.yaw_deg = quat_to_yaw_deg(msg.pose.pose.orientation)

    # (선택) /global_yaw/complementary를 직접 사용하려면
    def cb_yaw_rad(self, msg: Float32):
        self.yaw_deg = float(msg.data) * 180.0 / math.pi

    # ---------- Overlay ----------
    def publish_overlay(self):
        text = OverlayText()
        text.action = OverlayText.ADD
        # 컴팩트하게: 화면 상단에 작은 박스
        text.width = 320          # int
        text.height = 150          # int
        text.text_size = 12.0     # float
        text.line_width = 1       # int
        text.font = "DejaVu Sans Mono"

        # 색상 (RGBA float)
        text.fg_color.r = 0.60
        text.fg_color.g = 1.00
        text.fg_color.b = 1.00
        text.fg_color.a = 1.00

        text.bg_color.r = 0.00
        text.bg_color.g = 0.00
        text.bg_color.b = 0.00
        text.bg_color.a = 0.35

        text.text = (
            f"Speed: {self.current_speed:.2f} m/s (Command: {self.target_speed:.2f})\n"
            f"\n"
            f"Steer: {self.current_steer:.1f} deg (Command: {self.target_steer:.1f})\n"
            f"\n"
            f"Yaw:   {self.yaw_deg:.1f} deg\n"
            f"\n"
            f"Pose: X={self.position[0]:.2f}, Y={self.position[1]:.2f}\n"
        )

        self.pub_text.publish(text)


def main(args=None):
    rclpy.init(args=args)
    node = HudOverlayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
