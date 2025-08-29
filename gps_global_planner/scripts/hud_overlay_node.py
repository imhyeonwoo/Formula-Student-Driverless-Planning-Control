#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float32
from sensor_msgs.msg import Imu, NavSatFix
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
        self.pub_kmh = self.create_publisher(Float32, '/current_kmh', 10)  # ← 추가

        # ===== Subscribers (Sensor QoS로 호환성 확보) =====
        qos = qos_profile_sensor_data
        self.create_subscription(Float32, '/current_speed', self.cb_current_speed, qos)
        self.create_subscription(Float32, '/cmd/speed', self.cb_target_speed, qos)
        self.create_subscription(Float32, '/ctrl/steer', self.cb_current_steer, qos)
        self.create_subscription(Float32, '/cmd/steer', self.cb_target_steer, qos)
        self.create_subscription(Imu, '/imu/processed', self.cb_imu, qos)  # yaw는 여기서 계산하지 않음
        self.create_subscription(Odometry, '/odometry/filtered', self.cb_odom, qos)
        self.create_subscription(NavSatFix, '/ublox_gps_node/fix', self.cb_fix, qos)

        # ===== State =====
        self.current_speed = 0.0  # m/s
        self.target_speed = 0.0
        self.current_steer = 0.0
        self.target_steer = 0.0
        self.yaw_deg = 0.0
        self.position = (0.0, 0.0)

        # GPS 오차 (1-σ, m)
        self.gps_sigma_e = float('nan')  # East 표준편차
        self.gps_sigma_n = float('nan')  # North 표준편차
        self.gps_drms = float('nan')     # 수평 단일 값(=sqrt(σ_E^2+σ_N^2))
        self.gps_cov_ok = False

        # ===== Timer =====
        self.create_timer(0.10, self.publish_overlay)  # 10Hz
        self.create_timer(0.10, self.publish_kmh)      # 10Hz, km/h 값 발행

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

    def cb_yaw_rad(self, msg: Float32):
        self.yaw_deg = float(msg.data) * 180.0 / math.pi

    def cb_fix(self, msg: NavSatFix):
        """
        NavSatFix.position_covariance는 (m^2) 단위 3x3 행렬(행우선).
        type이 2(DIAGONAL_KNOWN) 또는 3(KNOWN)일 때 대각원소를 신뢰.
        cov[0]: East, cov[4]: North, cov[8]: Up
        """
        cov = msg.position_covariance
        if msg.position_covariance_type in (2, 3) and len(cov) == 9:
            try:
                var_e = max(float(cov[0]), 0.0)
                var_n = max(float(cov[4]), 0.0)

                self.gps_sigma_e = math.sqrt(var_e)
                self.gps_sigma_n = math.sqrt(var_n)
                self.gps_drms = math.hypot(self.gps_sigma_e, self.gps_sigma_n)
                self.gps_cov_ok = True
            except Exception as e:
                self.get_logger().warn(f'GPS covariance parse error: {e}')
                self.gps_cov_ok = False
        else:
            self.gps_cov_ok = False

    # ---------- Publishers ----------
    def publish_overlay(self):
        text = OverlayText()
        text.action = OverlayText.ADD
        text.width = 380
        text.height = 195
        text.text_size = 12.0
        text.line_width = 1
        text.font = "DejaVu Sans Mono"

        text.fg_color.r = 0.60
        text.fg_color.g = 1.00
        text.fg_color.b = 1.00
        text.fg_color.a = 1.00

        text.bg_color.r = 0.00
        text.bg_color.g = 0.00
        text.bg_color.b = 0.00
        text.bg_color.a = 0.35

        if self.gps_cov_ok:
            gps_line1 = f"Pose error E/N: {self.gps_sigma_e:.3f} / {self.gps_sigma_n:.3f} m"
            gps_line2 = f"Pose error H  : {self.gps_drms:.3f} m (DRMS)"
        else:
            gps_line1 = "Pose error E/N: N/A"
            gps_line2 = "Pose error H  : N/A"

        text.text = (
            f"Speed: {self.current_speed:.2f} m/s (Command: {self.target_speed:.2f} m/s)\n"
            f"\n"
            f"Steer: {self.current_steer:.1f} deg (Command: {self.target_steer:.1f} deg)\n"
            f"\n"
            f"Yaw:   {self.yaw_deg:.1f} deg\n"
            f"\n"
            f"Pose: X={self.position[0]:.2f}, Y={self.position[1]:.2f}\n"
            f"\n"
            f"{gps_line1}\n"
            f"{gps_line2}\n"
        )

        self.pub_text.publish(text)

    def publish_kmh(self):
        """현재 속도를 km/h로 변환하여 퍼블리시"""
        kmh_msg = Float32()
        kmh_msg.data = self.current_speed * 3.6
        self.pub_kmh.publish(kmh_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HudOverlayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
