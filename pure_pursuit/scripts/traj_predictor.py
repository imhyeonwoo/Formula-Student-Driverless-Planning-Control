#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.duration import Duration

from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray, Marker


class TrajPredictor(Node):
    """
    - 입력:
        /cmd/steer : Float32 (deg, 좌+, 우-), 목표 조향각
        /current_speed : Float32 (m/s), 차량 현재 속도 (우선 사용)
        /cmd/speed : Float32 (m/s), 목표 속도 (백업)
    - 출력:
        /predicted_path : nav_msgs/Path
        /predicted_markers : MarkerArray (진행 방향 화살표, 텍스트 RPM 등)
    - 모델: Kinematic bicycle (자전거 모델)
        x_{k+1} = x_k + v cosθ dt
        y_{k+1} = y_k + v sinθ dt
        θ_{k+1} = θ_k + (v/L) tanδ dt
      좌표계: frame_id 파라미터 (기본 map). 시작 포즈는 (0,0,0)로 가정하거나
      필요 시 /current_pose를 구독하도록 확장 가능.
    """

    def __init__(self) -> None:
        super().__init__('traj_predictor')

        # ----- Frame / Topics -----
        self.declare_parameter('frame_id', 'map')
        self.frame_id: str = self.get_parameter('frame_id').get_parameter_value().string_value

        self.declare_parameter('steer_topic', '/cmd/steer')
        self.declare_parameter('speed_topic', '/cmd/speed')
        self.declare_parameter('current_speed_topic', '/current_speed')

        steer_topic = self.get_parameter('steer_topic').get_parameter_value().string_value
        speed_topic = self.get_parameter('speed_topic').get_parameter_value().string_value
        cur_speed_topic = self.get_parameter('current_speed_topic').get_parameter_value().string_value

        # ----- Vehicle Params -----
        self.declare_parameter('wheel_base', 1.30)          # [m]
        self.declare_parameter('max_steer_deg', 28.0)       # 시각화/안정화용 제한
        self.declare_parameter('default_speed', 1.5)        # [m/s] 백업 속도
        self.declare_parameter('sim_dt', 0.02)              # [s] 적분 간격
        self.declare_parameter('horizon_seconds', 3.0)      # [s] 예측 구간

        self.declare_parameter('wheel_diameter_m', 0.47)    # [m]
        self.declare_parameter('gear_ratio', 4.6)           # motor:wheel (RPM_wheel = RPM_motor / ratio)

        self.L = float(self.get_parameter('wheel_base').value)
        self.max_steer_rad = math.radians(float(self.get_parameter('max_steer_deg').value))
        self.default_speed = float(self.get_parameter('default_speed').value)
        self.dt = float(self.get_parameter('sim_dt').value)
        self.horizon = float(self.get_parameter('horizon_seconds').value)
        self.wheel_diam = float(self.get_parameter('wheel_diameter_m').value)
        self.gear_ratio = float(self.get_parameter('gear_ratio').value)

        # ----- QoS -----
        path_qos = QoSProfile(depth=10)  # Path는 일반 QoS
        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL  # RViz 재시작 시 유지

        # ----- Pubs -----
        self.pub_path = self.create_publisher(Path, 'predicted_path', path_qos)
        self.pub_mark = self.create_publisher(MarkerArray, 'predicted_markers', marker_qos)

        # ----- Subs -----
        self.sub_steer = self.create_subscription(Float32, steer_topic, self._on_steer, 10)
        self.sub_cur_speed = self.create_subscription(Float32, cur_speed_topic, self._on_current_speed, 10)
        self.sub_cmd_speed = self.create_subscription(Float32, speed_topic, self._on_cmd_speed, 10)

        # ----- States -----
        self._steer_rad: float = 0.0
        self._v_current: Optional[float] = None
        self._v_cmd: Optional[float] = None

        # 주기 실행 타이머
        self.timer = self.create_timer(0.05, self._tick)  # 20Hz

        self.get_logger().info(
            f"[traj_predictor] frame={self.frame_id}, L={self.L:.2f} m, dt={self.dt:.3f}s, "
            f"horizon={self.horizon:.1f}s, wheel_diam={self.wheel_diam:.3f}m, gear={self.gear_ratio:.2f}"
        )

    # ---------- Callbacks ----------
    def _on_steer(self, msg: Float32) -> None:
        # 입력 단위: deg (좌+, 우-)
        d = float(msg.data)
        d = max(-math.degrees(self.max_steer_rad), min(math.degrees(self.max_steer_rad), d))
        self._steer_rad = math.radians(d)

    def _on_current_speed(self, msg: Float32) -> None:
        self._v_current = float(msg.data)

    def _on_cmd_speed(self, msg: Float32) -> None:
        self._v_cmd = float(msg.data)

    # ---------- Core ----------
    def _pick_speed(self) -> float:
        if self._v_current is not None and math.isfinite(self._v_current):
            return float(self._v_current)
        if self._v_cmd is not None and math.isfinite(self._v_cmd):
            return float(self._v_cmd)
        return self.default_speed

    def _tick(self) -> None:
        v = self._pick_speed()
        d = max(-self.max_steer_rad, min(self.max_steer_rad, self._steer_rad))

        steps = max(1, int(math.ceil(self.horizon / self.dt)))

        # 시작 상태: (0,0,0) — frame_id 상의 로컬 미리보기.
        # 필요하면 /current_pose 구독해 초기화하도록 확장 가능.
        x = 0.0
        y = 0.0
        yaw = 0.0

        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()

        for k in range(steps + 1):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.0
            # yaw만 반영 (RPY → 쿼터니언)
            qx, qy, qz, qw = self._quat_from_rpy(0.0, 0.0, yaw)
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            path.poses.append(ps)

            if k < steps:
                x += v * math.cos(yaw) * self.dt
                y += v * math.sin(yaw) * self.dt
                yaw += (v / max(self.L, 1e-6)) * math.tan(d) * self.dt
                yaw = self._wrap_pi(yaw)

        self.pub_path.publish(path)
        self._publish_markers(v, d)

    # ---------- Utils ----------
    def _publish_markers(self, v: float, steer_rad: float) -> None:
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        # 진행 방향 화살표
        m_arrow = Marker()
        m_arrow.header.frame_id = self.frame_id
        m_arrow.header.stamp = now
        m_arrow.ns = 'traj_preview'
        m_arrow.id = 1
        m_arrow.type = Marker.ARROW
        m_arrow.action = Marker.ADD
        m_arrow.pose.orientation.w = 1.0
        m_arrow.pose.position.x = 0.0
        m_arrow.pose.position.y = 0.0
        m_arrow.pose.position.z = 0.05
        m_arrow.scale.x = 1.5    # 길이
        m_arrow.scale.y = 0.06   # shaft
        m_arrow.scale.z = 0.12   # head
        # (색상은 RViz 표시를 위해 alpha만 기본 유효하게. 필요 시 YAML로 확장 가능)
        m_arrow.color.a = 1.0
        m_arrow.color.r = 0.2
        m_arrow.color.g = 1.0
        m_arrow.color.b = 0.2
        m_arrow.lifetime = Duration(seconds=0.0).to_msg()
        m_arrow.frame_locked = True
        arr.markers.append(m_arrow)

        # 텍스트: v, steer, R, wheel_rpm, motor_rpm
        text = self._compose_text(v, steer_rad)
        m_txt = Marker()
        m_txt.header.frame_id = self.frame_id
        m_txt.header.stamp = now
        m_txt.ns = 'traj_preview'
        m_txt.id = 2
        m_txt.type = Marker.TEXT_VIEW_FACING
        m_txt.action = Marker.ADD
        m_txt.pose.position.x = 0.0
        m_txt.pose.position.y = 0.0
        m_txt.pose.position.z = 0.6
        m_txt.scale.z = 0.2
        m_txt.color.a = 1.0
        m_txt.color.r = 1.0
        m_txt.color.g = 1.0
        m_txt.color.b = 1.0
        m_txt.text = text
        m_txt.lifetime = Duration(seconds=0.0).to_msg()
        m_txt.frame_locked = True
        arr.markers.append(m_txt)

        self.pub_mark.publish(arr)

    def _compose_text(self, v: float, steer_rad: float) -> str:
        # 곡률/회전반경
        if abs(steer_rad) < 1e-9:
            R = float('inf')
        else:
            R = self.L / math.tan(steer_rad)

        # 휠/모터 RPM 계산
        # wheel_rpm = v / (π*D) * 60
        # motor_rpm = wheel_rpm * gear_ratio   (RPM_wheel = RPM_motor / ratio)
        circ = math.pi * max(self.wheel_diam, 1e-6)
        wheel_rpm = (v / circ) * 60.0
        motor_rpm = wheel_rpm * self.gear_ratio

        if math.isinf(R):
            R_str = "∞"
        else:
            R_str = f"{R:.2f}"

        return (f"v = {v:.2f} m/s\n"
                f"δ = {math.degrees(steer_rad):+.1f}°\n"
                f"R = {R_str} m\n"
                f"wheel ≈ {wheel_rpm:.1f} RPM\n"
                f"motor ≈ {motor_rpm:.1f} RPM")

    @staticmethod
    def _quat_from_rpy(roll: float, pitch: float, yaw: float):
        cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5);   sy = math.sin(yaw * 0.5)
        w = cr*cp*cy + sr*sp*sy
        x = sr*cp*cy - cr*sp*sy
        y = cr*sp*cy + sr*cp*sy
        z = cr*cp*sy - sr*cp*cy
        return (x, y, z, w)

    @staticmethod
    def _wrap_pi(a: float) -> float:
        while a > math.pi:  a -= 2.0*math.pi
        while a < -math.pi: a += 2.0*math.pi
        return a


def main():
    rclpy.init()
    node = TrajPredictor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
