#!/usr/bin/env python3
# file: car_marker_publisher.py
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.duration import Duration
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray


class CarMarkerPublisher(Node):
    def __init__(self) -> None:
        super().__init__('car_marker_publisher')

        # ===== 프레임 =====
        self.declare_parameter('frame_id', 'base_link')  # base_link = 후륜축 중심
        self.frame_id: str = self.get_parameter('frame_id').get_parameter_value().string_value

        # ===== 차체/섀시 치수 (m) =====
        self.declare_parameter('body_length', 2.6)   # 범퍼~범퍼
        self.declare_parameter('body_width',  1.2)
        self.declare_parameter('body_height', 0.9)
        self.body_length: float = float(self.get_parameter('body_length').value)
        self.body_width:  float = float(self.get_parameter('body_width').value)
        self.body_height: float = float(self.get_parameter('body_height').value)

        self.declare_parameter('wheel_base',  1.3)   # 앞/뒤 축 간 거리
        self.declare_parameter('track_width', 1.2)   # 좌/우 바퀴 간 거리
        self.wheel_base:  float = float(self.get_parameter('wheel_base').value)
        self.track_width: float = float(self.get_parameter('track_width').value)

        # ===== 오버행 (m) =====
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
        self.declare_parameter('wheel_radius', 0.235)
        self.declare_parameter('wheel_width',  0.19)
        self.wheel_radius: float = float(self.get_parameter('wheel_radius').value)
        self.wheel_width:  float = float(self.get_parameter('wheel_width').value)

        # ===== 높이 배치 (m) =====
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
        self.declare_parameter('body_color_a', 0.25)  # 바디 반투명으로 (시야확보)
        self.declare_parameter('wheel_color_r', 0.1)
        self.declare_parameter('wheel_color_g', 0.1)
        self.declare_parameter('wheel_color_b', 0.1)
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
        self.declare_parameter('rate_hz', 30.0)  # 시각화 부드럽게
        rate_hz: float = float(self.get_parameter('rate_hz').value)

        # ===== 조향 토픽/필터/제한 =====
        # 입력: deg, 좌회전 + / 우회전 −
        self.declare_parameter('steer_topic', '/cmd/steer') # 추후 제어에서 넘겨주는 실제 조향각 토픽(/ctrl/steer)으로 변환하기
        self.declare_parameter('steer_limit_deg', 28.0)   # 시각화용 제한(도)
        self.declare_parameter('steer_ema_tau', 0.10)     # [s] 시각화 필터(EMA)
        # 외부 입력의 부호 체계가 반대일 때(True면 입력 부호를 반전: 좌회전 음수/우회전 양수 → 내부 표준 좌+, 우−로 변환)
        self.declare_parameter('invert_steer_sign', True)
        steer_topic = self.get_parameter('steer_topic').get_parameter_value().string_value
        self.steer_limit_deg = float(self.get_parameter('steer_limit_deg').value)
        self.steer_limit_rad = math.radians(self.steer_limit_deg)
        self.steer_tau = float(self.get_parameter('steer_ema_tau').value)
        self.invert_steer_sign = bool(self.get_parameter('invert_steer_sign').value)

        # ===== 시각화 옵션 =====
        # Ackermann 분해 토글 + 화살표 스타일
        self.declare_parameter('use_ackermann_visual', False)   # True면 좌/우 앞바퀴 각 분리
        self.declare_parameter('arrow_style', 'center')         # 'center' or 'per_wheel'

        # 화살표/텍스트
        self.declare_parameter('show_steer_arrow', False)
        self.declare_parameter('arrow_length', 4.0)        # [m]
        self.declare_parameter('arrow_thickness', 0.08)    # [m]
        self.declare_parameter('arrow_height', 0.07)       # [m] 바퀴/전륜 중앙 위로 띄우기

        self.declare_parameter('show_steer_text', True)
        self.declare_parameter('text_height', 0.28)        # [m]
        self.declare_parameter('text_size', 0.20)          # [m]

        # 전륜 보조선(Line Strip)
        self.declare_parameter('show_front_link', True)
        self.declare_parameter('front_link_width', 0.03)   # 선 두께
        self.declare_parameter('front_link_height', 0.05)  # z 오프셋
        self.declare_parameter('front_link_follow_steer', False)  # True: 조향각 따라 회전된 바

        self.use_ackermann_visual = bool(self.get_parameter('use_ackermann_visual').value)
        self.arrow_style = str(self.get_parameter('arrow_style').value)

        self.show_steer_arrow = bool(self.get_parameter('show_steer_arrow').value)
        self.arrow_length = float(self.get_parameter('arrow_length').value)
        self.arrow_thickness = float(self.get_parameter('arrow_thickness').value)
        self.arrow_height = float(self.get_parameter('arrow_height').value)

        self.show_steer_text = bool(self.get_parameter('show_steer_text').value)
        self.text_height = float(self.get_parameter('text_height').value)
        self.text_size = float(self.get_parameter('text_size').value)

        self.show_front_link = bool(self.get_parameter('show_front_link').value)
        self.front_link_width = float(self.get_parameter('front_link_width').value)
        self.front_link_height = float(self.get_parameter('front_link_height').value)
        self.front_link_follow_steer = bool(self.get_parameter('front_link_follow_steer').value)

        # QoS: Transient Local + Reliable (RViz 새로 열어도 마지막 마커 유지)
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability  = DurabilityPolicy.TRANSIENT_LOCAL

        self.pub   = self.create_publisher(MarkerArray, 'car_marker', qos)
        self.timer = self.create_timer(1.0 / max(rate_hz, 1e-3), self._on_timer)

        # 조향 구독 (deg 입력)
        self.sub_steer = self.create_subscription(Float32, steer_topic, self._on_steer_deg, 10)

        # 조향 상태
        self._steer_raw_rad = 0.0   # [rad] 최신 명령(라디안으로 변환 저장) — '자전거' 조향각
        self._steer_filt = 0.0      # [rad] EMA 필터 결과 (자전거 조향각)
        self._last_tick = None

        # ===== 배치/겹침 검증 =====
        self._sanity_checks()

        self.get_logger().info(
            f'frame={self.frame_id} (rear-axle origin), '
            f'LxWxH={self.body_length:.2f}x{self.body_width:.2f}x{self.body_height:.2f}, '
            f'WB={self.wheel_base:.2f}, TW={self.track_width:.2f}, '
            f'OH(front/rear)={self.front_overhang:.2f}/{self.rear_overhang:.2f}, '
            f'body_z={self.body_z:.2f}, wheel_z={self.wheel_z:.2f}, '
            f'use_ackermann_visual={self.use_ackermann_visual}, arrow_style={self.arrow_style}'
        )

    # -------- Helpers --------
    def _sanity_checks(self):
        body_bottom = self.body_z - self.body_height / 2.0
        wheel_top   = self.wheel_z + self.wheel_radius
        if body_bottom <= wheel_top + 1e-3:
            self.get_logger().warn(
                f'차체 바닥({body_bottom:.3f}) ≤ 바퀴 꼭대기({wheel_top:.3f}). '
                f'body_z(혹은 ground_clearance)를 키우세요.'
            )
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

    def _make_wheel_marker(self, mid: int, x: float, y: float, z: float, steer_yaw: float = 0.0) -> Marker:
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

        # 실린더 축(Z)→바퀴 축(Y): roll=+90°, 여기에 yaw=steer 적용 (좌회전+)
        qx, qy, qz, qw = self._quat_from_rpy(math.pi/2.0, 0.0, steer_yaw)
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

    def _make_arrow_marker(self, mid: int, x: float, y: float, z: float, yaw: float) -> Marker:
        now = self.get_clock().now().to_msg()
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = now
        m.ns = 'car'
        m.id = mid
        m.type = Marker.ARROW
        m.action = Marker.ADD

        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z + self.arrow_height  # 살짝 띄움
        # yaw 방향으로 향하는 화살표
        qx, qy, qz, qw = self._quat_from_rpy(0.0, 0.0, yaw)
        m.pose.orientation.x = qx; m.pose.orientation.y = qy; m.pose.orientation.z = qz; m.pose.orientation.w = qw

        # ARROW: scale.x=길이, scale.y=샤프트 직경, scale.z=헤드 직경
        m.scale.x = self.arrow_length
        m.scale.y = self.arrow_thickness
        m.scale.z = self.arrow_thickness * 2.0

        # 눈에 띄는 색 (노란색)
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.85, 0.1, 1.0
        m.lifetime = Duration(seconds=0.0).to_msg()
        m.frame_locked = True
        return m

    def _make_text_marker(self, mid: int, x: float, y: float, z: float, text: str) -> Marker:
        now = self.get_clock().now().to_msg()
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = now
        m.ns = 'car'
        m.id = mid
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD

        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z + self.text_height

        m.scale.z = self.text_size   # TEXT_VIEW_FACING은 scale.z가 글자 크기
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 1.0, 1.0  # 흰색
        m.text = text
        m.lifetime = Duration(seconds=0.0).to_msg()
        m.frame_locked = True
        return m

    def _make_front_link_strip(self, mid: int, fx: float, ly: float, ry: float, z: float, yaw: float) -> Marker:
        """
        전륜 사이 보조선(Line Strip).
        - 기본: FL( fx, +ly ) ↔ FR( fx, -ly )를 직선으로 연결
        - front_link_follow_steer=True 인 경우: 전륜 중앙(fx,0)에서 조향각(yaw)에 수직인 방향으로
          track_width/2 만큼 양쪽으로 뻗는 바를 그림(타이바 같은 느낌)
        """
        now = self.get_clock().now().to_msg()
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = now
        m.ns = 'car'
        m.id = mid
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD

        m.scale.x = self.front_link_width  # LINE_STRIP 두께
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 1.0, 1.0  # 시안색

        from geometry_msgs.msg import Point
        if not self.front_link_follow_steer:
            p1 = Point(); p1.x = fx; p1.y = ly; p1.z = z + self.front_link_height
            p2 = Point(); p2.x = fx; p2.y = ry; p2.z = z + self.front_link_height
            m.points = [p1, p2]
        else:
            half = self.track_width / 2.0
            # yaw에 수직(좌/우)인 단위벡터 n = (-sin(yaw), cos(yaw))
            nx = -math.sin(yaw); ny = math.cos(yaw)
            cx = fx; cy = 0.0
            p1 = Point(); p1.x = cx + nx * half; p1.y = cy + ny * half; p1.z = z + self.front_link_height
            p2 = Point(); p2.x = cx - nx * half; p2.y = cy - ny * half; p2.z = z + self.front_link_height
            m.points = [p1, p2]

        m.lifetime = Duration(seconds=0.0).to_msg()
        m.frame_locked = True
        return m

    # ----- Ackermann 분해 -----
    def _split_ackermann(self, steer_bicycle_rad: float) -> Tuple[float, float]:
        """
        입력: '자전거 모델' 전륜 조향각 (steer_bicycle_rad)
        출력: (FL, FR) 각도 [rad] — 좌회전 +, 우회전 −
        수식:
          kappa = tan(δ_bicycle) / L,  R = 1/kappa = L / tan(δ_bicycle)
          δ_in  = atan( L / (R - T/2) ),  δ_out = atan( L / (R + T/2) )
        """
        L = max(1e-6, self.wheel_base)
        T = max(1e-6, self.track_width)
        d = steer_bicycle_rad
        if abs(d) < 1e-9:
            return (0.0, 0.0)

        tan_d = math.tan(d)
        # 자전거 모델 곡률/반경
        kappa = tan_d / L
        if abs(kappa) < 1e-12:
            return (d, d)

        R = 1.0 / kappa
        sign = 1.0 if d > 0 else -1.0   # +:좌회전, −:우회전

        Rabs = abs(R)
        # 키네매틱 불안정 방지(아주 작은 R에서 분모 음수 방지)
        eps = 1e-6
        Rin = max(eps, Rabs - T * 0.5)
        Rout = max(eps, Rabs + T * 0.5)
        d_in = math.atan(L / Rin)
        d_out = math.atan(L / Rout)

        if sign > 0:   # 좌회전: 왼쪽이 inner
            return (+d_in, +d_out)
        else:          # 우회전: 오른쪽이 inner
            return (-d_out, -d_in)

    # -------- Callbacks --------
    def _on_steer_deg(self, msg: Float32) -> None:
        # 입력: deg. 기본 가정은 좌회전 + / 우회전 −.
        # invert_steer_sign=True면 외부 입력(좌−/우+)을 내부 표준(좌+/우−)으로 변환.
        steer_deg = float(msg.data)
        if self.invert_steer_sign:
            steer_deg = -steer_deg
        # 시각화용 제한(도) → 라디안 변환 순서대로 클램프
        steer_deg = max(-self.steer_limit_deg, min(self.steer_limit_deg, steer_deg))
        self._steer_raw_rad = math.radians(steer_deg)

    # -------- Timer --------
    def _on_timer(self) -> None:
        # 조향 EMA 필터 (시각화 안정화) — '자전거' 조향각에 적용
        now = self.get_clock().now()
        dt = 0.0 if self._last_tick is None else (now - self._last_tick).nanoseconds * 1e-9
        self._last_tick = now

        alpha = math.exp(-max(0.0, dt) / max(self.steer_tau, 1e-3))
        steer_bicycle = alpha * self._steer_filt + (1.0 - alpha) * self._steer_raw_rad
        steer_bicycle = max(-self.steer_limit_rad, min(self.steer_limit_rad, steer_bicycle))
        self._steer_filt = steer_bicycle

        # Ackermann 분해(옵션)
        if self.use_ackermann_visual:
            steer_fl, steer_fr = self._split_ackermann(steer_bicycle)
        else:
            steer_fl = steer_fr = steer_bicycle

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

        # 2) 바퀴 4개 (앞바퀴 yaw: FL=steer_fl, FR=steer_fr)
        markers.append(self._make_wheel_marker(1, fx, ly, self.wheel_z, steer_yaw=steer_fl))  # FL
        markers.append(self._make_wheel_marker(2, fx, ry, self.wheel_z, steer_yaw=steer_fr))  # FR
        markers.append(self._make_wheel_marker(3, rx, ly, self.wheel_z, steer_yaw=0.0))       # RL
        markers.append(self._make_wheel_marker(4, rx, ry, self.wheel_z, steer_yaw=0.0))       # RR

        # 3) 전륜 보조선(Line Strip)
        if self.show_front_link:
            # follow_steer가 True면 '자전거' 조향각 기준으로 회전한 바를 그림
            base_yaw_for_bar = steer_bicycle if self.front_link_follow_steer else 0.0
            markers.append(self._make_front_link_strip(301, fx, ly, ry, self.wheel_z, base_yaw_for_bar))

        # 4) 화살표 & 각도 텍스트
        if self.show_steer_arrow:
            if self.arrow_style == 'per_wheel':
                # 각 바퀴 위에 화살표
                markers.append(self._make_arrow_marker(101, fx, ly, self.wheel_z, yaw=steer_fl))  # FL
                markers.append(self._make_arrow_marker(102, fx, ry, self.wheel_z, yaw=steer_fr))  # FR
            else:
                # 전륜 중앙( fx, 0 )에 '자전거' 조향각 화살표 1개
                markers.append(self._make_arrow_marker(100, fx, 0.0, self.wheel_z, yaw=steer_bicycle))

        if self.show_steer_text:
            if self.use_ackermann_visual or self.arrow_style == 'per_wheel':
                # 좌/우 각도 따로 표기
                txt_fl = f"{math.degrees(steer_fl):+.1f}°"
                txt_fr = f"{math.degrees(steer_fr):+.1f}°"
                markers.append(self._make_text_marker(201, fx, ly, self.wheel_z, txt_fl))  # FL
                markers.append(self._make_text_marker(202, fx, ry, self.wheel_z, txt_fr))  # FR
            else:
                # 중앙에 자전거 조향각 표기
                txt = f"{math.degrees(steer_bicycle):+.1f}°"
                markers.append(self._make_text_marker(200, fx, 0.0, self.wheel_z, txt))

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
