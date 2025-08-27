#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
본 스크립트는 좌우 라벨링이 되지 않은 콘 데이터를 이용하여 Potential Field를 형성합니다. (PotentialField)
형성된 Potential Field의 Trough(골)을 필드 영역 전체에 대해 연산하여 저장합니다. (TroughPoints)
산출된 Trough(골) 좌표에서, 차량(0,0)과 가장 가까운 좌표를 시작으로,
임계값(DIST_THRESH) 이하이면서 가장 가까운 좌표만 필터링하여 저장합니다. (ValidPath)
유효한 Trough(골) 좌표를 대상으로 Catmull–Rom(centripetal) 스플라인을 진행하고,
호길이 등간격 재샘플 + Savitzky–Golay 스무딩으로 최종 경로를 부드럽게 만듭니다. (SplinedPath)
모든 콘 좌표에 대해 SplinedPath 기준 좌/우를 판단하여 라벨링합니다. (left_cones, right_cones)
판단된 라벨링 데이터는 /labeled_cones 토픽의 class_names에 저장하고, 해당 좌표를 퍼블리시합니다.(pub_labeled_cones)
'''

import math
import numpy as np
from collections import deque
from typing import List, Tuple, Optional

from scipy.signal import savgol_filter

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from visualization_msgs.msg import Marker
from custom_interface.msg import TrackedConeArray, ModifiedFloat32MultiArray
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA, UInt8

# tf2 (프레임 일치용)
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

qos = QoSProfile(depth=10)
qos.reliability = ReliabilityPolicy.BEST_EFFORT

# =================== 전역 파라미터 =================== #
#----- Potential Field -----#
X_RANGE = 30.0
Y_RANGE = 8.0
Y_BACK  = 20.0
GRID_RES = 0.5
AMP = 1.5
SIGMA = 0.9

#----- Car Position (로컬 센서 프레임 기준) -----#
CAR_X, CAR_Y = 0.0, 0.0

#----- Path -----#
DIST_THRESH = 2.0
FRAME_BUFFER = 3

#----- AEB -----#
AEB_TRHESH = 7
AEB_HOR = 6.0
AEB_VER = 8.0

#----- Forward 판단 임계 -----#
FORWARD_DOT_EPS = -0.05  # 음수 약간 허용(너무 보수적이면 경로가 끊길 수 있음)

#=====================================================#

class PotentialFieldLabeler(Node):
    def __init__(self):
        super().__init__('Potential_Field_Labeler')

        ###### (선택) 스플라인/스무딩 파라미터 선언 ######
        # 런치에서 --ros-args -p resample_ds:=0.2 -p sg_window:=9 -p sg_poly:=3 -p cr_alpha:=0.5 -p samples_per_seg:=20 조정 가능
        self.declare_parameter('resample_ds', 0.2)      # 최종 경로 점 간격[m]
        self.declare_parameter('sg_window', 9)          # Savitzky–Golay 윈도(홀수)
        self.declare_parameter('sg_poly', 3)            # Savitzky–Golay 폴리 차수
        self.declare_parameter('cr_alpha', 0.5)         # Catmull–Rom alpha(0: uniform, 0.5: centripetal, 1: chordal)
        self.declare_parameter('samples_per_seg', 20)   # 스플라인 각 세그먼트 샘플 수(기본 20)

        self.resample_ds = float(self.get_parameter('resample_ds').value)
        self.sg_window   = int(self.get_parameter('sg_window').value)
        self.sg_poly     = int(self.get_parameter('sg_poly').value)
        self.cr_alpha    = float(self.get_parameter('cr_alpha').value)
        self.samples_per_seg = int(self.get_parameter('samples_per_seg').value)

        ###### Subscriber ######
        # ★ 토픽명 유지 (네 환경과 동일)
        self.unknown_sub = self.create_subscription(
            TrackedConeArray, '/cone/lidar/ukf', self.cones_callback, qos)
        self.red_sub = self.create_subscription(
            TrackedConeArray, '/cone/fused/ukf', self.red_callback, qos)

        ###### Publisher ######
        self.pub_labeled_cones = self.create_publisher(ModifiedFloat32MultiArray, '/labeled_cones', 10)
        self.leftcone_pub  = self.create_publisher(Marker, '/left_cone_marker', 10)
        self.rightcone_pub = self.create_publisher(Marker, '/right_cone_marker', 10)
        self.aeb_pub       = self.create_publisher(UInt8, '/estop', 10)

        # ----- Visualize -----#
        self.field_pub         = self.create_publisher(Marker, '/potential_field', 10)
        self.trough_pub        = self.create_publisher(Marker, '/trough_marker', 10)
        self.valid_trough_pub  = self.create_publisher(Marker, '/valid_trough_marker', 10)
        self.carpos_pub        = self.create_publisher(Marker, '/carpos_marker', 10)
        self.pub_splined_path  = self.create_publisher(Path, '/local_planned_path', 10)
        self.redcone_pub       = self.create_publisher(Marker, '/red_cone_marker', 10)
        self.aeb_roi_pub       = self.create_publisher(Marker, '/aeb_roi', 10)

        #----- 그리드 및 좌표계 정의 -----#
        self.grid_x = np.arange(-X_RANGE, X_RANGE + GRID_RES, GRID_RES)
        self.grid_y = np.arange(-Y_BACK, Y_RANGE + GRID_RES, GRID_RES)
        self.X, self.Y = np.meshgrid(self.grid_x, self.grid_y)

        #----- 콘 좌표 정의 -----#
        self.cone_data: List[Tuple[float, float, float]] = []
        self.frame_buffer: deque = deque(maxlen=FRAME_BUFFER)
        self.red_cone: List[Tuple[float, float, float]] = []

        #----- TF Buffer/Listener (프레임 변환용) -----#
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #----- 메인루프 주기적 실행 -----#
        self.create_timer(0.1, self.main_loop)

    #=======================
    #  Callbacks
    #=======================
    def cones_callback(self, msg: TrackedConeArray):
        n_cones = len(msg.cones)
        if n_cones == 0:
            return
        new_frame = []
        for cone in msg.cones:
            x, y, z = cone.position.x, cone.position.y, cone.position.z
            new_frame.append((x, y, z))
        self.frame_buffer.append(new_frame)
        # 버퍼가 가득 차야만 사용 → 기존 로직 유지(필요시 완화 가능)
        if len(self.frame_buffer) == FRAME_BUFFER:
            self.cone_data = sum(self.frame_buffer, [])
        else:
            self.cone_data = []

    def red_callback(self, msg: TrackedConeArray):
        if len(msg.cones) < 3:
            return
        self.red_cone = []
        for cone in msg.cones:
            if getattr(cone, "color", "") == "Red Cone":
                x, y, z = cone.position.x, cone.position.y, cone.position.z
                self.red_cone.append((x, y, z))

    #=======================
    #  Math / TF Helpers
    #=======================
    @staticmethod
    def _quat_to_yaw(q) -> float:
        # q: geometry_msgs/Quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _heading_in_os_sensor(self) -> np.ndarray:
        """
        base_link의 +x 전방 벡터를 os_sensor 좌표계로 표현한 2D 단위벡터를 반환.
        TF 조회 실패 시 [1,0] (os_sensor의 +x) 가정.
        """
        try:
            # base_link -> os_sensor (target=os_sensor, source=base_link)
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                'os_sensor', 'base_link', Time())
            yaw = self._quat_to_yaw(tf.transform.rotation)
            return np.array([math.cos(yaw), math.sin(yaw)], dtype=float)
        except Exception:
            return np.array([1.0, 0.0], dtype=float)

    def _transform_points_to_baselink(self, pts_xy: List[Tuple[float, float]]) -> Optional[List[Tuple[float, float]]]:
        """
        os_sensor 좌표계의 (x,y) 목록을 base_link 좌표계로 변환.
        TF 조회 실패 시 None 반환.
        """
        try:
            # os_sensor -> base_link (target=base_link, source=os_sensor)
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                'base_link', 'os_sensor', Time())
        except Exception:
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = self._quat_to_yaw(q)
        c, s = math.cos(yaw), math.sin(yaw)

        out: List[Tuple[float, float]] = []
        for x_, y_ in pts_xy:
            xb = c * x_ - s * y_ + t.x
            yb = s * x_ + c * y_ + t.y
            out.append((float(xb), float(yb)))
        return out

    #=======================
    #  Field / Path
    #=======================
    def create_potential_field(self, points: List[Tuple[float, float]]) -> np.ndarray:
        field = np.zeros_like(self.X)
        for x_i, y_i in points:
            field += AMP * np.exp(-((self.X - x_i) ** 2 + (self.Y - y_i) ** 2) / (2 * SIGMA ** 2))
        return field

    def get_trough_point(self, field: np.ndarray) -> List[Tuple[float, float]]:
        trough_point: List[Tuple[float, float]] = []
        rows, cols = field.shape
        for i in range(1, rows - 1):
            for j in range(1, cols - 1):
                center = field[i, j]
                satisfied = 0
                if center < field[i, j - 1] and center < field[i, j + 1]:
                    satisfied += 1
                if center < field[i - 1, j] and center < field[i + 1, j]:
                    satisfied += 1
                if center < field[i - 1, j - 1] and center < field[i + 1, j + 1]:
                    satisfied += 1
                if center < field[i - 1, j + 1] and center < field[i + 1, j - 1]:
                    satisfied += 1
                if satisfied >= 2:
                    x, y = self.grid_x[j], self.grid_y[i]
                    trough_point.append((x, y))
        return trough_point

    # ---- 전방 필터 ---- #
    def _filter_ahead(self, pts: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        if not pts:
            return []
        heading = self._heading_in_os_sensor()
        out = []
        for x, y in pts:
            v = np.array([x - CAR_X, y - CAR_Y], dtype=float)
            if float(np.dot(v, heading)) >= FORWARD_DOT_EPS:
                out.append((x, y))
        return out

    # ---- 유효좌표 연결(전방 제약) ---- #
    def validation_filter(self, troughpoints: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        if not troughpoints:
            return []

        heading = self._heading_in_os_sensor()
        # 전방 후보만
        ahead = []
        for p in troughpoints:
            v = np.array([p[0] - CAR_X, p[1] - CAR_Y], dtype=float)
            if float(np.dot(v, heading)) >= FORWARD_DOT_EPS:
                ahead.append(p)
        if not ahead:
            return []

        remaining = set(ahead)
        current = min(ahead, key=lambda p: np.hypot(p[0] - CAR_X, p[1] - CAR_Y))
        path = [current]
        remaining.remove(current)

        while remaining:
            def score(p):
                v = np.array([p[0] - current[0], p[1] - current[1]], dtype=float)
                dist = float(np.hypot(v[0], v[1]))
                forward = float(np.dot(v, heading))
                pen = 1e3 if forward < FORWARD_DOT_EPS else 0.0
                return dist + pen

            next_point = min(remaining, key=score)
            vx, vy = (next_point[0] - current[0]), (next_point[1] - current[1])
            dist = math.hypot(vx, vy)
            # 거리 제한 + 뒤로 크게 가는 스텝 차단
            if dist > DIST_THRESH or np.dot([vx, vy], heading) < FORWARD_DOT_EPS:
                break

            path.append(next_point)
            remaining.remove(next_point)
            current = next_point

        return path

    #=======================
    #  Catmull–Rom Spline (centripetal)
    #=======================
    @staticmethod
    def _catmull_rom_segment(p0, p1, p2, p3, alpha, n_samples):
        """
        단일 세그먼트를 Catmull–Rom(centripetal) 파라미터화로 보간하여 n_samples 포인트 반환.
        p0..p3: (x,y)
        alpha: 0(Uniform) ~ 1(Chordal), 0.5 권장
        """
        p0 = np.array(p0, dtype=float); p1 = np.array(p1, dtype=float)
        p2 = np.array(p2, dtype=float); p3 = np.array(p3, dtype=float)

        def tj(ti, pi, pj):
            dist = np.linalg.norm(pj - pi)
            return ti + (dist ** alpha)

        # 파라미터 t
        t0 = 0.0
        t1 = tj(t0, p0, p1)
        t2 = tj(t1, p1, p2)
        t3 = tj(t2, p2, p3)

        # 분모 0 방지
        if abs(t1 - t0) < 1e-6: t1 += 1e-6
        if abs(t2 - t1) < 1e-6: t2 += 1e-6
        if abs(t3 - t2) < 1e-6: t3 += 1e-6

        ts = np.linspace(t1, t2, n_samples)
        pts = []
        for t in ts:
            # 재귀적 보간(표준 공식)
            A1 = (t1 - t)/(t1 - t0) * p0 + (t - t0)/(t1 - t0) * p1
            A2 = (t2 - t)/(t2 - t1) * p1 + (t - t1)/(t2 - t1) * p2
            A3 = (t3 - t)/(t3 - t2) * p2 + (t - t2)/(t3 - t2) * p3

            B1 = (t2 - t)/(t2 - t0) * A1 + (t - t0)/(t2 - t0) * A2
            B2 = (t3 - t)/(t3 - t1) * A2 + (t - t1)/(t3 - t1) * A3

            C  = (t2 - t)/(t2 - t1) * B1 + (t - t1)/(t2 - t1) * B2
            pts.append((float(C[0]), float(C[1])))
        return pts

    def _catmull_rom_spline(self, pts: List[Tuple[float,float]], alpha: float, samples_per_seg: int) -> List[Tuple[float,float]]:
        """
        전체 점열에 대해 Catmull–Rom(centripetal) 스플라인 곡선을 생성.
        양 끝단은 endpoint clamping(끝점 중복)으로 처리.
        """
        if len(pts) < 2:
            return pts
        if len(pts) == 2:
            # 직선 보간
            p0, p1 = pts[0], pts[1]
            ts = np.linspace(0.0, 1.0, max(2, samples_per_seg))
            out = [(float((1-t)*p0[0] + t*p1[0]), float((1-t)*p0[1] + t*p1[1])) for t in ts]
            return out

        # 중복 제거(너무 가까운 점 제거)
        uniq = [pts[0]]
        for p in pts[1:]:
            if np.hypot(p[0]-uniq[-1][0], p[1]-uniq[-1][1]) > 1e-3:
                uniq.append(p)
        if len(uniq) < 2:
            return uniq

        # 끝점 패딩
        P = [uniq[0], *uniq, uniq[-1]]
        curve = []
        for i in range(len(P)-3):
            seg = self._catmull_rom_segment(P[i], P[i+1], P[i+2], P[i+3], alpha, samples_per_seg)
            if i > 0:
                # 이음매 중복 제거
                seg = seg[1:]
            curve.extend(seg)
        return curve

    # ---- 호길이 등간격 재샘플 ---- #
    @staticmethod
    def _resample_by_arclength(pts: List[Tuple[float,float]], ds: float) -> List[Tuple[float,float]]:
        if not pts or len(pts) < 2:
            return pts
        p = np.array(pts, dtype=float)
        seg = np.linalg.norm(p[1:] - p[:-1], axis=1)
        s   = np.concatenate([[0.0], np.cumsum(seg)])
        L   = s[-1]
        if L < 1e-6:
            return pts
        n   = max(2, int(L/ds) + 1)
        s_new = np.linspace(0.0, L, n)
        x_new = np.interp(s_new, s, p[:,0])
        y_new = np.interp(s_new, s, p[:,1])
        return list(zip(map(float, x_new), map(float, y_new)))

    # ---- Savitzky–Golay 스무딩 ---- #
    @staticmethod
    def _savgol(pts: List[Tuple[float,float]], window: int, poly: int) -> List[Tuple[float,float]]:
        if not pts:
            return pts
        x = np.array([p[0] for p in pts], dtype=float)
        y = np.array([p[1] for p in pts], dtype=float)
        # 윈도: 홀수, 길이 이하
        w = window if window % 2 == 1 else window + 1
        max_w = len(x) if len(x) % 2 == 1 else len(x) - 1
        if max_w < 3:
            return pts
        w = min(w, max_w)
        if w <= poly:
            return pts
        x_s = savgol_filter(x, window_length=w, polyorder=poly, mode='interp')
        y_s = savgol_filter(y, window_length=w, polyorder=poly, mode='interp')
        return list(zip(map(float, x_s), map(float, y_s)))

    # ---- 스플라이닝 파이프라인 ---- #
    def splining(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        if not path:
            return []

        # 1) Catmull–Rom(centripetal) 스플라인(포인트 통과, 오버슈트 적음)
        cr_curve = self._catmull_rom_spline(path, alpha=self.cr_alpha, samples_per_seg=self.samples_per_seg)

        # 2) 전방 후처리(뒤쪽 제거)
        cr_curve = self._filter_ahead(cr_curve)
        if not cr_curve:
            return []

        # 3) 호길이 등간격 재샘플
        curve = self._resample_by_arclength(cr_curve, self.resample_ds)

        # 4) Savitzky–Golay 스무딩(곡률 보존하면서 노이즈 완화)
        curve = self._savgol(curve, self.sg_window, self.sg_poly)

        return curve

    #=======================
    #  좌/우 라벨링
    #=======================
    def cone_sort(self, cones: List[Tuple[float, float, float]], path: List[Tuple[float, float]]):
        if len(cones) == 0 or len(path) < 2:
            return [], []

        path_np = np.array(path, dtype=float)
        left_cones = []
        right_cones = []

        for cone in cones:
            cone_xy = np.array(cone[:2], dtype=float)
            dists = np.linalg.norm(path_np - cone_xy, axis=1)
            idx = int(np.argmin(dists))

            if idx < len(path_np) - 1:
                tangent = path_np[idx + 1] - path_np[idx]
            else:
                tangent = path_np[idx] - path_np[idx - 1]

            if np.linalg.norm(tangent) < 1e-3:
                continue

            tangent = tangent / np.linalg.norm(tangent)
            normal = np.array([-tangent[1], tangent[0]], dtype=float)
            rel = cone_xy - path_np[idx]
            side = float(np.dot(rel, normal))

            if side > 0:
                left_cones.append(cone)
            else:
                right_cones.append(cone)

        return left_cones, right_cones

    def determine_aeb(self, cone: List[Tuple[float, float, float]]):
        n_red = 0
        for x, y, z in cone:
            if 0.0 <= x <= AEB_VER and -AEB_HOR <= y <= AEB_HOR:
                n_red += 1
        aebmsg = UInt8()
        aebmsg.data = 1 if n_red > AEB_TRHESH else 0
        self.aeb_pub.publish(aebmsg)
        return aebmsg.data

    #=======================
    #  Visualization
    #=======================
    def vis_Field(self, field: np.ndarray):
        marker = Marker()
        marker.header.frame_id = 'os_sensor'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'Potential Field'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = GRID_RES
        marker.scale.y = GRID_RES
        marker.pose.orientation.w = 1.0

        norm = (field - field.min()) / (field.max() - field.min() + 1e-6)

        for i in range(field.shape[0]):
            for j in range(field.shape[1]):
                x_raw = self.grid_x[j]
                y_raw = self.grid_y[i]
                z = float(field[i, j])
                p = Point(x=float(x_raw), y=float(y_raw), z=z)
                c = ColorRGBA(r=norm[i, j], g=0.0, b=1.0 - norm[i, j], a=0.4)
                marker.points.append(p)
                marker.colors.append(c)
        self.field_pub.publish(marker)

    def vis_TroughPoints(self, troughs: List[Tuple[float, float]]):
        marker = Marker()
        marker.header.frame_id = 'os_sensor'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'Trough Points'
        marker.id = 1
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        marker.pose.orientation.w = 1.0
        for x, y in troughs:
            marker.points.append(Point(x=float(x), y=float(y), z=1.0))
        self.trough_pub.publish(marker)

    def vis_CarPosition(self):
        marker = Marker()
        marker.header.frame_id = 'os_sensor'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'Car Position'
        marker.id = 2
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        marker.pose.orientation.w = 1.0
        marker.points.append(Point(x=float(CAR_X), y=float(CAR_Y), z=1.0))
        self.carpos_pub.publish(marker)

    def vis_ValidPoints(self, points: List[Tuple[float, float]]):
        marker = Marker()
        marker.header.frame_id = 'os_sensor'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'Valid Trough Path'
        marker.id = 3
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        marker.pose.orientation.w = 1.0
        for x, y in points:
            marker.points.append(Point(x=float(x), y=float(y), z=1.5))
        self.valid_trough_pub.publish(marker)

    def vis_SplinedPath(self, points: List[Tuple[float, float]]):
        if not points:
            return
        # os_sensor → base_link 변환 시도
        bl_points = self._transform_points_to_baselink(points)

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()

        if bl_points is not None:
            # 정석: base_link 좌표로 퍼블리시
            path_msg.header.frame_id = "base_link"
            use_pts = bl_points
        else:
            # 변환 실패 시에도 좌표-라벨 일치 유지: os_sensor로 퍼블리시
            path_msg.header.frame_id = "os_sensor"
            use_pts = points

        path_msg.poses = []
        for x, y in use_pts:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.3
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.pub_splined_path.publish(path_msg)

    def vis_leftcone(self, cones: List[Tuple[float, float, float]]):
        marker = Marker()
        marker.header.frame_id = 'os_sensor'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'Left Cones'
        marker.id = 5
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
        marker.pose.orientation.w = 1.0
        for x, y, z in cones:
            marker.points.append(Point(x=float(x), y=float(y), z=float(z)))
        self.leftcone_pub.publish(marker)

    def vis_rightcone(self, cones: List[Tuple[float, float, float]]):
        marker = Marker()
        marker.header.frame_id = 'os_sensor'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'Right Cones'
        marker.id = 6
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        marker.pose.orientation.w = 1.0
        for x, y, z in cones:
            marker.points.append(Point(x=float(x), y=float(y), z=float(z)))
        self.rightcone_pub.publish(marker)

    def vis_RedCone(self):
        marker = Marker()
        marker.header.frame_id = 'os_sensor'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'Red Cones'
        marker.id = 7
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.pose.orientation.w = 1.0
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
        for x, y, z in self.red_cone:
            marker.points.append(Point(x=float(x), y=float(y), z=float(z)))
        self.redcone_pub.publish(marker)

    def vis_AEBzone(self, aeb_flag: int):
        marker = Marker()
        marker.header.frame_id = 'os_sensor'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'AEB Zone'
        marker.id = 8
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = AEB_VER
        marker.scale.y = 2 * AEB_HOR
        marker.scale.z = 0.1
        marker.pose.position.x = AEB_VER / 2.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=(0.8 if aeb_flag == 1 else 0.2))
        self.aeb_roi_pub.publish(marker)

    #=======================
    #  Main Loop
    #=======================
    def main_loop(self):
        '''
        1) 콘 좌표 수집 → 2) Potential Field → 3) Trough 탐색
        4) 전방 필터 + 근접 연결 → 5) Catmull–Rom 스플라인(+재샘플/스무딩)
        6) 경로 기준 좌/우 라벨링 → 7) 퍼블리시/시각화
        '''
        if len(self.cone_data) < 3:
            return

        # 1) Field 생성
        xy_cones = [(x, y) for x, y, _ in self.cone_data]
        PotentialField = self.create_potential_field(xy_cones)

        # 2) Trough 추출
        TroughPoints = self.get_trough_point(PotentialField)

        # 3) 전방 필터 + 유효 포인트 연결
        AheadTroughs = self._filter_ahead(TroughPoints)
        ValidPath = self.validation_filter(AheadTroughs)

        # 4) 스플라인 + 후처리
        SplinedPath = self.splining(ValidPath)

        # 5) 좌/우 라벨링
        left_cones, right_cones = self.cone_sort(self.frame_buffer[-1], SplinedPath)

        # 6) AEB
        aeb_flag = self.determine_aeb(self.red_cone)

        # 7) Labeled Cone Publish
        labeled_msg = ModifiedFloat32MultiArray()
        labeled_msg.header.frame_id = "os_sensor"
        labeled_msg.header.stamp = self.get_clock().now().to_msg()
        labeled_msg.data = []
        labeled_msg.class_names = []
        for cone in left_cones:
            labeled_msg.data.extend([cone[0], cone[1], cone[2]])
            labeled_msg.class_names.append("left")
        for cone in right_cones:
            labeled_msg.data.extend([cone[0], cone[1], cone[2]])
            labeled_msg.class_names.append("right")
        self.pub_labeled_cones.publish(labeled_msg)

        # 시각화
        self.vis_Field(PotentialField)
        self.vis_TroughPoints(TroughPoints)
        self.vis_CarPosition()
        self.vis_ValidPoints(ValidPath)
        self.vis_SplinedPath(SplinedPath)
        self.vis_leftcone(left_cones)
        self.vis_rightcone(right_cones)
        self.vis_RedCone()
        self.vis_AEBzone(aeb_flag)

def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldLabeler()
    node.get_logger().info('Potential Field Labeler Started (Catmull–Rom + resample + SG)')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
