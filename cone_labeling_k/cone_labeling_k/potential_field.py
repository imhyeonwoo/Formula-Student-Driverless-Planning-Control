#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
본 스크립트는 좌우 라벨링이 되지 않은 콘 데이터를 이용하여 Potential Field를 형성합니다. (PotentialField)
형성된 Potential Field의 Trough(골)을 필드 영역 전체에 대해 연산하여 저장합니다. (TroughPoints)
산출된 Trough(골) 좌표에서, 차량(0,0)과 가장 가까운 좌표를 시작으로, 
임계값(DIST_THRESH) 이하이면서 가장 가까운 좌표만 필터링하여 저장합니다. (ValidPath)
유효한 Trough(골) 좌표를 대상으로 B-Spline을 진행합니다. (SplinedPath)
모든 콘 좌표에 대해 SplinedPath 상의 최단거리 좌표까지의 직선(벡터)과, 해당 좌표에서의 접선(벡터)을 내적하여
SplinedPath 기준의 좌,우를 판단합니다. (left_cones, right_cones)
판단된 라벨링 데이터는 /labeled_cones 토픽의 class_names에 저장하고, 해당하는 좌표를 퍼블리시합니다.(pub_labeled_cones)
'''

'''''''''
주요 라이브러리 Import
'''''''''

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from visualization_msgs.msg import Marker
from custom_interface.msg import TrackedConeArray, ModifiedFloat32MultiArray
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA, UInt8
from collections import deque
import math
from math import hypot
import numpy as np
from typing import List, Tuple, Optional
from scipy.signal import savgol_filter
from scipy.interpolate import splprep, splev

from rclpy.qos import QoSProfile, ReliabilityPolicy
qos = QoSProfile(depth=10)
qos.reliability = ReliabilityPolicy.BEST_EFFORT

'''''''''
전역변수 선언
'''''''''

#----- Potential Field -----#
GRID_FORWARD = 10.0             #필드 형성
GRID_BACK = 5.0
GRID_LEFTRIGHT = 8.0
GRID_RES = 0.1                  #그리드 단위
AMP = 1.5                       #가우시안 필터 최댓값
SIGMA = 1.2                     #표준편차

#----- Car Position -----#
CAR_X, CAR_Y = 0,0

#----- Path -----#
DIST_THRESH = 2.0           #유효값 산출을 위한 좌표간 최대 거리 제한
FRAME_BUFFER = 1            #연산 대상 프레임 누적 개수
PATH_FORWARD = -0.05        #거리 전방 필터링
PATH_START_THRESH = 3.0     #시작거리 제한

#----- AEB -----#
AEB_TRHESH = 8
AEB_HOR = 6.0               #AEB 가로구간
AEB_VER = 8.0               #AEB 세로구간

'''
Ros2 Node 생성
'''
class PotentialFieldLabeler(Node):
    def __init__(self):
        super().__init__('Potential_Field_Labeler')

        ###### (선택) 스플라인/스무딩 파라미터 선언 ######
        # 런치에서 --ros-args -p resample_ds:=0.2 -p sg_window:=9 -p sg_poly:=3 -p cr_alpha:=0.5 -p samples_per_seg:=20 조정 가능
        self.declare_parameter('resample_ds', 0.2)      # 최종 경로 점 간격[m]
        self.declare_parameter('sg_window', 9)          # Savitzky–Golay 윈도(홀수)
        self.declare_parameter('sg_poly', 3)            # Savitzky–Golay 폴리 차수
        self.declare_parameter('cr_alpha', 0.7)         # Catmull–Rom alpha(0: uniform, 0.5: centripetal, 1: chordal)
        self.declare_parameter('samples_per_seg', 20)   # 스플라인 각 세그먼트 샘플 수(기본 20)
        
        ##### 필드 관련 파라미터 #####
        self.declare_parameter('GRID_Forward', GRID_FORWARD)      
        self.GRID_FORWARD = float(self.get_parameter('GRID_Forward').value)
        self.declare_parameter('GRID_Back', -8.0)         
        self.GRID_BACK = float(self.get_parameter('GRID_Back').value)
        self.declare_parameter('GRID_LeftRight', 10.0)    
        self.GRID_LEFTRIGHT = float(self.get_parameter('GRID_LeftRight').value)
        self.declare_parameter('FIELD_SIGMA', 1.0)    
        self.FIELD_SIGMA = float(self.get_parameter('FIELD_SIGMA').value)

        self.resample_ds = float(self.get_parameter('resample_ds').value)
        self.sg_window   = int(self.get_parameter('sg_window').value)
        self.sg_poly     = int(self.get_parameter('sg_poly').value)
        self.cr_alpha    = float(self.get_parameter('cr_alpha').value)
        self.samples_per_seg = int(self.get_parameter('samples_per_seg').value)

        ###### Subscriber ######
        # ----- Data -----#
        self.unknown_sub = self.create_subscription(
            TrackedConeArray,
            '/cone/lidar/ukf',
            self.cones_callback,
            qos)
        
        self.red_sub = self.create_subscription(
            TrackedConeArray,
            '/cone/fused/ukf',
            self.red_callback,
            qos)
        
        ###### Publisher ######
        self.pub_labeled_cones = self.create_publisher(ModifiedFloat32MultiArray, '/labeled_cones', 10)
        self.leftcone_pub = self.create_publisher(Marker, '/left_cone_marker', 10)
        self.rightcone_pub = self.create_publisher(Marker, '/right_cone_marker', 10)
        self.aeb_pub = self.create_publisher(UInt8, '/estop', 10)

        # -----Visualize -----#
        self.field_pub = self.create_publisher(Marker, '/potential_field', 10)
        self.trough_pub = self.create_publisher(Marker, '/trough_marker', 10)
        self.valid_trough_pub = self.create_publisher(Marker, '/valid_trough_marker', 10)
        self.carpos_pub = self.create_publisher(Marker, '/carpos_marker', 10)
        self.pub_splined_path = self.create_publisher(Path, '/local_planned_path', 10)
        self.redcone_pub = self.create_publisher(Marker, '/red_cone_marker', 10)
        self.aeb_roi_pub = self.create_publisher(Marker, '/aeb_roi', 10)

        #----- 그리드 및 좌표계 정의 -----#
        self.grid_x = np.arange(self.GRID_BACK, self.GRID_FORWARD + GRID_RES, GRID_RES)
        self.grid_y = np.arange(-self.GRID_LEFTRIGHT, self.GRID_LEFTRIGHT + GRID_RES, GRID_RES)
        self.X, self.Y   = np.meshgrid(self.grid_x, self.grid_y)

        #----- 콘 좌표 정의 -----#
        self.cone_data = []
        self.frame_buffer = deque(maxlen=FRAME_BUFFER)
        self.red_cone = []

        #----- TF 프레임 변환 -----#
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #----- 메인루프 주기적 실행 -----#
        self.create_timer(0.1, self.main_loop)

    # #----- Callback : Cone 데이터 저장 -----#
    def cones_callback(self, msg: TrackedConeArray):
        n_cones = len(msg.cones)

        # 데이터가 없으면 return
        if n_cones == 0:
            return 
        new_frame = []
        for _, cone in enumerate(msg.cones):
            # 각 cone은 TrackedCone 타입
            x, y, z = cone.position.x, cone.position.y, cone.position.z
            new_frame.append((x, y, z))

        self.frame_buffer.append(new_frame)

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

    #----- TF 관련 -----#
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
    
    #----- Potential Field 형성 -----#
    def create_potential_field(self, points: List[Tuple[float, float]]) -> np.ndarray:
        field = np.zeros_like(self.X)
        for x_i, y_i in points:
            field += AMP * np.exp(-((self.X - x_i)**2 + (self.Y - y_i)**2) / (2 * self.FIELD_SIGMA **2))
        return field
    
    #----- Trough(골) 좌표 산출 -----#
    def get_trough_point(self, field: np.ndarray) -> List[Tuple[float, float]]:
        trough_point: List[Tuple[float, float]] = []
        rows, cols = field.shape

        for i in range(1, rows - 1):
            for j in range(1, cols - 1):
                center = field[i, j]
                satisfied = 0
                #8방향 4쌍
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
            if float(np.dot(v, heading)) >= PATH_FORWARD:
                out.append((x, y))
        return out
    
    #----- 유효좌표 선정 -----#
    def validation_filter(self, troughpoints: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        if not troughpoints:
            return []

        heading = self._heading_in_os_sensor()
        # 전방 후보만
        ahead = []
        for p in troughpoints:
            v = np.array([p[0] - CAR_X, p[1] - CAR_Y], dtype=float)
            if float(np.dot(v, heading)) >= PATH_FORWARD:
                ahead.append(p)
        if not ahead:
            return []

        remaining = set(ahead)
        current = min(ahead, key=lambda p: np.hypot(p[0] - CAR_X, p[1] - CAR_Y))
        path = [current]
        remaining.remove(current)

        if np.hypot(current[0], current[1]) > PATH_START_THRESH:
            filtered = []
            return filtered

        while remaining:
            def score(p):
                v = np.array([p[0] - current[0], p[1] - current[1]], dtype=float)
                dist = float(np.hypot(v[0], v[1]))
                forward = float(np.dot(v, heading))
                pen = 1e3 if forward < PATH_FORWARD else 0.0
                return dist + pen

            next_point = min(remaining, key=score)
            vx, vy = (next_point[0] - current[0]), (next_point[1] - current[1])
            dist = math.hypot(vx, vy)
            # 거리 제한 + 뒤로 크게 가는 스텝 차단
            if dist > DIST_THRESH or np.dot([vx, vy], heading) < PATH_FORWARD:
                break

            path.append(next_point)
            remaining.remove(next_point)
            current = next_point

        return path

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

    def splining(self, path, ds: float = 0.2):
        """
        path: [(x,y), ...]
        return: 시작점부터 ds 간격(실제로는 전체 길이를 n등분)으로 리샘플링된 경로
        """
        # 0) 빈 입력 방어
        if not path:
            return []

        # 1) 너무 가까운 중복점 제거
        unique = [path[0]]
        for p in path[1:]:
            if hypot(p[0] - unique[-1][0], p[1] - unique[-1][1]) > 1e-3:
                unique.append(p)

        # 2) 점이 1~2개면 스플라인 대신 직선/점 리샘플링
        if len(unique) < 3:
            return self._resample_by_arclength(unique, ds)

        # 3) 스플라인 보간 후, 아크길이 기준으로 재샘플링
        x, y = zip(*unique)
        x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)

        try:
            from scipy.interpolate import splprep, splev
            k = min(3, len(unique) - 1)
            tck, _ = splprep([x, y], s=2.0, k=k)

            # 곡선 형상을 충분히 보존하기 위해 촘촘히 샘플링
            n_dense = max(200, len(unique) * 20)
            u_fine = np.linspace(0.0, 1.0, n_dense)
            x_f, y_f = splev(u_fine, tck)
            pts_dense = list(zip(x_f, y_f))

            return self._resample_by_arclength(pts_dense, ds)
        except Exception:
            # 스플라인 실패 시 원본(중복 제거된) 궤적을 그대로 거리기반 리샘플링
            return self._resample_by_arclength(unique, ds)
        
    #----- 좌,우 라벨링 -----#
    def cone_sort(self, cones: List[Tuple[float, float, float]], path: List[Tuple[float, float]]):
        if len(cones) == 0 or len(path) < 2:
            return [], []

        path_np = np.array(path)
        left_cones = []
        right_cones = []

        for cone in cones:
            cone_xy = np.array(cone[:2])
            dists = np.linalg.norm(path_np - cone_xy, axis=1)
            idx = np.argmin(dists)

            if idx < len(path_np) - 1:
                tangent = path_np[idx + 1] - path_np[idx]
            else:
                tangent = path_np[idx] - path_np[idx - 1]

            if np.linalg.norm(tangent) < 1e-3:
                continue

            tangent = tangent / np.linalg.norm(tangent)
            normal = np.array([-tangent[1], tangent[0]])
            rel = cone_xy - path_np[idx]
            side = np.dot(rel, normal)

            if side > 0:
                left_cones.append(cone)
            else:
                right_cones.append(cone)

        return left_cones, right_cones
    
    def determine_aeb(self, cone: List[Tuple[float, float, float]]):
        n_red = 0
        for x, y, z in cone:
            if 0 <= x <= AEB_VER and -AEB_HOR <= y <= AEB_HOR:
                n_red += 1
        aebmsg = UInt8()
        aebmsg.data = 1 if n_red > AEB_TRHESH else 0
        self.aeb_pub.publish(aebmsg)
        return aebmsg.data

    '''''''''''''''
    Vsualization
    '''''''''''''''

    #----- 퍼텐셜 필드 시각화 -----#
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
                c = ColorRGBA(r=norm[i, j], g=0.0, b=1.0 - norm[i, j], a=0.8)
                marker.points.append(p)
                marker.colors.append(c)

        self.field_pub.publish(marker)

    #----- 골 좌표 시각화 -----#
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
            p = Point(x=float(x), y=float(y), z=1.0)
            marker.points.append(p)

        self.trough_pub.publish(marker)

    #----- 차량 좌표 시각화 -----#
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

    #----- 유효 좌표 시각화 -----#
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
            p = Point(x=float(x), y=float(y), z=1.5)
            marker.points.append(p)

        self.valid_trough_pub.publish(marker)

    #----- Splined 경로 시각화 -----#
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

    #----- Left Cone -----#
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
        marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)  # 파란색
        marker.pose.orientation.w = 1.0

        for x, y, z in cones:
            marker.points.append(Point(x=float(x), y=float(y), z=float(z)))

        self.leftcone_pub.publish(marker)

    #----- Right Cone -----#
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
        marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # 노란색
        marker.pose.orientation.w = 1.0

        for x, y, z in cones:
            marker.points.append(Point(x=float(x), y=float(y), z=float(z)))

        self.rightcone_pub.publish(marker)

    #----- Red Cone -----#
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
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)  # 항상 동일한 빨간색

        for x, y, z in self.red_cone:
            marker.points.append(Point(x=float(x), y=float(y), z=float(z)))

        self.redcone_pub.publish(marker)

    #----- AEB ZONE -----#
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

        alpha = 0.8 if aeb_flag == 1 else 0.2
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=alpha)

        self.aeb_roi_pub.publish(marker)

    def main_loop(self):
        '''
        **파이프라인 구성**
        1. 구독한 Cone 데이터 x,y좌표 추출
        2. Potential Field 생성
        3. Field 영역 내 Trough(골) Point 탐색
        4. 근거리 기반 유효 좌표 필터링
        5. 스플라이닝 진행
        6. 경로 퍼블리시를 위한 등간격 리샘플링
        7. 법선 기반 좌우측 콘 Labeling
        8. 퍼블리시
        '''
        #데이터 오류 예외처리
        if len(self.cone_data) < 3:
            return

        #----- Potential Field 생성 -----#
        xy_cones = [(x, y) for x, y, _ in self.cone_data]
        PotentialField = self.create_potential_field(xy_cones)

        #----- Trough Point 탐색 -----#
        TroughPoints = self.get_trough_point(PotentialField)

        #----- 유효 좌표 필터링 -----#
        AheadTroughs = self._filter_ahead(TroughPoints)
        ValidPath = self.validation_filter(AheadTroughs)

        #----- Splining -----#
        SplinedPath = self.splining(ValidPath)

        #----- Cone Sorting -----#
        left_cones, right_cones = self.cone_sort(self.frame_buffer[-1], SplinedPath)

        #----- AEB Determination -----#
        aeb_flag = self.determine_aeb(self.red_cone)

        #----- Labeled Cone Publish -----#
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

        #----- Visualize -----#
        self.vis_Field(PotentialField)
        self.vis_TroughPoints(TroughPoints)
        self.vis_CarPosition()
        self.vis_ValidPoints(ValidPath)
        self.vis_SplinedPath(SplinedPath) #시각화와 경로 토픽 동시 퍼블리시
        self.vis_leftcone(left_cones)
        self.vis_rightcone(right_cones)
        self.vis_RedCone()
        self.vis_AEBzone(aeb_flag)

        return
    
def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldLabeler()
    node.get_logger().info('Potential Field Labeler Started')
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()