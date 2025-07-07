#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cone_roi_publisher.py

• CSV의 WGS84 콘 절대좌표 → reference 평면(x,y) 변환
• TF(reference→gps_antenna→os_sensor)로 os_sensor 상대좌표 계산
• 전방 180°(x ≥ 0) & 반경 roi_radius 이내 콘만 선택
• /sorted_cones_time  (ModifiedFloat32MultiArray) 로 (x,y,z=0) 3-tuple 퍼블리시  ★
• /lidar_roi_marker   (visualization_msgs/Marker) 로 ROI 부채꼴 시각화
"""

import math, os
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from ament_index_python.packages import get_package_share_directory

from visualization_msgs.msg import Marker
from custom_interface.msg import ModifiedFloat32MultiArray

import pandas as pd
import numpy as np

import tf2_ros
import tf_transformations

R_EARTH = 6_378_137.0  # [m]


def latlon_to_local(lat: float, lon: float,
                    ref_lat: float, ref_lon: float) -> Tuple[float, float]:
    lat_r, lon_r = math.radians(lat), math.radians(lon)
    ref_lat_r, ref_lon_r = math.radians(ref_lat), math.radians(ref_lon)
    x = (lon_r - ref_lon_r) * math.cos(ref_lat_r) * R_EARTH
    y = (lat_r - ref_lat_r) * R_EARTH
    return x, y


class ConeROIPublisher(Node):
    def __init__(self):
        super().__init__("cone_roi_publisher")

        # ────── 파라미터 ──────
        pkg_share = get_package_share_directory("gps_global_planner")
        def_csv   = os.path.join(pkg_share, "data", "placed_cones_temp.csv")
        self.declare_parameter("csv_file",    def_csv)
        self.declare_parameter("ref_lat",     37.541274)
        self.declare_parameter("ref_lon",     127.077796)
        self.declare_parameter("roi_radius",  20.0)   # [m]
        self.declare_parameter("timer_hz",    10.0)   # [Hz]

        self.ref_lat  = self.get_parameter("ref_lat").value
        self.ref_lon  = self.get_parameter("ref_lon").value
        self.roi_R    = float(self.get_parameter("roi_radius").value)
        csv_file      = self.get_parameter("csv_file").value
        timer_hz      = float(self.get_parameter("timer_hz").value)

        # ────── CSV 로드 ──────
        try:
            df = pd.read_csv(csv_file)
        except Exception as e:
            self.get_logger().fatal(f"CSV 읽기 실패: {e}")
            raise e

        lat_col = [c for c in df.columns if "lat" in c.lower()][0]
        lon_col = [c for c in df.columns if "lon" in c.lower()][0]

        self.cones_ref: List[Tuple[float, float]] = [
            latlon_to_local(df.loc[i, lat_col],
                            df.loc[i, lon_col],
                            self.ref_lat, self.ref_lon)
            for i in range(len(df))
        ]
        self.get_logger().info(f"{len(self.cones_ref)} 개 콘 로드 완료")

        # ────── TF Buffer / Listener ──────
        self.tf_buf = tf2_ros.Buffer(cache_time=Duration(seconds=3))
        self.tf_lst = tf2_ros.TransformListener(self.tf_buf, self)

        # ────── 퍼블리셔 ──────
        self.pub_cone = self.create_publisher(
            ModifiedFloat32MultiArray, "/sorted_cones_time", 10)
        self.pub_roi  = self.create_publisher(
            Marker, "/lidar_roi_marker", 1)

        # ────── 타이머 ──────
        self.create_timer(1.0 / timer_hz, self.timer_cb)

        # ROI 마커(부채꼴) 템플릿
        self._roi_marker_template = self._create_roi_marker()

    # --------------------------------------------------------------
    def _create_roi_marker(self) -> Marker:
        """os_sensor 기준 전방 180° 부채꼴 LINE_STRIP"""
        mk = Marker()
        mk.header.frame_id = "os_sensor"
        mk.ns, mk.id = "roi", 0
        mk.type, mk.action = Marker.LINE_STRIP, Marker.ADD
        mk.scale.x = 0.05
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.0, 0.6, 1.0, 0.7

        R = self.roi_R
        angles = np.linspace(-math.pi/2, math.pi/2, 36)
        mk.points = [self._pt(R*math.cos(a), R*math.sin(a)) for a in angles]
        mk.points.insert(0, self._pt(0.0, 0.0))
        mk.points.append(self._pt(0.0, 0.0))
        return mk

    @staticmethod
    def _pt(x: float, y: float):
        from geometry_msgs.msg import Point
        p = Point(); p.x, p.y, p.z = x, y, 0.0
        return p

    # --------------------------------------------------------------
    def timer_cb(self):
        # TF(reference → os_sensor)
        try:
            tf = self.tf_buf.lookup_transform(
                "reference", "os_sensor", rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            self.get_logger().warn_once("TF(reference→os_sensor) 미획득…")
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        R = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        t_vec = np.array([t.x, t.y, t.z])
        RtT   = R.T                                   # 역회전

        # ROI 필터링
        rel_pts = []
        for cx, cy in self.cones_ref:
            p_ref = np.array([cx, cy, 0.0])
            p_s   = RtT.dot(p_ref - t_vec)            # os_sensor 좌표
            if p_s[0] < 0.0:               # 뒤쪽
                continue
            dist = np.hypot(p_s[0], p_s[1])
            if dist > self.roi_R:
                continue
            rel_pts.append((p_s[0], p_s[1], dist))

        rel_pts.sort(key=lambda x: x[2])              # 거리순 정렬

        # ModifiedFloat32MultiArray (x,y,z=0) 3-tuple ★
        msg = ModifiedFloat32MultiArray()
        msg.data = [coord
                    for (x, y, _) in rel_pts
                    for coord in (float(x), float(y), 0.0)]   # ★ z=0 추가
        self.pub_cone.publish(msg)

        # ROI 마커 time stamp 업데이트
        roi_mk = self._roi_marker_template
        roi_mk.header.stamp = self.get_clock().now().to_msg()
        self.pub_roi.publish(roi_mk)


# ------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ConeROIPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
