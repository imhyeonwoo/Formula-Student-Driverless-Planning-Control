#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
classify_cones_by_side.py  (2025-07-08)

목표
────
1. /global_path_marker   (Marker, LINE_STRIP,  reference frame)  ↘
2. /sorted_cones_time    (ModifiedFloat32MultiArray, os_sensor)  ↘  Frenet 좌/우 분류
3. TF(reference ← os_sensor)                                     ↗
   └→ /left_cone_marker  (Marker, Spheres, blue)   – 좌측 콘
   └→ /right_cone_marker (Marker, Spheres, yellow) – 우측 콘
"""

from typing import List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from custom_interface.msg import ModifiedFloat32MultiArray

import tf2_ros
import tf_transformations


# ────────────────────────────────────────────────────────────────
class ConeSideClassifier(Node):
    def __init__(self):
        super().__init__("classify_cones_by_side")

        # ─── 구독 ───
        self.create_subscription(
            Marker, "/global_path_marker", self.cb_path, 1)
        self.create_subscription(
            ModifiedFloat32MultiArray, "/sorted_cones_time", self.cb_cones, 10)

        # ─── 퍼블리셔 ───
        self.pub_left  = self.create_publisher(Marker, "/left_cone_marker",  1)
        self.pub_right = self.create_publisher(Marker, "/right_cone_marker", 1)

        # ─── TF ───
        self.tf_buf = tf2_ros.Buffer(cache_time=Duration(seconds=2))
        self.tf_lst = tf2_ros.TransformListener(self.tf_buf, self)

        # ─── 경로 데이터 ───
        self.path_pts: np.ndarray = np.empty((0, 2))  # [N,2]
        self.seg_len: np.ndarray  = np.empty(0)       # [N-1]
        self.cum_s:   np.ndarray  = np.empty(0)       # [N]

        # ─── 플래그 ───
        self._warn_no_path = False
        self._warn_no_tf   = False

        self.get_logger().info("ConeSideClassifier node started")

    # ----------------------------------------------------------
    # 1) 경로 콜백
    def cb_path(self, msg: Marker):
        if msg.type != Marker.LINE_STRIP or not msg.points:
            return

        self.path_pts = np.array([(p.x, p.y) for p in msg.points])
        if len(self.path_pts) < 2:
            return

        # 세그먼트 길이 & 누적 s
        diff      = self.path_pts[1:] - self.path_pts[:-1]        # [N-1,2]
        self.seg_len = np.linalg.norm(diff, axis=1)               # [N-1]
        self.cum_s   = np.hstack(([0.0], np.cumsum(self.seg_len)))# [N]

        self._warn_no_path = False
        self.get_logger().info(f"경로 수신: {len(self.path_pts)} pts")

    # ----------------------------------------------------------
    # 2) 콘 콜백
    def cb_cones(self, msg: ModifiedFloat32MultiArray):
        # 경로 확인
        if len(self.path_pts) < 2:
            if not self._warn_no_path:
                self.get_logger().warn("경로 미수신 – 분류 보류")
                self._warn_no_path = True
            return

        # TF(reference ← os_sensor)
        try:
            tf = self.tf_buf.lookup_transform(
                "reference", "os_sensor", rclpy.time.Time())
            self._warn_no_tf = False
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            if not self._warn_no_tf:
                self.get_logger().warn("TF(reference←os_sensor) 미획득 – 분류 보류")
                self._warn_no_tf = True
            return

        # 변환 행렬
        R = tf_transformations.quaternion_matrix(
            [tf.transform.rotation.x,
             tf.transform.rotation.y,
             tf.transform.rotation.z,
             tf.transform.rotation.w])[:3, :3]
        t = np.array([tf.transform.translation.x,
                      tf.transform.translation.y,
                      tf.transform.translation.z])

        # 콘 좌표 (os_sensor → reference)
        cones_ref = []
        data = msg.data
        for i in range(0, len(data), 3):
            p_s = np.array([data[i], data[i + 1], 0.0])
            p_r = R @ p_s + t
            cones_ref.append(p_r[:2])

        # 좌/우 분류
        left_pts, right_pts = self._classify_cones(cones_ref)

        # 마커 발행
        stamp = self.get_clock().now().to_msg()
        self._publish_marker(self.pub_left,  left_pts,  stamp,
                             ns="left",  mid=0, color=(0.0, 0.3, 1.0))
        self._publish_marker(self.pub_right, right_pts, stamp,
                             ns="right", mid=0, color=(1.0, 1.0, 0.0))

    # ----------------------------------------------------------
    # 3) Frenet 기반 좌/우 분류
    def _classify_cones(self, cones: List[np.ndarray]):
        """
        반환: (left_points, right_points)
        """
        path   = self.path_pts
        left, right = [], []

        for c in cones:
            # a) 최단 점·세그먼트 거리 탐색
            best_dist2 = float('inf')
            best_i     = 0
            best_u     = 0.0
            best_proj  = None

            for i in range(len(path) - 1):
                p, q = path[i], path[i + 1]
                v    = q - p
                v2   = np.dot(v, v)
                if v2 == 0.0:
                    continue
                u = np.clip(np.dot(c - p, v) / v2, 0.0, 1.0)
                proj = p + u * v
                dist2 = np.dot(c - proj, c - proj)
                if dist2 < best_dist2:
                    best_dist2, best_i, best_u, best_proj = dist2, i, u, proj

            # b) 접선 벡터 & 외적 z
            t_hat = path[best_i + 1] - path[best_i]
            d_vec = c - best_proj
            z     = t_hat[0] * d_vec[1] - t_hat[1] * d_vec[0]

            # c) 분류
            (left if z > 0 else right).append(c)

        return left, right

    # ----------------------------------------------------------
    # 4) Marker helper
    def _publish_marker(self,
                        pub,
                        pts: List[np.ndarray],
                        stamp,
                        *,
                        ns: str,
                        mid: int,
                        color: Tuple[float, float, float]):
        mk = Marker()
        mk.header.stamp    = stamp
        mk.header.frame_id = "reference"
        mk.ns, mk.id       = ns, mid
        mk.type, mk.action = Marker.SPHERE_LIST, Marker.ADD
        mk.scale.x = mk.scale.y = mk.scale.z = 0.3
        mk.color.r, mk.color.g, mk.color.b = color
        mk.color.a = 1.0
        mk.points = [Point(x=float(p[0]), y=float(p[1]), z=0.0) for p in pts]
        pub.publish(mk)


# ────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = ConeSideClassifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
