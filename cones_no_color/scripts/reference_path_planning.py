#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
reference_path_planning.py  (time-sync fixed 2025-07-08)
"""

import math
from typing import List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration as MsgDuration
from geometry_msgs.msg import Pose, PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial import Delaunay
from scipy.interpolate import make_interp_spline

import tf2_ros, tf_transformations


# ─────────────────────────────────────────────────────────────
class Waypoint:
    __slots__ = ("x", "y", "speed")
    def __init__(self, x: float, y: float, speed: float):
        self.x, self.y, self.speed = float(x), float(y), float(speed)


# ─────────────────────────────────────────────────────────────
class ReferencePathPlanner(Node):
    def __init__(self):
        super().__init__("reference_path_planning")

        # 퍼블리셔 ---------------------------------------------------
        self.pub_path   = self.create_publisher(Path,        "/local_planned_path",          10)
        self.pub_lines  = self.create_publisher(Marker,      "/delaunay_internal_lines",     10)
        self.pub_midpts = self.create_publisher(Marker,      "/delaunay_internal_midpoints", 10)
        self.pub_wps    = self.create_publisher(MarkerArray, "/final_waypoints",             10)
        self.pub_speed  = self.create_publisher(MarkerArray, "/waypoint_speed_bars",         10)

        # 구독 -------------------------------------------------------
        self.create_subscription(Marker, "/left_cone_marker",  self.cb_left,  10)
        self.create_subscription(Marker, "/right_cone_marker", self.cb_right, 10)

        # TF ---------------------------------------------------------
        self.sensor_frame = "os_sensor"
        self.ref_frame    = "reference"
        self.tf_buf = tf2_ros.Buffer(cache_time=Duration(seconds=2))
        self.tf_lst = tf2_ros.TransformListener(self.tf_buf, self)

        # 상태 -------------------------------------------------------
        self.left_ref:  List[Tuple[float, float]] = []
        self.right_ref: List[Tuple[float, float]] = []

        # 파라미터 ---------------------------------------------------
        self.arc_step      = 0.5
        self.default_speed = 5.0
        self.scale_wp      = 0.15
        self.marker_life   = MsgDuration()  # 0 = forever
        self.prev_wp_n     = 0
        self._tf_warned    = False

    # ==========================================================
    # 콜백
    def cb_left(self, mk: Marker):
        if mk.type == Marker.SPHERE_LIST:
            self.left_ref = [(p.x, p.y) for p in mk.points]

    def cb_right(self, mk: Marker):
        if mk.type == Marker.SPHERE_LIST:
            self.right_ref = [(p.x, p.y) for p in mk.points]
            self.plan_path()

    # ==========================================================
    # 경로 계획
    def plan_path(self):
        if len(self.left_ref) < 3 or len(self.right_ref) < 3:
            return

        # 변환(reference → sensor) 조회
        try:
            tf_s_r = self.tf_buf.lookup_transform(
                self.sensor_frame, self.ref_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.2))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            if not self._tf_warned:
                self.get_logger().warning("TF(sensor←reference) unavailable – waiting…")
                self._tf_warned = True
            return
        self._tf_warned = False

        # ▶️ TF 시각 기록 ◀️
        stamp_time = tf_s_r.header.stamp        # --- 핵심 수정 ---

        # 행렬
        R_sr, t_sr = self.tf_to_mat(tf_s_r)

        left_s  = sorted([self.transform_pt(R_sr, t_sr, p) for p in self.left_ref],
                         key=lambda p: p[0])
        right_s = sorted([self.transform_pt(R_sr, t_sr, p) for p in self.right_ref],
                         key=lambda p: p[0])

        Lx, Ly = self.safe_spline(left_s)
        Rx, Ry = self.safe_spline(right_s)
        if Lx.size == 0 or Rx.size == 0:
            return
        Mx, My = 0.5*(Lx + Rx), 0.5*(Ly + Ry)
        wps = self.arc_sample(Mx, My)

        # 퍼블리시 (모든 header.stamp = stamp_time)
        self.publish_path(wps, stamp_time)
        self.publish_waypoints(wps, stamp_time)
        self.publish_delaunay(left_s, right_s, stamp_time)

    # ----------------------------------------------------------
    # 보조 함수
    @staticmethod
    def tf_to_mat(tf):
        t = tf.transform.translation
        q = tf.transform.rotation
        R = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        return R, np.array([t.x, t.y, t.z])

    @staticmethod
    def transform_pt(R, t, pt_ref):
        xs, ys, _ = R @ np.array([pt_ref[0], pt_ref[1], 0.0]) + t
        return float(xs), float(ys)

    @staticmethod
    def safe_spline(pts, n=120):
        if len(pts) < 3:
            arr = np.array(pts, float)
            return arr[:, 0], arr[:, 1] if len(arr) else (np.array([]), np.array([]))

        arr = np.array(pts, float)
        t = np.arange(len(arr))
        try:
            sx = make_interp_spline(t, arr[:, 0], k=3)
            sy = make_interp_spline(t, arr[:, 1], k=3)
            t2 = np.linspace(0, len(arr) - 1, n)
            return sx(t2), sy(t2)
        except Exception:
            t2 = np.linspace(0, len(arr) - 1, n)
            return np.interp(t2, t, arr[:, 0]), np.interp(t2, t, arr[:, 1])

    def arc_sample(self, X, Y):
        if len(X) == 0:
            return []
        wps = [Waypoint(X[0], Y[0], self.default_speed)]
        acc = 0.0
        for i in range(1, len(X)):
            seg = math.hypot(X[i] - X[i - 1], Y[i] - Y[i - 1])
            acc += seg
            if acc >= self.arc_step:
                r = (acc - self.arc_step) / seg
                px = X[i] - r*(X[i] - X[i - 1])
                py = Y[i] - r*(Y[i] - Y[i - 1])
                wps.append(Waypoint(px, py, self.default_speed))
                acc = 0.0
        wps.append(Waypoint(X[-1], Y[-1], self.default_speed))
        return wps

    # ----------------------------------------------------------
    # 퍼블리시(모두 stamp_time 사용)
    def publish_path(self, wps: List[Waypoint], stamp_time):
        msg = Path()
        msg.header.stamp = stamp_time
        msg.header.frame_id = self.sensor_frame
        for wp in wps:
            ps = PoseStamped()
            ps.pose.position.x, ps.pose.position.y = wp.x, wp.y
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    def publish_waypoints(self, wps: List[Waypoint], stamp_time):
        header = Header(stamp=stamp_time, frame_id=self.sensor_frame)

        arr  = MarkerArray()
        bars = MarkerArray()

        for i, wp in enumerate(wps):
            m = Marker(header=header, ns="final_wp", id=i,
                       type=Marker.SPHERE, action=Marker.ADD)
            m.pose.position.x, m.pose.position.y = wp.x, wp.y
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = self.scale_wp
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 1.0, 1.0
            m.lifetime = self.marker_life
            arr.markers.append(m)

            b = Marker(header=header, ns="wp_speed", id=i,
                       type=Marker.CUBE, action=Marker.ADD)
            b.pose.position.x, b.pose.position.y = wp.x, wp.y
            b.pose.position.z = max(wp.speed, 0.01) * 0.5
            b.pose.orientation.w = 1.0
            b.scale.x = b.scale.y = 0.2
            b.scale.z = max(wp.speed, 0.01)
            b.color.r, b.color.g, b.color.b, b.color.a = 0.0, 1.0, 0.0, 0.8
            b.lifetime = self.marker_life
            bars.markers.append(b)

        # DELETE 남은 id
        for j in range(len(wps), self.prev_wp_n):
            arr.markers.append(Marker(header=header, ns="final_wp", id=j, action=Marker.DELETE))
            bars.markers.append(Marker(header=header, ns="wp_speed", id=j, action=Marker.DELETE))

        self.prev_wp_n = len(wps)
        self.pub_wps.publish(arr)
        self.pub_speed.publish(bars)

    def publish_delaunay(self, left_s, right_s, stamp_time):
        pts = np.array(left_s + right_s, float)
        if len(pts) < 3:
            return
        tri = Delaunay(pts)
        lN = len(left_s); is_left = lambda i: i < lN

        header = Header(stamp=stamp_time, frame_id=self.sensor_frame)

        lines = Marker(header=header, ns="bridge_lines", id=0,
                       type=Marker.LINE_LIST, action=Marker.ADD)
        lines.scale.x = 0.02
        lines.color.r = lines.color.g = lines.color.b = lines.color.a = 1.0
        lines.lifetime = self.marker_life

        mids = Marker(header=header, ns="bridge_midpts", id=0,
                      type=Marker.SPHERE_LIST, action=Marker.ADD)
        mids.scale.x = mids.scale.y = mids.scale.z = 0.3
        mids.color.r, mids.color.g, mids.color.b, mids.color.a = 0.0, 1.0, 1.0, 1.0
        mids.lifetime = self.marker_life

        for s in tri.simplices:
            for a, b in ((s[0], s[1]), (s[1], s[2]), (s[2], s[0])):
                if is_left(a) != is_left(b):
                    pA = Point(x=float(pts[a, 0]), y=float(pts[a, 1]))
                    pB = Point(x=float(pts[b, 0]), y=float(pts[b, 1]))
                    lines.points.extend([pA, pB])
                    mids.points.append(Point(x=(pA.x + pB.x) / 2,
                                             y=(pA.y + pB.y) / 2))

        self.pub_lines.publish(lines)
        self.pub_midpts.publish(mids)


# ----------------------------------------------------------------------
def main():
    rclpy.init()
    node = ReferencePathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
