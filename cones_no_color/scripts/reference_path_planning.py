#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
reference_path_planning.py
─────────────────────────────────────────────────────────────
• 좌우 콘 → Delaunay 내부선 중점 → B-spline → 등간격 샘플로
  로컬 참조 경로(/final_waypoints) 생성
• base_link 근처에 콘이 부족하면 Cubic Bezier 브리지(3차 곡선)로
  차량 위치와 첫 mid-point를 부드럽게 연결
• /desired_speed_profile 로 Waypoint 속도 막대(height=z) 갱신
(마지막 수정: 2025-08-07)
"""

import math
from typing import List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Header, Float32MultiArray
from builtin_interfaces.msg import Duration as MsgDuration
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial import Delaunay
from scipy.interpolate import make_interp_spline

import tf2_ros, tf_transformations


# ────────────────────────────────────────────────
class Waypoint:
    __slots__ = ("x", "y", "speed")
    def __init__(self, x: float, y: float, speed: float):
        self.x, self.y, self.speed = float(x), float(y), float(speed)


# ────────────────────────────────────────────────
class ReferencePathPlanner(Node):
    def __init__(self):
        super().__init__("reference_path_planning")

        # ── Publisher -------------------------------------------------
        self.pub_path   = self.create_publisher(Path,        "/local_planned_path",          10)
        self.pub_lines  = self.create_publisher(Marker,      "/delaunay_internal_lines",     10)
        self.pub_midpts = self.create_publisher(Marker,      "/delaunay_internal_midpoints", 10)
        self.pub_wps    = self.create_publisher(MarkerArray, "/final_waypoints",             10)
        self.pub_speed  = self.create_publisher(MarkerArray, "/waypoint_speed_bars",         10)

        # ── Subscriber ------------------------------------------------
        self.create_subscription(Marker, "/left_cone_marker",  self.cb_left,  10)
        self.create_subscription(Marker, "/right_cone_marker", self.cb_right, 10)

        self.speed_profile: List[float] = []
        self.create_subscription(Float32MultiArray,
                                 "/desired_speed_profile",
                                 self.cb_speed_profile, 10)

        # ── TF --------------------------------------------------------
        self.sensor_frame = "base_link"
        self.ref_frame    = "reference"
        self.tf_buf = tf2_ros.Buffer(cache_time=Duration(seconds=2))
        self.tf_lst = tf2_ros.TransformListener(self.tf_buf, self)

        # ── 상태 ------------------------------------------------------
        self.left_ref:  List[Tuple[float, float]] = []
        self.right_ref: List[Tuple[float, float]] = []
        self.latest_wps: List[Waypoint] = []

        # ── 파라미터 --------------------------------------------------
        self.arc_step        = 0.5          # Waypoint 간격 [m]
        self.default_speed   = 0.0          # 기본 속도 [m/s]
        self.scale_wp        = 0.15
        self.min_bar_height  = 0.01
        self.marker_life     = MsgDuration()   # 0 = forever
        self.prev_wp_n       = 0
        self._tf_warned      = False

        # Cubic Bezier 브리지 설정
        self.anchor_max_gap  = 1.0          # 첫 mid-point 가 이 이상 멀면 브리지 생성 [m]
        self.ctrl_ratio      = 0.3          # 제어점 길이 계수 (0~1)

        self.last_valid_stamp = rclpy.time.Time()

        self.get_logger().info("ReferencePathPlanner with Bezier bridge started")

    # ==========================================================
    # 콜백
    def cb_left(self, mk: Marker):
        if mk.type == Marker.SPHERE_LIST:
            self.left_ref = [(p.x, p.y) for p in mk.points]

    def cb_right(self, mk: Marker):
        if mk.type == Marker.SPHERE_LIST:
            self.right_ref = [(p.x, p.y) for p in mk.points]
            self.plan_path()

    def cb_speed_profile(self, msg: Float32MultiArray):
        self.speed_profile = list(msg.data)
        if not self.latest_wps:
            return
        n_common = min(len(self.latest_wps), len(self.speed_profile))
        for i in range(n_common):
            self.latest_wps[i].speed = self.speed_profile[i]
        for i in range(n_common, len(self.latest_wps)):
            self.latest_wps[i].speed = self.default_speed
        self.publish_waypoints(self.latest_wps, self.last_valid_stamp.to_msg())

    # ==========================================================
    # 유틸
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

    # ==========================================================
    # Delaunay 내부선 중점
    @staticmethod
    def delaunay_midpoints(left_s, right_s):
        pts = np.array(left_s + right_s, float)
        if len(pts) < 3:
            return np.empty((0, 2))
        tri = Delaunay(pts)
        lN  = len(left_s)
        mids = []
        for s in tri.simplices:
            for a, b in ((s[0], s[1]), (s[1], s[2]), (s[2], s[0])):
                if (a < lN) != (b < lN):               # 좌↔우 Edge
                    mids.append([(pts[a, 0] + pts[b, 0]) * 0.5,
                                 (pts[a, 1] + pts[b, 1]) * 0.5])
        if len(mids) < 3:
            return np.empty((0, 2))
        mids = np.array(mids, float)

        # 최근접-이웃 순서화
        unvis = set(range(len(mids)))
        order = []
        cur   = int(np.argmin(mids[:, 0]))
        while unvis:
            order.append(cur)
            unvis.discard(cur)
            if not unvis:
                break
            dist = ((mids[list(unvis)] - mids[cur])**2).sum(1)
            cur  = list(unvis)[int(np.argmin(dist))]
        return mids[order]

    # Cubic Bezier 보조
    @staticmethod
    def bezier(p0, p1, p2, p3, t):
        u = 1.0 - t
        return (u**3)*p0 + 3*(u**2)*t*p1 + 3*u*(t**2)*p2 + (t**3)*p3

    # 중앙선 스플라인
    @staticmethod
    def centerline_from_midpts(mids: np.ndarray, n_fit: int = 300):
        t = np.arange(len(mids))
        try:
            sx = make_interp_spline(t, mids[:, 0], k=3)
            sy = make_interp_spline(t, mids[:, 1], k=3)
            tt = np.linspace(0, len(mids) - 1, n_fit)
            return sx(tt), sy(tt)
        except Exception:
            tt = np.linspace(0, len(mids) - 1, n_fit)
            return np.interp(tt, t, mids[:, 0]), np.interp(tt, t, mids[:, 1])

    # Waypoint 등간격 샘플
    def arc_sample(self, X, Y):
        if len(X) == 0:
            return []
        wps = [Waypoint(X[0], Y[0], self.default_speed)]
        acc = 0.0
        for i in range(1, len(X)):
            seg = math.hypot(X[i]-X[i-1], Y[i]-Y[i-1])
            acc += seg
            if acc >= self.arc_step:
                r  = (acc - self.arc_step) / seg
                px = X[i] - r * (X[i]-X[i-1])
                py = Y[i] - r * (Y[i]-Y[i-1])
                wps.append(Waypoint(px, py, self.default_speed))
                acc = 0.0
        wps.append(Waypoint(X[-1], Y[-1], self.default_speed))
        return wps

    # ==========================================================
    # 경로 계획
    def plan_path(self):
        if len(self.left_ref) < 3 or len(self.right_ref) < 3:
            return

        # TF(reference → sensor)
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
        stamp_time = tf_s_r.header.stamp
        self.last_valid_stamp = rclpy.time.Time.from_msg(stamp_time)

        R_sr, t_sr = self.tf_to_mat(tf_s_r)
        left_s  = sorted([self.transform_pt(R_sr, t_sr, p) for p in self.left_ref],
                         key=lambda p: p[0])
        right_s = sorted([self.transform_pt(R_sr, t_sr, p) for p in self.right_ref],
                         key=lambda p: p[0])

        mids = self.delaunay_midpoints(left_s, right_s)
        if mids.size == 0:
            return

        # ── Cubic Bezier 브리지 삽입 ─────────────────────────
        dx, dy = mids[0]
        dist   = math.hypot(dx, dy)
        if dist > self.anchor_max_gap:
            p0 = np.array([0.0, 0.0])             # base_link
            p3 = np.array([dx,  dy])              # 첫 mid-point
            u  = p3 / dist                        # 전방 단위벡터
            p1 = p0 + u * (self.ctrl_ratio * dist)
            p2 = p3 - u * (self.ctrl_ratio * dist)

            fine = np.array([self.bezier(p0, p1, p2, p3, t/100.0)
                             for t in range(1, 101)])   # t=0 제외
            bridge = []
            acc = 0.0
            last = p0
            for pt in fine:
                seg = math.hypot(*(pt - last))
                acc += seg
                if acc >= self.arc_step:
                    bridge.append(pt)
                    acc = 0.0
                    last = pt
            mids = np.vstack(([ [0.0, 0.0] ], bridge, mids))

        # ── B-spline → Waypoint 샘플 ───────────────────────
        Cx, Cy = self.centerline_from_midpts(mids)
        wps    = self.arc_sample(Cx, Cy)

        # 속도 프로필 반영
        if self.speed_profile:
            n_common = min(len(wps), len(self.speed_profile))
            for i in range(n_common):
                wps[i].speed = self.speed_profile[i]

        # 퍼블리시
        self.latest_wps = wps
        self.publish_path(wps, stamp_time)
        self.publish_waypoints(wps, stamp_time)
        self.publish_delaunay(left_s, right_s, stamp_time)

    # ==========================================================
    # 퍼블리시 함수들
    def publish_path(self, wps, stamp_time):
        msg = Path()
        msg.header.stamp = stamp_time
        msg.header.frame_id = self.sensor_frame
        for wp in wps:
            ps = PoseStamped()
            ps.pose.position.x, ps.pose.position.y = wp.x, wp.y
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    def publish_waypoints(self, wps, stamp_time):
        header = Header(stamp=stamp_time, frame_id=self.sensor_frame)
        arr  = MarkerArray()
        bars = MarkerArray()

        for i, wp in enumerate(wps):
            # Waypoint sphere
            m = Marker(header=header, ns="final_wp", id=i,
                       type=Marker.SPHERE, action=Marker.ADD)
            m.pose.position.x, m.pose.position.y = wp.x, wp.y
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = self.scale_wp
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 1.0, 1.0
            m.lifetime = self.marker_life
            arr.markers.append(m)

            # Speed bar cube
            bar_h = max(wp.speed, self.min_bar_height)
            b = Marker(header=header, ns="wp_speed", id=i,
                       type=Marker.CUBE, action=Marker.ADD)
            b.pose.position.x, b.pose.position.y = wp.x, wp.y
            b.pose.position.z = bar_h * 0.5
            b.pose.orientation.w = 1.0
            b.scale.x = b.scale.y = 0.2
            b.scale.z = bar_h
            b.color.r, b.color.g, b.color.b, b.color.a = 1.0, 0.0, 1.0, 0.4
            b.lifetime = self.marker_life
            bars.markers.append(b)

        # DELETE 여분 id
        for j in range(len(wps), self.prev_wp_n):
            arr.markers.append(Marker(header=header, ns="final_wp", id=j, action=Marker.DELETE))
            bars.markers.append(Marker(header=header, ns="wp_speed",  id=j, action=Marker.DELETE))

        self.prev_wp_n = len(wps)
        self.pub_wps.publish(arr)
        self.pub_speed.publish(bars)

    def publish_delaunay(self, left_s, right_s, stamp_time):
        pts = np.array(left_s + right_s, float)
        if len(pts) < 3:
            return
        tri = Delaunay(pts)
        lN = len(left_s)
        is_left = lambda i: i < lN

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
                    pA = Point(x=float(pts[a,0]), y=float(pts[a,1]))
                    pB = Point(x=float(pts[b,0]), y=float(pts[b,1]))
                    lines.points.extend([pA, pB])
                    mids.points.append(Point(x=(pA.x+pB.x)/2,
                                             y=(pA.y+pB.y)/2))

        self.pub_lines.publish(lines)
        self.pub_midpts.publish(mids)


# ─────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = ReferencePathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
