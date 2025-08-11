#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
reference_path_planning.py
─────────────────────────────────────────────────────────────
• 좌우 콘 → Delaunay 내부선 중점 → B-spline → 등간격 샘플로
  로컬 참조 경로( /final_waypoints ) 생성
• speed_planner 노드가 보내는 /desired_speed_profile 을 받아
  Waypoint.speed 를 갱신해 속도 막대(height=z) 시각화
(2025-07-22  - Delaunay-midpoint 기반 버전)
(2025-07-31~08/01  - speed_profile 길이 불일치 대응 & Marker 깜빡임 수정)
(2025-08-07  - ★ MOD: max_lane_width 파라미터 + Edge 거리 필터)
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

import tf2_ros
import tf_transformations


# ────────────────────────────────────────────────
class Waypoint:
    __slots__ = ("x", "y", "speed")

    def __init__(self, x: float, y: float, speed: float):
        self.x, self.y, self.speed = float(x), float(y), float(speed)


# ────────────────────────────────────────────────
class ReferencePathPlanner(Node):
    def __init__(self):
        super().__init__("reference_path_planning")

        # ── Pub -------------------------------------------------
        self.pub_path = self.create_publisher(Path, "/local_planned_path", 10)
        self.pub_lines = self.create_publisher(
            Marker, "/delaunay_internal_lines", 10
        )
        self.pub_midpts = self.create_publisher(
            Marker, "/delaunay_internal_midpoints", 10
        )
        self.pub_wps = self.create_publisher(
            MarkerArray, "/final_waypoints", 10
        )
        self.pub_speed = self.create_publisher(
            MarkerArray, "/waypoint_speed_bars", 10
        )

        # ── Sub -------------------------------------------------
        self.create_subscription(Marker, "/left_cone_marker", self.cb_left, 10)
        self.create_subscription(
            Marker, "/right_cone_marker", self.cb_right, 10
        )

        # speed_planner → 속도 배열
        self.speed_profile: List[float] = []
        self.create_subscription(
            Float32MultiArray,
            "/desired_speed_profile",
            self.cb_speed_profile,
            10,
        )

        # ── TF --------------------------------------------------
        self.sensor_frame = "base_link"
        self.ref_frame = "reference"
        self.tf_buf = tf2_ros.Buffer(cache_time=Duration(seconds=2))
        self.tf_lst = tf2_ros.TransformListener(self.tf_buf, self)

        # ── 상태 ------------------------------------------------
        self.left_ref: List[Tuple[float, float]] = []
        self.right_ref: List[Tuple[float, float]] = []
        self.latest_wps: List[Waypoint] = []  # 최근 Waypoints

        # ── 파라미터 -------------------------------------------
        self.arc_step = 0.5
        self.default_speed = 2.0
        self.scale_wp = 0.15
        self.min_bar_height = 0.01
        self.marker_life = MsgDuration()  # 0 = forever
        self.prev_wp_n = 0
        self._tf_warned = False

        self.last_valid_stamp = rclpy.time.Time()

        # ★ MOD: 좌·우 콘 사이 최대 허용 거리 [m]
        self.max_lane_width = 6.0

    # ==========================================================
    # Marker 콜백
    def cb_left(self, mk: Marker):
        if mk.type == Marker.SPHERE_LIST:
            self.left_ref = [(p.x, p.y) for p in mk.points]

    def cb_right(self, mk: Marker):
        if mk.type == Marker.SPHERE_LIST:
            self.right_ref = [(p.x, p.y) for p in mk.points]
            self.plan_path()

    # ==========================================================
    # speed_planner → 속도 배열
    def cb_speed_profile(self, msg: Float32MultiArray):
        """길이 불일치가 있어도 가능한 범위까지만 속도를 반영"""
        self.speed_profile = list(msg.data)

        # Waypoint가 아직 없으면 나중에 plan_path가 호출될 때 반영됨
        if not self.latest_wps:
            return

        n_wps = len(self.latest_wps)
        n_sp = len(self.speed_profile)
        if n_sp == 0:
            return

        n_common = min(n_wps, n_sp)
        for i in range(n_common):
            self.latest_wps[i].speed = self.speed_profile[i]

        for i in range(n_common, n_wps):
            self.latest_wps[i].speed = self.default_speed

        self.publish_waypoints(self.latest_wps, self.last_valid_stamp.to_msg())
        self.publish_waypoints(
            self.latest_wps, self.last_valid_stamp.to_msg()
        )

    # ==========================================================
    # ── Delaunay 내부선 중점 추출 ─────────────────────────
    # ★ MOD: staticmethod → 인스턴스 메서드 (self 사용)
    def delaunay_midpoints(
        self,
        left_s: List[Tuple[float, float]],
        right_s: List[Tuple[float, float]],
    ):
        """
        sensor 좌표계의 좌·우 포인트 → 내부선 중점 (N×2 ndarray)
        좌↔우 Edge만 남긴 뒤, 너무 먼 Edge는 필터링한 후
        최근접-이웃 순서로 정렬
        """
        pts = np.array(left_s + right_s, float)
        if len(pts) < 3:
            return np.empty((0, 2))

        tri = Delaunay(pts)
        lN = len(left_s)
        mids = []

        for s in tri.simplices:
            for a, b in ((s[0], s[1]), (s[1], s[2]), (s[2], s[0])):
                if (a < lN) != (b < lN):  # 좌 ↔ 우만
                    # ★ MOD: 거리 제한
                    if np.linalg.norm(pts[a] - pts[b]) > self.max_lane_width:
                        continue
                    mids.append(
                        [
                            (pts[a, 0] + pts[b, 0]) * 0.5,
                            (pts[a, 1] + pts[b, 1]) * 0.5,
                        ]
                    )

        if len(mids) < 3:
            return np.empty((0, 2))
        mids = np.array(mids, float)

        # ── 최근접-이웃으로 '적당한' 순서 정렬 ───────────────
        unvis = set(range(len(mids)))
        order = []
        cur = int(np.argmin(mids[:, 0]))  # 가장 x 작은 점부터
        while unvis:
            order.append(cur)
            unvis.discard(cur)
            if not unvis:
                break
            dist = ((mids[list(unvis)] - mids[cur]) ** 2).sum(1)
            cur = list(unvis)[int(np.argmin(dist))]
        return mids[order]

    # ── B-spline 보간 ────────────────────────────────────
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
            return (
                np.interp(tt, t, mids[:, 0]),
                np.interp(tt, t, mids[:, 1]),
            )

    # ==========================================================
    # 경로 계획
    def plan_path(self):
        if len(self.left_ref) < 3 or len(self.right_ref) < 3:
            return

        try:
            tf_s_r = self.tf_buf.lookup_transform(
                self.sensor_frame,
                self.ref_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            if not self._tf_warned:
                self.get_logger().warning(
                    "TF(sensor←reference) unavailable – waiting…"
                )
                self._tf_warned = True
            return

        self._tf_warned = False
        stamp_time = tf_s_r.header.stamp
        self.last_valid_stamp = rclpy.time.Time.from_msg(stamp_time)

        R_sr, t_sr = self.tf_to_mat(tf_s_r)

        left_s = sorted(
            [self.transform_pt(R_sr, t_sr, p) for p in self.left_ref],
            key=lambda p: p[0],
        )
        right_s = sorted(
            [self.transform_pt(R_sr, t_sr, p) for p in self.right_ref],
            key=lambda p: p[0],
        )

        # ── Delaunay 중점 → B-spline → 등간격 샘플 ────────
        mids = self.delaunay_midpoints(left_s, right_s)
        if mids.size == 0:
            return

        Cx, Cy = self.centerline_from_midpts(mids)
        wps = self.arc_sample(Cx, Cy)

        if self.speed_profile:
            n_common = min(len(wps), len(self.speed_profile))
            for i in range(n_common):
                wps[i].speed = self.speed_profile[i]

        self.latest_wps = wps

        self.publish_path(wps, stamp_time)
        self.publish_waypoints(wps, stamp_time)
        self.publish_delaunay(left_s, right_s, stamp_time)

    # ----------------------------------------------------------
    # 보조 함수
    @staticmethod
    def tf_to_mat(tf):
        t = tf.transform.translation
        q = tf.transform.rotation
        R = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])[ :3, :3]
        return R, np.array([t.x, t.y, t.z])

    @staticmethod
    def transform_pt(R, t, pt_ref):
        xs, ys, _ = R @ np.array([pt_ref[0], pt_ref[1], 0.0]) + t
        return float(xs), float(ys)

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
                px = X[i] - r * (X[i] - X[i - 1])
                py = Y[i] - r * (Y[i] - Y[i - 1])
                wps.append(Waypoint(px, py, self.default_speed))
                acc = 0.0
        wps.append(Waypoint(X[-1], Y[-1], self.default_speed))
        return wps

    # ----------------------------------------------------------
    # 퍼블리시
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

        arr = MarkerArray()
        bars = MarkerArray()

        for i, wp in enumerate(wps):
            m = Marker(
                header=header,
                ns="final_wp",
                id=i,
                type=Marker.SPHERE,
                action=Marker.ADD,
            )
            m.pose.position.x, m.pose.position.y = wp.x, wp.y
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = self.scale_wp
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 1.0, 1.0
            m.lifetime = self.marker_life
            arr.markers.append(m)

            bar_h = max(wp.speed, self.min_bar_height)
            b = Marker(
                header=header,
                ns="wp_speed",
                id=i,
                type=Marker.CUBE,
                action=Marker.ADD,
            )
            b.pose.position.x, b.pose.position.y = wp.x, wp.y
            b.pose.position.z = bar_h * 0.5
            b.pose.orientation.w = 1.0
            b.scale.x = b.scale.y = 0.2
            b.scale.z = bar_h
            b.color.r, b.color.g, b.color.b, b.color.a = 1.0, 0.0, 1.0, 0.4
            b.lifetime = self.marker_life
            bars.markers.append(b)

        for j in range(len(wps), self.prev_wp_n):
            arr.markers.append(
                Marker(
                    header=header,
                    ns="final_wp",
                    id=j,
                    action=Marker.DELETE,
                )
            )
            bars.markers.append(
                Marker(header=header, ns="wp_speed", id=j, action=Marker.DELETE)
            )

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

        lines = Marker(
            header=header,
            ns="bridge_lines",
            id=0,
            type=Marker.LINE_LIST,
            action=Marker.ADD,
        )
        lines.scale.x = 0.02
        lines.color.r = lines.color.g = lines.color.b = lines.color.a = 1.0
        lines.lifetime = self.marker_life

        mids = Marker(
            header=header,
            ns="bridge_midpts",
            id=0,
            type=Marker.SPHERE_LIST,
            action=Marker.ADD,
        )
        mids.scale.x = mids.scale.y = mids.scale.z = 0.3
        mids.color.r, mids.color.g, mids.color.b, mids.color.a = 0.0, 1.0, 1.0, 1.0
        mids.lifetime = self.marker_life

        for s in tri.simplices:
            for a, b in ((s[0], s[1]), (s[1], s[2]), (s[2], s[0])):
                if is_left(a) != is_left(b):
                    # ★ MOD: 시각화에도 동일한 거리 제한
                    if np.linalg.norm(pts[a] - pts[b]) > self.max_lane_width:
                        continue
                    pA = Point(x=float(pts[a, 0]), y=float(pts[a, 1]))
                    pB = Point(x=float(pts[b, 0]), y=float(pts[b, 1]))
                    lines.points.extend([pA, pB])
                    mids.points.append(
                        Point(x=(pA.x + pB.x) / 2, y=(pA.y + pB.y) / 2)
                    )

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
