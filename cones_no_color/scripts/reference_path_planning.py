#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
reference_path_planning.py
────────────────────────────────────────────────────────────
입력
  • /left_cones_marker   (MarkerArray, frame=reference)
  • /right_cones_marker  (MarkerArray, frame=reference)
출력
  • /local_planned_path                 (Path)
  • /delaunay_internal_lines      (Marker)
  • /delaunay_internal_midpoints  (Marker)
  • /final_waypoints              (MarkerArray)
  • /waypoint_speed_bars          (MarkerArray)
모든 출력 좌표는 os_sensor 기준.
"""

import math
from typing import List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration as MsgDuration
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseStamped, Point
from nav_msgs.msg import Path
from scipy.spatial import Delaunay
from scipy.interpolate import make_interp_spline

import tf2_ros, tf_transformations


# ─── 웨이포인트 자료형 ───
class Waypoint:
    __slots__ = ("x", "y", "speed")

    def __init__(self, x: float, y: float, speed: float):
        self.x, self.y, self.speed = float(x), float(y), float(speed)


# ─── 메인 노드 ───
class ReferencePathPlanner(Node):
    def __init__(self):
        super().__init__("reference_path_planning")

        # ── 퍼블리셔 ──────────────────────────
        self.pub_path   = self.create_publisher(Path,        "/local_planned_path",         10)
        self.pub_lines  = self.create_publisher(Marker,      "/delaunay_internal_lines",    10)
        self.pub_midpts = self.create_publisher(Marker,      "/delaunay_internal_midpoints",10)
        self.pub_wps    = self.create_publisher(MarkerArray, "/final_waypoints",            10)
        self.pub_speed  = self.create_publisher(MarkerArray, "/waypoint_speed_bars",        10)

        # ── 구독자 ────────────────────────────
        self.create_subscription(MarkerArray, "/left_cones_marker",  self.cb_left,  10)
        self.create_subscription(MarkerArray, "/right_cones_marker", self.cb_right, 10)

        # ── TF ────────────────────────────────
        self.sensor_frame = "os_sensor"
        self.ref_frame    = "reference"
        self.tf_buf = tf2_ros.Buffer(rclpy.duration.Duration(seconds=2))
        self.tf_lst = tf2_ros.TransformListener(self.tf_buf, self)

        # ── 내부 저장 ──────────────────────────
        self.left_ref:  List[Tuple[float, float]] = []
        self.right_ref: List[Tuple[float, float]] = []

        # ── 파라미터 ──────────────────────────
        self.arc_step      = 0.5
        self.default_speed = 5.0
        self.scale_wp      = 0.15
        self.marker_life   = MsgDuration()   # 0 = 무한
        self.prev_wp_n     = 0
        self._tf_warned    = False           # 동일 경고 1회만

        self.get_logger().info("reference_path_planning ready (TF-aware)")

    # ===== 콜백 =====================================================
    def cb_left(self, msg: MarkerArray):
        self.left_ref = [(m.pose.position.x, m.pose.position.y)
                         for m in msg.markers if m.type == Marker.SPHERE]

    def cb_right(self, msg: MarkerArray):
        self.right_ref = [(m.pose.position.x, m.pose.position.y)
                          for m in msg.markers if m.type == Marker.SPHERE]
        self.plan_path()

    # ===== 경로 계획 =================================================
    def plan_path(self):
        if len(self.left_ref) < 3 or len(self.right_ref) < 3:
            return

        try:
            tf_s_r = self.tf_buf.lookup_transform(
                self.sensor_frame, self.ref_frame, rclpy.time.Time())
            if self._tf_warned:
                self._tf_warned = False
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            if not self._tf_warned:
                self.get_logger().warning("TF(reference→os_sensor) unavailable; waiting…")
                self._tf_warned = True
            return

        R_sr, t_sr = self.tf_to_mat(tf_s_r)

        left_s  = [self.transform_pt(R_sr, t_sr, p) for p in self.left_ref]
        right_s = [self.transform_pt(R_sr, t_sr, p) for p in self.right_ref]
        left_s.sort(key=lambda p: p[0])
        right_s.sort(key=lambda p: p[0])

        Lx, Ly = self.safe_spline(left_s)
        Rx, Ry = self.safe_spline(right_s)
        if Lx.size == 0 or Rx.size == 0:
            return
        Mx, My = 0.5*(Lx+Rx), 0.5*(Ly+Ry)
        wps = self.arc_sample(Mx, My)

        self.publish_path(wps)
        self.publish_waypoints(wps)
        self.publish_delaunay(left_s, right_s)

    # ===== 헬퍼 ======================================================
    @staticmethod
    def tf_to_mat(tf):
        t = tf.transform.translation
        q = tf.transform.rotation
        R = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])[:3,:3]
        return R, np.array([t.x, t.y, t.z])

    @staticmethod
    def transform_pt(R, t, pt_ref):
        xs, ys, _ = R @ np.array([pt_ref[0], pt_ref[1], 0.0]) + t
        return float(xs), float(ys)

    def safe_spline(self, pts, n=120):
        if len(pts) < 3:
            arr = np.array(pts, float)
            return arr[:,0], arr[:,1]
        arr = np.array(pts, float)
        t = np.arange(len(arr))
        try:
            sx = make_interp_spline(t, arr[:,0], k=3)
            sy = make_interp_spline(t, arr[:,1], k=3)
            t2 = np.linspace(0, len(arr)-1, n)
            return sx(t2), sy(t2)
        except Exception:
            t2 = np.linspace(0, len(arr)-1, n)
            return np.interp(t2, t, arr[:,0]), np.interp(t2, t, arr[:,1])

    def arc_sample(self, X, Y):
        wps = [Waypoint(X[0], Y[0], self.default_speed)]
        acc = 0.0
        for i in range(1, len(X)):
            seg = math.hypot(X[i]-X[i-1], Y[i]-Y[i-1])
            acc += seg
            if acc >= self.arc_step:
                r = (acc-self.arc_step)/seg
                px = X[i] - r*(X[i]-X[i-1])
                py = Y[i] - r*(Y[i]-Y[i-1])
                wps.append(Waypoint(px, py, self.default_speed))
                acc = 0.0
        wps.append(Waypoint(X[-1], Y[-1], self.default_speed))
        return wps

    # ===== 퍼블리시 ==================================================
    def publish_path(self, wps: List[Waypoint]):
        now = self.get_clock().now().to_msg()
        msg = Path()
        msg.header.stamp = now
        msg.header.frame_id = self.sensor_frame
        for wp in wps:
            ps = PoseStamped()
            ps.pose.position.x = wp.x
            ps.pose.position.y = wp.y
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    def publish_waypoints(self, wps: List[Waypoint]):
        now = self.get_clock().now().to_msg()
        header = Header(stamp=now, frame_id=self.sensor_frame)

        arr  = MarkerArray()
        bars = MarkerArray()

        # --- ADD / UPDATE ---
        for i, wp in enumerate(wps):
            m = Marker()
            m.header = header
            m.ns, m.id = "final_wp", i
            m.type, m.action = Marker.SPHERE, Marker.ADD
            m.pose = Pose()
            m.pose.position.x, m.pose.position.y = wp.x, wp.y
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = self.scale_wp
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 1.0, 1.0
            m.lifetime = self.marker_life
            arr.markers.append(m)

            b = Marker()
            b.header = header
            b.ns, b.id = "wp_speed", i
            b.type, b.action = Marker.CUBE, Marker.ADD
            b.pose = Pose()
            b.pose.position.x, b.pose.position.y = wp.x, wp.y
            b.pose.position.z = max(wp.speed, 0.01)*0.5
            b.pose.orientation.w = 1.0
            b.scale.x = b.scale.y = 0.2
            b.scale.z = max(wp.speed, 0.01)
            b.color.r, b.color.g, b.color.b, b.color.a = 0.0, 1.0, 0.0, 0.8
            b.lifetime = self.marker_life
            bars.markers.append(b)

        # --- DELETE 남은 id ---
        for j in range(len(wps), self.prev_wp_n):
            del_wp = Marker(header=header, ns="final_wp", id=j, action=Marker.DELETE)
            del_sp = Marker(header=header, ns="wp_speed", id=j, action=Marker.DELETE)
            arr.markers.append(del_wp)
            bars.markers.append(del_sp)

        self.prev_wp_n = len(wps)

        self.pub_wps.publish(arr)
        self.pub_speed.publish(bars)

    def publish_delaunay(self, left_s, right_s):
        pts = np.array(left_s + right_s, float)
        if len(pts) < 3:
            return
        tri = Delaunay(pts)
        lN = len(left_s); is_left = lambda i: i < lN
        now = self.get_clock().now().to_msg()

        lines = Marker()
        lines.header.frame_id = self.sensor_frame
        lines.header.stamp = now
        lines.ns = "bridge_lines"; lines.id = 0
        lines.type = Marker.LINE_LIST; lines.action = Marker.ADD
        lines.scale.x = 0.02
        lines.color.r = lines.color.g = lines.color.b = lines.color.a = 1.0
        lines.lifetime = self.marker_life

        mids = Marker()
        mids.header = lines.header
        mids.ns = "bridge_midpts"; mids.id = 0
        mids.type = Marker.SPHERE_LIST; mids.action = Marker.ADD
        mids.scale.x = mids.scale.y = mids.scale.z = 0.3
        mids.color.r,mids.color.g,mids.color.b,mids.color.a = 0.0,1.0,1.0,1.0
        mids.lifetime = self.marker_life

        for s in tri.simplices:
            for a,b in ((s[0],s[1]),(s[1],s[2]),(s[2],s[0])):
                if is_left(a) != is_left(b):
                    pA = Point(x=float(pts[a,0]), y=float(pts[a,1]))
                    pB = Point(x=float(pts[b,0]), y=float(pts[b,1]))
                    lines.points.extend([pA,pB])
                    mids.points.append(Point(x=(pA.x+pB.x)/2,
                                             y=(pA.y+pB.y)/2))

        self.pub_lines.publish(lines)
        self.pub_midpts.publish(mids)


# ----------------------------------------------------------------------
def main():
    rclpy.init()
    node = ReferencePathPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
