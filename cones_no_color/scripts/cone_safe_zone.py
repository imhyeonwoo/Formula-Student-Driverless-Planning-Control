#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cone_safe_zone.py  ─ TF 적용 & 면(scale) 보정
─────────────────────────────────────────────────────────────
1) /left_cone_marker · /right_cone_marker  (reference, SPHERE_LIST)
2) reference → os_sensor 변환
3) /cone_radius_rings   (MarkerArray, CYLINDER)
   /drivable_corridor   (Marker,       TRIANGLE_LIST & LINE_STRIP)
"""

from typing import List, Tuple
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from builtin_interfaces.msg import Duration as MsgDuration
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import tf2_ros, tf_transformations


class ConeSafeZone(Node):
    def __init__(self):
        super().__init__("cone_safe_zone")

        # ─ Parameters ─────────────────────────────────────────
        self.declare_parameter("reference_frame", "reference")
        self.declare_parameter("sensor_frame",    "os_sensor")
        self.declare_parameter("default_radius",   0.8)
        self.declare_parameter("corridor_res",       120)
        self.declare_parameter("lifetime_sec",      0.30)

        self.ref_frame    = self.get_parameter("reference_frame").value
        self.sensor_frame = self.get_parameter("sensor_frame").value
        self.radius       = float(self.get_parameter("default_radius").value)
        self.n_samples    = int(self.get_parameter("corridor_res").value)

        life_f = float(self.get_parameter("lifetime_sec").value)
        self.lifetime = MsgDuration(
            sec=int(life_f),
            nanosec=int((life_f - int(life_f))*1e9))

        # ─ TF listener ────────────────────────────────────────
        self.tf_buf = tf2_ros.Buffer(cache_time=Duration(seconds=2))
        self.tf_lst = tf2_ros.TransformListener(self.tf_buf, self)

        # ─ Publishers ─────────────────────────────────────────
        self.pub_rings    = self.create_publisher(
            MarkerArray, "/cone_radius_rings", 10)
        self.pub_corridor = self.create_publisher(
            Marker,      "/drivable_corridor", 10)

        # ─ Subscribers ────────────────────────────────────────
        self.create_subscription(
            Marker, "/left_cone_marker",  self.cb_left,  10)
        self.create_subscription(
            Marker, "/right_cone_marker", self.cb_right, 10)

        # ─ State ──────────────────────────────────────────────
        self.left_pts_ref:  List[Tuple[float,float,float]] = []
        self.right_pts_ref: List[Tuple[float,float,float]] = []
        self.prev_ring_n = 0

        self.get_logger().info("Cone Safe-Zone node ready.")

    # =========================================================
    # Callbacks
    def cb_left(self, mk: Marker):
        if mk.type == Marker.SPHERE_LIST:
            self.left_pts_ref = [(p.x, p.y, p.z) for p in mk.points]
            self.update()

    def cb_right(self, mk: Marker):
        if mk.type == Marker.SPHERE_LIST:
            self.right_pts_ref = [(p.x, p.y, p.z) for p in mk.points]
            self.update()

    # ---------------------------------------------------------
    def update(self):
        if len(self.left_pts_ref) < 3 or len(self.right_pts_ref) < 3:
            return

        # TF 변환
        try:
            tf_s_r = self.tf_buf.lookup_transform(
                self.sensor_frame, self.ref_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.2))
            R, t = self.tf_to_mat(tf_s_r)
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            self.get_logger().warn("TF unavailable – skip frame.")
            return

        left_s  = [self.transform_pt(R, t, p) for p in self.left_pts_ref]
        right_s = [self.transform_pt(R, t, p) for p in self.right_pts_ref]

        self.publish_radius_rings(left_s + right_s)
        self.publish_corridor(left_s, right_s)

    # =========================================================
    # Publishing helpers
    def publish_radius_rings(self, cones):
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()

        for idx, (x, y, z) in enumerate(cones):
            m = Marker()
            m.header.stamp    = now
            m.header.frame_id = self.sensor_frame
            m.ns, m.id        = "cone_radius", idx
            m.type, m.action  = Marker.CYLINDER, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 2.0 * self.radius
            m.scale.z = 0.05
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 0.35
            m.lifetime = self.lifetime
            arr.markers.append(m)

        for j in range(len(cones), self.prev_ring_n):
            arr.markers.append(Marker(ns="cone_radius", id=j,
                                      action=Marker.DELETE))
        self.prev_ring_n = len(cones)
        self.pub_rings.publish(arr)

    # ---------------------------------------------------------
    def publish_corridor(self, left_pts, right_pts):
        Lx, Ly = self.safe_spline([(x,y) for x,y,_ in left_pts], self.n_samples)
        Rx, Ry = self.safe_spline([(x,y) for x,y,_ in right_pts], self.n_samples)
        if Lx.size == 0 or Rx.size == 0:
            return

        inner_left, inner_right = [], []
        for lx, ly, rx, ry in zip(Lx, Ly, Rx, Ry):
            vec = np.array([rx - lx, ry - ly]); nrm = np.linalg.norm(vec)
            if nrm < 1e-6: continue
            unit = vec / nrm
            inner_left.append ((lx + self.radius*unit[0],
                                ly + self.radius*unit[1]))
            inner_right.append((rx - self.radius*unit[0],
                                ry - self.radius*unit[1]))

        if len(inner_left) < 3: return
        now = self.get_clock().now().to_msg()

        # ─ 면(TRIANGLE_LIST) ────────────────────────────────
        tri = Marker()
        tri.header.stamp    = now
        tri.header.frame_id = self.sensor_frame
        tri.ns, tri.id      = "drivable_zone", 0
        tri.type, tri.action= Marker.TRIANGLE_LIST, Marker.ADD
        tri.pose.orientation.w = 1.0
        tri.scale.x = tri.scale.y = tri.scale.z = 1.0      # ★ scale ≠ 0
        tri.color.r, tri.color.g, tri.color.b, tri.color.a = 0.0, 0.3, 1.0, 0.5
        tri.lifetime = self.lifetime

        for i in range(len(inner_left)-1):
            p0, p1 = self._pt(inner_left[i]),  self._pt(inner_right[i])
            p2, p3 = self._pt(inner_left[i+1]), self._pt(inner_right[i+1])
            tri.points.extend([p0, p1, p2,  p2, p1, p3])

        # ─ 경계선(Line Strip) ───────────────────────────────
        line_l = Marker(header=tri.header, ns="drivable_lines", id=1,
                        type=Marker.LINE_STRIP, action=Marker.ADD)
        line_l.pose.orientation.w = 1.0
        line_l.scale.x = 0.05
        line_l.color.r, line_l.color.g, line_l.color.b, line_l.color.a = 0.0, 1.0, 1.0, 0.8
        line_l.points = [self._pt(p) for p in inner_left]

        line_r = Marker(header=tri.header, ns="drivable_lines", id=2,
                        type=Marker.LINE_STRIP, action=Marker.ADD)
        line_r.pose.orientation.w = 1.0
        line_r.scale.x = 0.05
        line_r.color.r, line_r.color.g, line_r.color.b, line_r.color.a = 0.0, 1.0, 1.0, 0.8
        line_r.points = [self._pt(p) for p in inner_right]

        # 퍼블리시
        self.pub_corridor.publish(tri)
        self.pub_corridor.publish(line_l)
        self.pub_corridor.publish(line_r)

    # =========================================================
    # Utility functions
    @staticmethod
    def tf_to_mat(tf_msg):
        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        R = tf_transformations.quaternion_matrix([q.x,q.y,q.z,q.w])[:3,:3]
        return R, np.array([t.x,t.y,t.z])

    @staticmethod
    def transform_pt(R, t, p):
        return tuple((R @ np.array([p[0],p[1],p[2]]) + t).tolist())

    @staticmethod
    def _pt(tup):
        return Point(x=float(tup[0]), y=float(tup[1]), z=0.02)

    @staticmethod
    def safe_spline(pts, n=120):
        if len(pts) < 3:
            arr = np.array(pts, float)
            return (arr[:,0], arr[:,1]) if len(arr) else (np.array([]), np.array([]))
        from scipy.interpolate import make_interp_spline
        arr = np.array(pts, float); t = np.arange(len(arr))
        try:
            sx, sy = make_interp_spline(t,arr[:,0],k=3), make_interp_spline(t,arr[:,1],k=3)
            t2 = np.linspace(0,len(arr)-1,n)
            return sx(t2), sy(t2)
        except Exception:
            t2 = np.linspace(0,len(arr)-1,n)
            return np.interp(t2,t,arr[:,0]), np.interp(t2,t,arr[:,1])


# ─────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = ConeSafeZone()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
