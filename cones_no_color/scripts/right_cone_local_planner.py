#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Right-side cone based local path planner.

- Subscribes: /cone/lidar/ukf (custom_interface/msg/TrackedConeArray)
- Publishes:  /local_planned_path (nav_msgs/Path)
- Frame:      base_link (forced, like cone_labeling_hsm/path_planning.py)

Strategy (simple & robust):
- Use only right-side cones (y < 0) in the incoming sensor frame.
- Sort by x (forward) and estimate tangent along the sequence.
- Offset each right wall point leftward by a fixed half-lane width to synthesize a centerline.
- Resample to fixed spacing and anchor from (0,0) forward for a fixed length.

Note: This node forces Path frame_id to 'base_link' without TF conversion, mirroring
      the behavior in cone_labeling_hsm/path_planning.py (FORCE_FALLBACK_FRAME = True).
"""

from typing import List, Tuple, Optional
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from custom_interface.msg import TrackedConeArray


class RightConeLocalPlanner(Node):
    def __init__(self) -> None:
        super().__init__('right_cone_local_planner')

        # Parameters
        self.declare_parameter('input_topic', '/cone/lidar/ukf')
        self.declare_parameter('path_out_topic', '/local_planned_path')
        self.declare_parameter('frame_id', 'base_link')
        # os_sensor frame: +x forward, +y left. Right cones lie at negative y.
        # Use only cones whose y ∈ [-3.0, 0.0] to suppress outliers.
        self.declare_parameter('right_y_min', -3.0)
        self.declare_parameter('right_y_max',  0.0)
        self.declare_parameter('x_min', -0.3)             # drop points well behind vehicle
        self.declare_parameter('half_width', 2.5)         # offset from right wall → center
        self.declare_parameter('ds', 0.6)                 # output spacing
        self.declare_parameter('length_m', 10.0)          # output length forward from (0,0)

        self.input_topic: str = self.get_parameter('input_topic').value
        self.path_out_topic: str = self.get_parameter('path_out_topic').value
        self.frame_id: str = self.get_parameter('frame_id').value
        self.right_y_min: float = float(self.get_parameter('right_y_min').value)
        self.right_y_max: float = float(self.get_parameter('right_y_max').value)
        self.x_min: float = float(self.get_parameter('x_min').value)
        self.half_width: float = float(self.get_parameter('half_width').value)
        self.ds: float = float(self.get_parameter('ds').value)
        self.length_m: float = float(self.get_parameter('length_m').value)

        # QoS
        qos_sub = QoSProfile(depth=1)
        qos_sub.reliability = ReliabilityPolicy.BEST_EFFORT
        qos_sub.history = HistoryPolicy.KEEP_LAST
        qos_sub.durability = DurabilityPolicy.VOLATILE

        qos_path = QoSProfile(depth=1)
        qos_path.reliability = ReliabilityPolicy.RELIABLE
        qos_path.durability = DurabilityPolicy.VOLATILE

        # IO
        self.sub = self.create_subscription(
            TrackedConeArray, self.input_topic, self.cb_cones, qos_sub
        )
        self.pub_path = self.create_publisher(Path, self.path_out_topic, qos_path)

        self.get_logger().info(
            f"RightConeLocalPlanner: {self.input_topic} → {self.path_out_topic} (frame={self.frame_id})"
        )

        self._last_path: Optional[Path] = None

    # ---------- callbacks ----------
    def cb_cones(self, msg: TrackedConeArray) -> None:
        try:
            path = self.process(msg)
            if path is not None:
                self.pub_path.publish(path)
                self._last_path = path
        except Exception as e:
            self.get_logger().error(f"[process] exception: {e}")

    # ---------- core ----------
    @staticmethod
    def _is_finite_xy(x: float, y: float) -> bool:
        return math.isfinite(x) and math.isfinite(y)

    @staticmethod
    def _unit(dx: float, dy: float) -> Tuple[float, float]:
        n = math.hypot(dx, dy)
        if n <= 1e-9:
            return (1.0, 0.0)
        return (dx / n, dy / n)

    @staticmethod
    def _resample_polyline_eqds(poly: List[Tuple[float, float]], ds: float) -> List[Tuple[float, float]]:
        if not poly:
            return []
        if len(poly) == 1:
            return poly[:]
        # cumulative arclength
        P = np.asarray(poly, dtype=np.float32)
        d = np.linalg.norm(P[1:] - P[:-1], axis=1)
        s = np.concatenate([[0.0], np.cumsum(d)])
        total = float(s[-1])
        if total <= 1e-6:
            return [tuple(map(float, P[0]))]
        n = max(1, int(total / max(1e-6, ds)))
        st = np.linspace(0.0, total, n + 1, dtype=np.float32)
        # segment-wise linear interpolation
        out = []
        j = 0
        for sv in st:
            while j + 1 < len(s) and s[j + 1] < sv:
                j += 1
            if j + 1 >= len(s):
                out.append((float(P[-1, 0]), float(P[-1, 1])))
                continue
            sj, sj1 = s[j], s[j + 1]
            if sj1 - sj <= 1e-9:
                out.append((float(P[j, 0]), float(P[j, 1])))
                continue
            t = float((sv - sj) / (sj1 - sj))
            x = float(P[j, 0] + t * (P[j + 1, 0] - P[j, 0]))
            y = float(P[j, 1] + t * (P[j + 1, 1] - P[j, 1]))
            out.append((x, y))
        return out

    def process(self, msg: TrackedConeArray) -> Optional[Path]:
        # 1) collect right cones in front area
        right_pts: List[Tuple[float, float]] = []
        for c in msg.cones:
            x = float(c.position.x)
            y = float(c.position.y)
            if not self._is_finite_xy(x, y):
                continue
            # Keep only right-side cones with y in [-3.0, 0.0]
            if (y >= self.right_y_min) and (y <= self.right_y_max) and (x >= self.x_min):
                right_pts.append((x, y))

        if len(right_pts) == 0:
            return self._publish_straight_fallback()

        # 2) sort by x (forward)
        right_pts.sort(key=lambda p: (p[0], p[1]))

        # 3) build an offset polyline (centerline) by shifting left of the tangent
        center_poly: List[Tuple[float, float]] = []
        n = len(right_pts)
        for i, (x, y) in enumerate(right_pts):
            if n == 1:
                # assume forward along +x
                tx, ty = 1.0, 0.0
            elif 0 < i < n - 1:
                # central difference tangent → smoother normals (linear interpolation basis)
                px, py = right_pts[i - 1]
                nx, ny = right_pts[i + 1]
                tx, ty = self._unit(nx - px, ny - py)
            elif i == 0:
                nx, ny = right_pts[i + 1]
                tx, ty = self._unit(nx - x, ny - y)
            else:
                px, py = right_pts[i - 1]
                tx, ty = self._unit(x - px, y - py)

            # left-normal of (tx,ty)
            nx, ny = -ty, tx
            cx = x + nx * self.half_width
            cy = y + ny * self.half_width
            center_poly.append((cx, cy))

        # 4) resample centerline and anchor to (0,0) forward
        resampled = self._resample_polyline_eqds(center_poly, max(0.2, self.ds))
        if not resampled:
            return self._publish_straight_fallback()

        # choose anchor index near (0,0) but not behind too much
        anchor = np.array([0.0, 0.0], dtype=np.float32)
        P = np.asarray(resampled, dtype=np.float32)
        # penalize points behind x < x_min a bit
        mask = P[:, 0] >= self.x_min
        if mask.any():
            Q = P[mask]
            d2 = np.sum((Q - anchor) ** 2, axis=1)
            j = int(np.argmin(d2))
            # map back to original index
            idxs = np.nonzero(mask)[0]
            j = int(idxs[j])
        else:
            d2 = np.sum((P - anchor) ** 2, axis=1)
            j = int(np.argmin(d2))

        # assemble anchored forward path of fixed length
        need = max(1, int(self.length_m / max(1e-6, self.ds)))
        out_pts: List[Tuple[float, float]] = []
        out_pts.append((0.0, 0.0))

        # add forward samples
        k = j
        while len(out_pts) < need + 1 and k + 1 < len(resampled):
            out_pts.append((float(resampled[k + 1][0]), float(resampled[k + 1][1])))
            k += 1

        # if not enough, extend straight along last tangent
        while len(out_pts) < need + 1:
            if len(out_pts) >= 2:
                dx = out_pts[-1][0] - out_pts[-2][0]
                dy = out_pts[-1][1] - out_pts[-2][1]
                ux, uy = self._unit(dx, dy)
            else:
                ux, uy = 1.0, 0.0
            lastx, lasty = out_pts[-1]
            out_pts.append((lastx + ux * self.ds, lasty + uy * self.ds))

        # 5) publish Path (forced base_link frame)
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        poses: List[PoseStamped] = []
        stamp = path.header.stamp
        for (x, y) in out_pts:
            ps = PoseStamped()
            ps.header.frame_id = self.frame_id
            ps.header.stamp = stamp
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            poses.append(ps)
        path.poses = poses
        return path

    def _publish_straight_fallback(self) -> Path:
        # Path straight ahead when no cones available
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        poses: List[PoseStamped] = []
        stamp = path.header.stamp
        n = max(1, int(self.length_m / max(1e-6, self.ds)))
        for i in range(n + 1):
            ps = PoseStamped()
            ps.header.frame_id = self.frame_id
            ps.header.stamp = stamp
            ps.pose.position.x = float(i * self.ds)
            ps.pose.position.y = 0.0
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            poses.append(ps)
        path.poses = poses
        return path


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RightConeLocalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
