#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
reference_path_planning.py  (anchored & forward-stable)
─────────────────────────────────────────────────────────────
• 좌/우 콘 → Delaunay 내부선 중점
   → (중복제거 + PCA 1D 정렬 + 갭 브리징)
   → chord-length B-spline(splprep) + 약한 스무딩
   → base_link (0,0) 앵커에서 시작하는 고정 길이 경로 생성
• /local_planned_path(nav_msgs/Path) + 디버그 마커 퍼블리시
• /desired_speed_profile 길이 불일치 허용 반영
(2025-09-02  - base_link 앵커/전방성/고정 길이/단측 fallback 추가)
"""

from typing import List, Tuple, Optional
import math
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
from scipy.interpolate import splprep, splev  # chord-length spline

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
        self.pub_path   = self.create_publisher(Path, "/local_planned_path", 10)
        self.pub_lines  = self.create_publisher(Marker, "/delaunay_internal_lines", 10)
        self.pub_midpts = self.create_publisher(Marker, "/delaunay_internal_midpoints", 10)
        self.pub_wps    = self.create_publisher(MarkerArray, "/final_waypoints", 10)
        self.pub_speed  = self.create_publisher(MarkerArray, "/waypoint_speed_bars", 10)

        # ── Sub -------------------------------------------------
        self.create_subscription(Marker, "/left_cone_marker",  self.cb_left,  10)
        self.create_subscription(Marker, "/right_cone_marker", self.cb_right, 10)
        self.create_subscription(Float32MultiArray, "/desired_speed_profile",
                                 self.cb_speed_profile, 10)

        # ── TF --------------------------------------------------
        self.sensor_frame = "base_link"  # 출력 프레임
        self.ref_frame    = "map"        # 입력 콘 프레임(마커가 map이면 map→base_link로 변환)
        self.tf_buf = tf2_ros.Buffer(cache_time=Duration(seconds=2))
        self.tf_lst = tf2_ros.TransformListener(self.tf_buf, self)

        # ── 상태 ------------------------------------------------
        self.left_ref:  List[Tuple[float, float]] = []
        self.right_ref: List[Tuple[float, float]] = []
        self.speed_profile: List[float] = []
        self.latest_wps: List[Waypoint] = []
        self.prev_wp_n = 0
        self._tf_warned = False
        self.last_valid_stamp = rclpy.time.Time()

        # ── 파라미터 -------------------------------------------
        # 최종 출력 간격/길이
        self.ds_out               = 0.20     # 최종 Waypoint 간격 [m]
        self.fixed_length_m       = 10.0     # (0,0)에서 앞으로 고정 길이 [m]
        self.forward_x_margin     = 0.3      # x < -margin 은 버림(원점 뒤쪽 제거)

        # 기본 속도/마커
        self.default_speed  = 2.0
        self.scale_wp       = 0.15
        self.min_bar_height = 0.01
        self.marker_life    = MsgDuration()  # 0 = forever

        # Delaunay 브릿지(좌우 연결 에지) 거리 필터
        self.min_lane_width = 1.0
        self.max_lane_width = 6.0

        # 중점 후처리
        self.voxel_dedup_m  = 0.05  # 5cm 격자 중복 제거
        self.gap_bridge_th  = 1.2   # 중점 간 간격이 이보다 크면 브리징
        self.bridge_step    = 0.6   # 브리징 보강 간격

        # 스플라인 스무딩(길이 비례, 0~0.05 권장)
        self.smooth_coef    = 0.02

        # 단측 fallback (폭 추정)
        self.half_width_est = 2.5   # 없으면 기본값
        self.half_width_min = 1.0
        self.half_width_max = 4.0

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
        self.speed_profile = list(msg.data)
        if not self.latest_wps or not self.speed_profile:
            return
        n_common = min(len(self.latest_wps), len(self.speed_profile))
        for i in range(n_common):
            self.latest_wps[i].speed = self.speed_profile[i]
        for i in range(n_common, len(self.latest_wps)):
            self.latest_wps[i].speed = self.default_speed
        self.publish_waypoints(self.latest_wps, self.last_valid_stamp.to_msg())

    # ==========================================================
    # ── Delaunay 내부선 중점 추출 ─────────────────────────
    def delaunay_midpoints(
        self,
        left_s: List[Tuple[float, float]],
        right_s: List[Tuple[float, float]],
    ) -> np.ndarray:
        """
        base_link 좌표계의 좌·우 콘 → Delaunay 내부선(좌↔우만) 중점 집합 (N×2)
        """
        pts = np.array(left_s + right_s, float)
        if len(pts) < 3:
            return np.empty((0, 2))
        tri = Delaunay(pts)
        lN = len(left_s)
        mids = []
        for s in tri.simplices:
            for a, b in ((s[0], s[1]), (s[1], s[2]), (s[2], s[0])):
                if (a < lN) != (b < lN):  # 좌↔우만
                    dist = float(np.linalg.norm(pts[a] - pts[b]))
                    if dist < self.min_lane_width or dist > self.max_lane_width:
                        continue
                    mids.append([(pts[a, 0] + pts[b, 0]) * 0.5,
                                 (pts[a, 1] + pts[b, 1]) * 0.5])
        if len(mids) < 3:
            return np.empty((0, 2))
        return np.array(mids, float)

    # ── 중점 후처리: 중복 제거 + PCA 1D 정렬 + 갭 브리징 ────
    def _dedup_points(self, pts: np.ndarray, voxel: float = 0.05) -> np.ndarray:
        if len(pts) == 0:
            return pts
        grid = np.round(pts / voxel).astype(np.int32)
        _, idx = np.unique(grid, axis=0, return_index=True)
        return pts[np.sort(idx)]

    def _order_by_pca(self, pts: np.ndarray) -> np.ndarray:
        if len(pts) < 3:
            return pts
        c = pts.mean(axis=0)
        X = pts - c
        _, _, vh = np.linalg.svd(X, full_matrices=False)
        axis = vh[0]                  # 주성분 1축
        u1 = X @ axis                 # 1D 투영
        order = np.argsort(u1)
        out = pts[order]
        # 전방성 보조: x가 감소 방향이면 뒤집기
        if out[0, 0] > out[-1, 0]:
            out = out[::-1]
        return out

    def _bridge_gaps(self, pts: np.ndarray, gap_th: float, step: float) -> np.ndarray:
        """연속 점 간 거리가 gap_th 초과면 선형으로 중간 점 삽입"""
        if len(pts) < 2:
            return pts
        out = [pts[0]]
        for i in range(len(pts) - 1):
            p0 = pts[i]; p1 = pts[i+1]
            d = float(np.linalg.norm(p1 - p0))
            if d > gap_th:
                n = max(1, int(d / max(1e-6, step)))
                for k in range(1, n):
                    s = k / float(n)
                    out.append((1.0 - s) * p0 + s * p1)
            out.append(p1)
        return np.array(out, float)

    # ── chord-length B-spline(+약한 스무딩) → 치밀 샘플 ────
    def _smooth_centerline(self, mids: np.ndarray) -> np.ndarray:
        if len(mids) < 3:
            return mids.copy()
        seg = np.linalg.norm(np.diff(mids, axis=0), axis=1)
        s = np.concatenate([[0.0], np.cumsum(seg)])
        L = float(s[-1])
        if L < 1e-3:
            return mids.copy()
        s_abs = (max(0.0, self.smooth_coef) * L) ** 2
        try:
            tck, _ = splprep([mids[:, 0], mids[:, 1]], u=s, s=s_abs, k=3)
            ss = np.linspace(0.0, L, num=max(100, int(L / 0.05)))
            x, y = splev(ss, tck)
            return np.column_stack([x, y]).astype(float)
        except Exception as e:
            self.get_logger().warn(f"splprep failed: {e}. Fallback to linear.")
            ss = np.linspace(0.0, L, num=max(50, int(L / 0.1)))
            x = np.interp(ss, s, mids[:, 0])
            y = np.interp(ss, s, mids[:, 1])
            return np.column_stack([x, y]).astype(float)

    # ── 전방성/원점 뒤쪽 제거 + 0-length 제거 ───────────────
    def _forward_sanitize(self, P: np.ndarray) -> np.ndarray:
        if len(P) == 0:
            return P
        # 원점 뒤쪽 제거
        P = P[P[:, 0] >= -float(self.forward_x_margin)]
        if len(P) < 2:
            return P
        # 0-length 제거
        d = np.linalg.norm(np.diff(P, axis=0), axis=1)
        mask = np.r_[True, d > 1e-4]
        return P[mask]

    # ── (0,0) 앵커에서 시작하는 고정 길이 등간격 경로 ──────
    def _anchor_fixed_length(self, P: np.ndarray) -> np.ndarray:
        """
        입력 polyline P(N×2)을 기준으로,
        (0,0)과 P의 최근접점 Q*를 찾아 (0,0)→Q* 연결 후
        앞으로 fixed_length_m 만큼 ds_out 간격으로 샘플링.
        부족하면 최종 접선방향으로 연장.
        """
        if len(P) == 0:
            return np.zeros((0, 2), float)

        O = np.array([0.0, 0.0], float)

        # 누적 s
        seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
        s = np.concatenate([[0.0], np.cumsum(seg)])
        L = float(s[-1])

        # O의 P에 대한 최근접점 파라미터 (세그먼트 탐색)
        best = (1e18, 0, 0.0, P[0])  # (d2, idx, t, point)
        for i in range(len(P) - 1):
            A = P[i]; B = P[i+1]
            v = B - A
            v2 = float(v.dot(v))
            if v2 < 1e-12:
                d2 = float((O - A).dot(O - A))
                if d2 < best[0]:
                    best = (d2, i, 0.0, A)
                continue
            t = float(np.clip(((O - A).dot(v)) / v2, 0.0, 1.0))
            Q = A + t * v
            d2 = float((O - Q).dot(O - Q))
            if d2 < best[0]:
                best = (d2, i, t, Q)

        i0 = best[1]; t0 = best[2]; Q = best[3]
        sQ = s[i0] + t0 * (s[i0+1] - s[i0])

        # (1) O→Q 선형 보간(등간격 ds_out)
        out_pts = []
        dOQ = float(np.linalg.norm(Q - O))
        if dOQ > 1e-6:
            n = max(1, int(dOQ / max(1e-6, self.ds_out)))
            for k in range(n):
                u = k / float(n)
                out_pts.append((1.0 - u) * O + u * Q)
        else:
            out_pts.append(O.copy())

        # (2) Q 이후 P 따라 진행하며 등간격 샘플
        # s_target: sQ, sQ+ds, ..., sQ+fixed_length
        need_len = float(self.fixed_length_m)
        ds = float(self.ds_out)
        s_targets = np.arange(sQ, sQ + need_len + 0.5 * ds, ds)

        # P에서 (선분 보간) 위치 계산
        # 인덱스 i를 추적하며 빠르게 보간
        i = i0
        for st in s_targets:
            st = float(np.clip(st, 0.0, L))
            # i such that s[i] <= st <= s[i+1]
            while i < len(s) - 2 and s[i+1] < st:
                i += 1
            if i >= len(s) - 1:
                i = len(s) - 2
            sA, sB = s[i], s[i+1]
            denom = max(1e-9, sB - sA)
            tau = (st - sA) / denom
            A = P[i]; B = P[i+1]
            out_pts.append((1.0 - tau) * A + tau * B)

        out = np.vstack(out_pts)

        # (3) P가 짧아 목표 길이에 못 미치면 마지막 접선으로 연장
        if len(out) >= 2:
            remain = need_len - float(np.linalg.norm(out[-1] - out[0]))
            if remain > 0.25 * ds:
                tdir = out[-1] - out[-2]
                nrm = float(np.linalg.norm(tdir))
                if nrm < 1e-9:
                    tdir = np.array([1.0, 0.0], float)
                    nrm = 1.0
                tdir = tdir / nrm
                k = int(remain / ds)
                base = out[-1]
                for j in range(1, k + 1):
                    out = np.vstack([out, base + j * ds * tdir])

        # (4) 등간격 재정렬(숫자오차 정리)
        if len(out) >= 2:
            seg = np.linalg.norm(np.diff(out, axis=0), axis=1)
            s = np.concatenate([[0.0], np.cumsum(seg)])
            L = float(s[-1])
            sample_s = np.arange(0.0, L + 0.5 * ds, ds)
            Xs = np.interp(sample_s, s, out[:, 0])
            Ys = np.interp(sample_s, s, out[:, 1])
            out = np.column_stack([Xs, Ys])

        return out

    # ── 단측 데이터만 있을 때 센터라인 합성 ───────────────────
    def _center_from_single_side(self, side_pts: List[Tuple[float, float]], side: str) -> np.ndarray:
        """한쪽(좌/우)만 있을 때: 방향 접선의 법선 방향으로 half_width_est 만큼 오프셋하여 center 합성"""
        if len(side_pts) < 2:
            return np.zeros((0, 2), float)
        P = np.array(sorted(side_pts, key=lambda p: p[0]), float)
        out = []
        for i in range(len(P) - 1):
            t = P[i+1] - P[i]
            nrm = float(np.linalg.norm(t))
            if nrm < 1e-9:
                continue
            t = t / nrm
            n = np.array([-t[1], t[0]], float)  # 좌측 법선
            sign = -1.0 if side == "left" else +1.0
            c = P[i] + sign * self.half_width_est * n
            out.append(c)
        return np.array(out, float)

    def _update_half_width_est(self, left_s: List[Tuple[float, float]], right_s: List[Tuple[float, float]]):
        """좌우가 둘 다 있을 때 폭 추정(중앙값)"""
        if len(left_s) < 2 or len(right_s) < 2:
            return
        L = np.array(sorted(left_s, key=lambda p: p[0]), float)
        R = np.array(sorted(right_s, key=lambda p: p[0]), float)
        # 간단: 왼쪽 각 점에 대해 오른쪽 최근접 거리
        widths = []
        for p in L:
            d = np.min(np.linalg.norm(R - p, axis=1))
            widths.append(d)
        if len(widths) >= 3:
            w = float(np.median(widths))
            hw = max(self.half_width_min, min(self.half_width_max, 0.5 * w))
            self.half_width_est = 0.8 * self.half_width_est + 0.2 * hw

    # ==========================================================
    # 경로 계획
    def plan_path(self):
        # TF(map → base_link)
        try:
            tf_s_r = self.tf_buf.lookup_transform(
                self.sensor_frame, self.ref_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.2))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            if not self._tf_warned:
                self.get_logger().warning("TF(sensor←map) unavailable – waiting…")
                self._tf_warned = True
            return

        self._tf_warned = False
        stamp_time = tf_s_r.header.stamp
        self.last_valid_stamp = rclpy.time.Time.from_msg(stamp_time)

        R_sr, t_sr = self.tf_to_mat(tf_s_r)

        # base_link 기준 좌/우 포인트
        left_s  = [self.transform_pt(R_sr, t_sr, p) for p in self.left_ref ]
        right_s = [self.transform_pt(R_sr, t_sr, p) for p in self.right_ref]

        # 폭 추정 업데이트(가능할 때만)
        self._update_half_width_est(left_s, right_s)

        # ── 센터 후보 생성: Delaunay midpoints 또는 단측 fallback ──
        mids = np.empty((0, 2))
        if len(left_s) >= 3 and len(right_s) >= 3:
            mids = self.delaunay_midpoints(
                sorted(left_s,  key=lambda p: p[0]),
                sorted(right_s, key=lambda p: p[0]),
            )
        else:
            # 단측만 있을 때 합성
            if len(left_s) >= 2 and len(right_s) < 2:
                mids = self._center_from_single_side(left_s, side="left")
            elif len(right_s) >= 2 and len(left_s) < 2:
                mids = self._center_from_single_side(right_s, side="right")
            else:
                # 데이터 부족
                return

        if mids.size == 0:
            return

        # 후처리: dedup → PCA 정렬 → 갭 브리징 → 스무딩
        mids = self._dedup_points(mids, voxel=self.voxel_dedup_m)
        if len(mids) < 2:
            return
        mids = self._order_by_pca(mids)
        mids = self._bridge_gaps(mids, gap_th=self.gap_bridge_th, step=self.bridge_step)
        center_dense = self._smooth_centerline(mids)

        # 전방성/뒤쪽 제거 + 0-length 제거
        center_dense = self._forward_sanitize(center_dense)
        if len(center_dense) < 2:
            return

        # (0,0) 앵커에서 시작하는 고정 길이 등간격 경로 생성
        anchored = self._anchor_fixed_length(center_dense)
        if len(anchored) < 2:
            return

        # Waypoints 생성(등간격 보장)
        wps = [Waypoint(float(x), float(y), self.default_speed) for x, y in anchored]

        # 속도 프로파일 반영
        if self.speed_profile:
            n_common = min(len(wps), len(self.speed_profile))
            for i in range(n_common):
                wps[i].speed = float(self.speed_profile[i])

        self.latest_wps = wps

        # 퍼블리시
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

    # ----------------------------------------------------------
    # 퍼블리시
    def publish_path(self, wps: List[Waypoint], stamp_time):
        msg = Path()
        msg.header.stamp = stamp_time
        msg.header.frame_id = self.sensor_frame
        msg.poses = []
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

        # 삭제 명령
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
        lN = len(left_s)

        header = Header(stamp=stamp_time, frame_id=self.sensor_frame)

        lines = Marker(header=header, ns="bridge_lines", id=0,
                       type=Marker.LINE_LIST, action=Marker.ADD)
        lines.scale.x = 0.02
        lines.color.r = lines.color.g = lines.color.b = lines.color.a = 1.0
        lines.lifetime = self.marker_life

        mids = Marker(header=header, ns="bridge_midpts", id=0,
                      type=Marker.SPHERE_LIST, action=Marker.ADD)
        mids.scale.x = mids.scale.y = mids.scale.z = 0.30
        mids.color.r, mids.color.g, mids.color.b, mids.color.a = 0.0, 1.0, 1.0, 1.0
        mids.lifetime = self.marker_life

        for s in tri.simplices:
            for a, b in ((s[0], s[1]), (s[1], s[2]), (s[2], s[0])):
                if (a < lN) != (b < lN):
                    dist = float(np.linalg.norm(pts[a] - pts[b]))
                    if dist < self.min_lane_width or dist > self.max_lane_width:
                        continue
                    pA = Point(x=float(pts[a, 0]), y=float(pts[a, 1]))
                    pB = Point(x=float(pts[b, 0]), y=float(pts[b, 1]))
                    lines.points.extend([pA, pB])
                    mids.points.append(Point(x=(pA.x + pB.x) / 2, y=(pA.y + pB.y) / 2))

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
