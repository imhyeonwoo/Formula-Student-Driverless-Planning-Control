#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Compact ConesColorSubscriber (+ Robust Midpoints, Vectorized Constraints, Heading-Gated Center Path)
- 좌/우 콘 레이블링 기반 벽 라인(좌/우) 생성
- 로버스트 중점 생성(헝가리안 + 프로젝션 + 리샘플 정렬 + 단측 오프셋)
- 중앙 경로: 제약 항 (Boundary, Directional, Smoothness) + 헤딩 변화 하드게이트 반영 Greedy 탐색
- RViz: 좌/우 벽, 중점(초록 점), 중앙 경로(초록 선) 퍼블리시
- 추가: (0,0) 앵커에서 시작하는 고정 길이 경로로 변환하여 퍼블리시
- 추가: 시작점 색상 기반 "색상 우선 탐색" (노랑/파랑 우선 → 없으면 완화)
- 추가: nav_msgs/Path로 중심 경로 퍼블리시(/local_planned_path) → PurePursuit 구독 가능
"""

import math
from typing import List, Tuple, Dict, Optional

import numpy as np
from scipy.optimize import linear_sum_assignment

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from builtin_interfaces.msg import Duration
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path

from custom_interface.msg import TrackedConeArray


# ──────────────────────────────────────────────────────────────────────────────
# 기본 파라미터
# ──────────────────────────────────────────────────────────────────────────────
INPUT_TOPIC = '/cone/lidar/ukf'
OUTPUT_TOPIC = '/cones_marker_array'
PATH_OUTPUT_TOPIC = '/local_planned_path'   # ← PurePursuit 가 구독할 토픽
USE_HEADER_FRAME = True
FALLBACK_FRAME = 'base_link'
FORCE_FALLBACK_FRAME = True

PROCESS_HZ = 10.0
REPUBLISH_IDLE_SEC = 0.5

# Grid 그래프 파라미터 (벽/중점 공통 기본)
EDGE_LEN_TH = 3.5
MAX_NEIGHBORS_PER_NODE = 6
MAX_STEPS = 1000
MIN_DIST_TH = 1.0

# 시작 영역 (x는 전방, y는 좌(+)/우(-))
START_X_RANGE = (-0.5, 5.0)
START_Y_LEFT = (0.1, 5.0)
START_Y_RIGHT = (-5.0, -0.1)
START_Y_CENTER = (-2.0, 2.0)

# 각도 티어(벽 라인용)
PARENT_DEG_TIERS = [65.0, 95.0]
GRAND_DEG_TIERS  = [85.0, 90.0]
GLOBAL_DEG_TIERS = [100.0]
MAX_RELAX_LEVELS = 2
X_BACKTRACK_MARGIN = 0.3
PREFER_GAIN = 0.25

# 색상 키/표시 색
LEFT_KEYS  = {'yellow', 'yellowcone'}
RIGHT_KEYS = {'blue', 'bluecone'}
RED_KEYS   = {'red', 'redcone'}
LEFT_COLOR   = (1.0, 1.0, 0.0, 1.0)
RIGHT_COLOR  = (0.0, 0.3, 1.0, 1.0)
RED_COLOR    = (1.0, 0.0, 0.0, 1.0)
MID_DOT_COLOR= (0.0, 1.0, 0.0, 0.9)
CENTER_COLOR = (0.0, 0.85, 0.0, 1.0)

# 마커
MARKER_SCALE = 0.45
LIFETIME_SEC = 0.5
MIN_PATH_LEN = 3

DEDUP_EPS = 1e-3

# ── 제약/최적화 하이퍼파라미터 (중앙 경로용) ─────────────────────────────────
BD_MARGIN = 1.5
W_BD = 3.0
W_DIR = 1.0
W_SMOOTH = 0.5

# 좌/우 페어링 ↔ 중점 생성 임계
MID_MAX_PAIR_DIST = 6.5
MID_DOT_SCALE = 0.18

# [단측/헝가리안 실패 대비]
DEFAULT_HALF_WIDTH = 2.8
HALF_WIDTH_MINMAX = (1.0, 4.0)
RESAMPLE_STEP = 1.2
PAIR_MIN_KEEP = 3

# ── 중앙 경로 간격 균질화/브릿지 파라미터 ─────────────────────────────
CENTER_STEP = 0.6
CENTER_EDGE_LEN_TH = 3.0
GAP_BRIDGE_TH = 1.2
GAP_SPLINE_TH = 3.5
MAX_BRIDGE_POINTS = 80

# 중앙 경로 역전/후퇴 방지
CENTER_BACKTRACK_MARGIN = 0.3
FWD_COS_TH = 0.0

# ── 헤딩 변화 하드 게이트 ─────────────────────────────────────
GLOBAL_MAX_HEADING_DELTA_DEG = 60.0
GLOBAL_MAX_HEADING_RELAX_DEG = 95.0
LOCAL_MAX_HEADING_DELTA_DEG  = 80.0

# 업데이트 EMA
REF_DIR_EMA_ALPHA = 0.85

# ── (0,0) 고정 길이 경로 출력 ──────────────────────────────────────
ANCHOR_PT = (0.0, 0.0)
PATH_LENGTH_M = 10.0


def _normalize_color_key(s: str) -> str:
    if not s:
        return 'unknown'
    return ''.join(s.lower().replace('_', '').split())


def _euclid(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _dot(u, v) -> float:
    return u[0]*v[0] + u[1]*v[1]


def _unit(vx: float, vy: float) -> Tuple[float, float]:
    n = math.hypot(vx, vy)
    return (vx/n, vy/n) if n > 1e-9 else (0.0, 0.0)


def _rotate90(vx: float, vy: float) -> Tuple[float, float]:
    return (-vy, vx)


class ConesColorSubscriber(Node):
    def __init__(self):
        super().__init__('cones_color_subscriber')

        # 토픽 파라미터
        self.declare_parameter('input_topic', INPUT_TOPIC)
        self.declare_parameter('output_topic', OUTPUT_TOPIC)
        self.declare_parameter('path_out_topic', PATH_OUTPUT_TOPIC)  # ← Path 토픽 파라미터
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.path_out_topic = self.get_parameter('path_out_topic').value

        # 퍼블리셔 (MarkerArray)
        qos_pub = QoSProfile(depth=1)
        qos_pub.reliability = ReliabilityPolicy.RELIABLE
        qos_pub.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(MarkerArray, self.output_topic, qos_pub)

        # 퍼블리셔 (Path) — VOLATILE 로 일반 서브스크립션과 호환
        qos_path = QoSProfile(depth=1)
        qos_path.reliability = ReliabilityPolicy.RELIABLE
        qos_path.durability = DurabilityPolicy.VOLATILE
        self.pub_path = self.create_publisher(Path, self.path_out_topic, qos_path)

        # 서브스크립션
        qos_sub = QoSProfile(depth=1)
        qos_sub.reliability = ReliabilityPolicy.BEST_EFFORT
        qos_sub.history = HistoryPolicy.KEEP_LAST
        qos_sub.durability = DurabilityPolicy.VOLATILE
        self.sub = self.create_subscription(
            TrackedConeArray, self.input_topic, self._cb, qos_sub)

        self.get_logger().info(
            f"Subscribed: {self.input_topic} → Publishing markers: {self.output_topic} | path: {self.path_out_topic}")

        self._latest_msg: Optional[TrackedConeArray] = None
        self._last_msg_time = self.get_clock().now()
        self._last_arr: Optional[MarkerArray] = None

        self.create_timer(1.0 / max(0.5, PROCESS_HZ), self._on_timer)

        # 각도 티어 미리 계산
        self.parent_cos = [math.cos(math.radians(d)) for d in PARENT_DEG_TIERS]
        self.grand_cos  = [math.cos(math.radians(d)) for d in GRAND_DEG_TIERS]
        self.global_cos = [math.cos(math.radians(d)) for d in GLOBAL_DEG_TIERS]
        self.life = Duration(sec=int(LIFETIME_SEC),
                             nanosec=int((LIFETIME_SEC - int(LIFETIME_SEC))*1e9))

        # 헤딩 게이트 코사인
        self.cos_global_delta = math.cos(math.radians(GLOBAL_MAX_HEADING_DELTA_DEG))
        self.cos_global_relax = math.cos(math.radians(GLOBAL_MAX_HEADING_RELAX_DEG))
        self.cos_local_delta  = math.cos(math.radians(LOCAL_MAX_HEADING_DELTA_DEG))

        # 폭 추정 / 이전 경로/헤딩 캐시
        self._half_width_est = DEFAULT_HALF_WIDTH
        self._prev_center_poly: List[Tuple[float, float]] = []
        self._prev_ref_dir: Optional[Tuple[float, float]] = None

    # 콜백
    def _cb(self, msg: TrackedConeArray):
        self._latest_msg = msg
        self._last_msg_time = self.get_clock().now()

    def _on_timer(self):
        now = self.get_clock().now()
        if self._latest_msg is None:
            if self._last_arr and (now - self._last_msg_time).nanoseconds/1e9 >= REPUBLISH_IDLE_SEC:
                ts = now.to_msg()
                for m in self._last_arr.markers:
                    m.header.stamp = ts
                self.pub.publish(self._last_arr)
            return
        try:
            arr, path_msg = self._process(self._latest_msg)
            self._last_arr = arr
            self.pub.publish(arr)
            if path_msg is not None:
                self.pub_path.publish(path_msg)
        except Exception as e:
            self.get_logger().error(f"[Process] exception: {e}")

    # ────────────────────────────────────────────────────────────────────────
    # 유틸 (생략 없이 기존 그대로)
    def _sort_midpoints_by_progress(self, mids, left_poly, right_poly):
        if not mids:
            return []
        if len(mids) <= 2:
            return sorted(mids, key=lambda p: p[0])
        d = self._compose_lane_dir(mids[0], left_poly, right_poly)
        ax = np.array(d, dtype=np.float32)
        if float(ax.dot(ax)) < 1e-6:
            P = np.array(mids, dtype=np.float32)
            c = P.mean(axis=0)
            _, _, Vt = np.linalg.svd(P - c, full_matrices=False)
            ax = Vt[0]
            if ax[0] < 0:
                ax = -ax
        ref = np.array(mids[0], dtype=np.float32)
        return sorted(mids, key=lambda p: float(np.dot(np.array(p, dtype=np.float32) - ref, ax)))

    def _sanitize(self, pts, zs, keys):
        mask = []
        uniq = set()
        for i, (p, z, k) in enumerate(zip(pts, zs, keys)):
            x, y = p
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            gx, gy = round(x / DEDUP_EPS), round(y / DEDUP_EPS)
            if (gx, gy) in uniq:
                continue
            uniq.add((gx, gy))
            mask.append(i)
        return mask

    def _build_graph_grid(self, pts: List[Tuple[float, float]]):
        r = max(1e-6, EDGE_LEN_TH)
        inv = 1.0 / r
        cell: Dict[Tuple[int, int], List[int]] = {}
        for i, (x, y) in enumerate(pts):
            cx, cy = int(math.floor(x * inv)), int(math.floor(y * inv))
            cell.setdefault((cx, cy), []).append(i)
        graph: Dict[int, List[int]] = {i: [] for i in range(len(pts))}
        for (cx, cy), idxs in cell.items():
            for i in idxs:
                x, y = pts[i]
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        n_idxs = cell.get((cx + dx, cy + dy))
                        if not n_idxs:
                            continue
                        for j in n_idxs:
                            if j <= i:
                                continue
                            if _euclid((x, y), pts[j]) <= r:
                                graph[i].append(j)
                                graph[j].append(i)
        if MAX_NEIGHBORS_PER_NODE > 0:
            for u, nbrs in graph.items():
                if len(nbrs) > MAX_NEIGHBORS_PER_NODE:
                    nbrs.sort(key=lambda v: _euclid(pts[u], pts[v]))
                    graph[u] = nbrs[:MAX_NEIGHBORS_PER_NODE]
        return graph

    def _build_graph_grid_radius(self, pts: List[Tuple[float, float]], radius: float,
                                 max_neighbors: int = MAX_NEIGHBORS_PER_NODE):
        if not pts:
            return {}
        r = max(1e-6, float(radius))
        inv = 1.0 / r
        cell: Dict[Tuple[int, int], List[int]] = {}
        for i, (x, y) in enumerate(pts):
            cx, cy = int(math.floor(x * inv)), int(math.floor(y * inv))
            cell.setdefault((cx, cy), []).append(i)
        graph: Dict[int, List[int]] = {i: [] for i in range(len(pts))}
        for (cx, cy), idxs in cell.items():
            for i in idxs:
                x, y = pts[i]
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        n_idxs = cell.get((cx + dx, cy + dy))
                        if not n_idxs:
                            continue
                        for j in n_idxs:
                            if j <= i:
                                continue
                            if _euclid((x, y), pts[j]) <= r:
                                graph[i].append(j)
                                graph[j].append(i)
        if max_neighbors > 0:
            for u, nbrs in graph.items():
                if len(nbrs) > max_neighbors:
                    nbrs.sort(key=lambda v: _euclid(pts[u], pts[v]))
                    graph[u] = nbrs[:max_neighbors]
        return graph

    def _hermite_bridge(self, p0, p1, t0, t1, step=CENTER_STEP):
        d = _euclid(p0, p1)
        n = min(MAX_BRIDGE_POINTS, max(1, int(d / max(1e-6, step))))
        m0 = (t0[0]*0.5*d, t0[1]*0.5*d)
        m1 = (t1[0]*0.5*d, t1[1]*0.5*d)
        out = []
        for k in range(1, n):
            s = k / float(n)
            h00 =  2*s**3 - 3*s**2 + 1
            h10 =      s**3 - 2*s**2 + s
            h01 = -2*s**3 + 3*s**2
            h11 =      s**3 -   s**2
            x = h00*p0[0] + h10*m0[0] + h01*p1[0] + h11*m1[0]
            y = h00*p0[1] + h10*m0[1] + h01*p1[1] + h11*m1[1]
            out.append((x, y))
        return out

    def _linear_bridge(self, p0, p1, step=CENTER_STEP):
        d = _euclid(p0, p1)
        n = min(MAX_BRIDGE_POINTS, max(1, int(d / max(1e-6, step))))
        out = []
        for k in range(1, n):
            s = k / float(n)
            out.append((p0[0] + s*(p1[0]-p0[0]),
                        p0[1] + s*(p1[1]-p0[1])))
        return out

    def _densify_midpoints(self, mids: List[Tuple[float, float]],
                           left_poly: List[Tuple[float, float]],
                           right_poly: List[Tuple[float, float]]):
        if not mids:
            return []
        out = [mids[0]]
        for i in range(len(mids)-1):
            p0, p1 = mids[i], mids[i+1]
            d = _euclid(p0, p1)
            if d > GAP_BRIDGE_TH:
                if d > GAP_SPLINE_TH:
                    t0 = self._compose_lane_dir(p0, left_poly, right_poly)
                    t1 = self._compose_lane_dir(p1, left_poly, right_poly)
                    bridge = self._hermite_bridge(p0, p1, t0, t1, step=CENTER_STEP)
                else:
                    bridge = self._linear_bridge(p0, p1, step=CENTER_STEP)
                out.extend(bridge)
            out.append(p1)
        return out

    def _tier(self, arr: List[float], i: int) -> float:
        if not arr:
            return -1.0
        return arr[i] if i < len(arr) else arr[-1]

    # 색상 우선 탐색용: 시작점 색상으로 우선 세트 결정
    def _choose_prefer_keys(self, start_idx: Optional[int], keys: List[str], default_keys: set):
        if start_idx is None:
            return default_keys
        k = keys[start_idx]
        if k in LEFT_KEYS:
            return LEFT_KEYS
        if k in RIGHT_KEYS:
            return RIGHT_KEYS
        return default_keys

    # 벽 라인 Greedy (색상 우선 → 없으면 완화)
    def _greedy_path(self, start, graph, pts, keys, prefer_keys) -> List[int]:
        if start is None:
            return []
        path = [start]
        seen = {start}
        maxx = max(p[0] for p in pts) if pts else 0.0
        steps = 0
        use_global = len(self.global_cos) > 0
        max_levels = min(MAX_RELAX_LEVELS, max(len(self.parent_cos), len(self.grand_cos), len(self.global_cos) or [1]))
        while steps < MAX_STEPS:
            steps += 1
            u = path[-1]
            parent = path[-2] if len(path) >= 2 else None
            grand  = path[-3] if len(path) >= 3 else None
            px, py = pts[u]
            p1 = None
            if parent is not None:
                dx1, dy1 = px - pts[parent][0], py - pts[parent][1]
                n1 = math.hypot(dx1, dy1)
                p1 = (dx1/n1, dy1/n1) if n1 > 0 else None
            p2 = None
            if grand is not None:
                dx2, dy2 = px - pts[grand][0], py - pts[grand][1]
                n2 = math.hypot(dx2, dy2)
                p2 = (dx2/n2, dy2/n2) if n2 > 0 else None

            best = None
            for tier in range(max_levels):
                gcos = self._tier(self.global_cos, tier)
                pcos = self._tier(self.parent_cos, tier)
                qcos = self._tier(self.grand_cos,  tier)

                for prefer_only in (True, False):
                    cand = None
                    cand_score = 1e18
                    for v in graph.get(u, []):
                        if v == parent or v in seen:
                            continue
                        if prefer_only and prefer_keys and keys[v] not in prefer_keys:
                            continue
                        vx, vy = pts[v][0] - px, pts[v][1] - py
                        dist = math.hypot(vx, vy)
                        if dist <= MIN_DIST_TH:
                            continue
                        ux, uy = vx/dist, vy/dist
                        if use_global and gcos > -1.0:
                            if ux < gcos:
                                continue
                        elif not use_global:
                            if pts[v][0] < px - X_BACKTRACK_MARGIN:
                                continue
                        if p1 and pcos > -1.0 and (_dot(p1, (ux, uy)) < pcos):
                            continue
                        if p2 and qcos > -1.0 and (_dot(p2, (ux, uy)) < qcos):
                            continue

                        score = dist
                        if not prefer_only and prefer_keys and keys[v] in prefer_keys:
                            score *= (1.0 - PREFER_GAIN)
                        score -= 1e-3 * (pts[v][0] - px)

                        if score < cand_score:
                            cand_score = score
                            cand = v
                    if cand is not None:
                        best = cand
                        break
                if best is not None:
                    break

            if best is None:
                break
            path.append(best)
            seen.add(best)
            if pts[best][0] >= maxx - 1e-6:
                break
        return path

    def _get_start_node(self, pts: List[Tuple[float, float]], y_range: Tuple[float, float]):
        try:
            cands = [(i, x) for i, (x, y) in enumerate(pts)
                     if START_X_RANGE[0] <= x < START_X_RANGE[1] and y_range[0] <= y < y_range[1]]
            return min(cands, key=lambda it: it[1])[0] if cands else None
        except Exception:
            return None

    # ────────────────────────────────────────────────────────────────────────
    # 거리/접선 유틸
    def _point_seg_distance_and_tangent(self, p: Tuple[float, float],
                                        poly: List[Tuple[float, float]]):
        if not poly:
            return float('inf'), (0.0, 0.0)
        if len(poly) == 1:
            dx, dy = p[0]-poly[0][0], p[1]-poly[0][1]
            return math.hypot(dx, dy), (0.0, 0.0)

        px, py = p
        best_d = float('inf')
        best_tan = (0.0, 0.0)
        for i in range(len(poly)-1):
            x1, y1 = poly[i]
            x2, y2 = poly[i+1]
            vx, vy = x2-x1, y2-y1
            seg_len2 = vx*vx + vy*vy
            if seg_len2 < 1e-9:
                d = math.hypot(px-x1, py-y1)
                if d < best_d:
                    best_d = d
                    best_tan = (0.0, 0.0)
                continue
            t = ((px-x1)*vx + (py-y1)*vy) / seg_len2
            t = max(0.0, min(1.0, t))
            cx, cy = (x1 + t*vx, y1 + t*vy)
            d = math.hypot(px-cx, py-cy)
            if d < best_d:
                best_d = d
                best_tan = _unit(vx, vy)
        return best_d, best_tan

    def _compose_lane_dir(self, p, left_poly, right_poly):
        dL, tanL = self._point_seg_distance_and_tangent(p, left_poly)
        dR, tanR = self._point_seg_distance_and_tangent(p, right_poly)
        vx, vy = tanL[0]+tanR[0], tanL[1]+tanR[1]
        if abs(vx) < 1e-9 and abs(vy) < 1e-9:
            if dL < float('inf'):
                vx, vy = tanL
            elif dR < float('inf'):
                vx, vy = tanR
        return _unit(vx, vy)

    # ────────────────────────────────────────────────────────────────────────
    # 리샘플/프로젝션/중점 생성 유틸
    def _resample_poly(self, poly: List[Tuple[float, float]], step=RESAMPLE_STEP):
        if not poly:
            return []
        if len(poly) == 1:
            return poly[:]
        pts = np.array(poly, dtype=np.float32)
        seg = pts[1:] - pts[:-1]
        seglen = np.linalg.norm(seg, axis=1)
        L = float(np.sum(seglen))
        if L < 1e-6:
            return poly[:]
        n = max(2, int(L / max(1e-3, step)) + 1)
        s_target = np.linspace(0.0, L, n)
        acc = np.concatenate([[0.0], np.cumsum(seglen)])
        out = []
        j = 0
        for st in s_target:
            while j < len(seglen) and acc[j+1] < st:
                j += 1
            if j >= len(seglen):
                out.append(tuple(pts[-1]))
                continue
            r = (st - acc[j]) / max(1e-9, seglen[j])
            out.append(tuple(pts[j] + r * seg[j]))
        return out

    def _nearest_point_on_poly(self, p: Tuple[float, float], poly: List[Tuple[float, float]]):
        if not poly:
            return (float('inf'), (p[0], p[1]))
        px, py = p
        best_d = float('inf')
        best_c = (px, py)
        for i in range(len(poly)-1):
            x1, y1 = poly[i]
            x2, y2 = poly[i+1]
            vx, vy = x2-x1, y2-y1
            seg_len2 = vx*vx + vy*vy
            if seg_len2 < 1e-9:
                d = math.hypot(px-x1, py-y1)
                if d < best_d:
                    best_d = d
                    best_c = (x1, y1)
                continue
            t = ((px-x1)*vx + (py-y1)*vy) / seg_len2
            t = max(0.0, min(1.0, t))
            cx, cy = (x1 + t*vx, y1 + t*vy)
            d = math.hypot(px-cx, py-cy)
            if d < best_d:
                best_d = d
                best_c = (cx, cy)
        return best_d, best_c

    def _pair_midpoints_hungarian(self, left_poly: List[Tuple[float, float]],
                                  right_poly: List[Tuple[float, float]]):
        if len(left_poly) == 0 or len(right_poly) == 0:
            return []
        A = np.array(left_poly, dtype=np.float32)
        B = np.array(right_poly, dtype=np.float32)
        dmat = np.sqrt(((A[:, None, :] - B[None, :, :])**2).sum(axis=2))
        big = 1e6
        dmat_masked = dmat.copy()
        dmat_masked[dmat_masked > MID_MAX_PAIR_DIST] = big
        row_ind, col_ind = linear_sum_assignment(dmat_masked)
        mids = []
        for i, j in zip(row_ind, col_ind):
            d = dmat[i, j]
            if d <= MID_MAX_PAIR_DIST:
                mids.append(((A[i,0]+B[j,0])*0.5, (A[i,1]+B[j,1])*0.5))
        mids.sort(key=lambda p: p[0])
        return mids

    def _pair_midpoints_projection(self, left_poly, right_poly):
        mids = []
        if left_poly and right_poly:
            for p in left_poly:
                _, q = self._nearest_point_on_poly(p, right_poly)
                mids.append(((p[0]+q[0])*0.5, (p[1]+q[1])*0.5))
            for q in right_poly:
                _, p = self._nearest_point_on_poly(q, left_poly)
                mids.append(((p[0]+q[0])*0.5, (p[1]+q[1])*0.5))
        return self._dedup_sort_xy(mids)

    def _pair_midpoints_resample_align(self, left_poly, right_poly):
        Ls = self._resample_poly(left_poly, RESAMPLE_STEP)
        Rs = self._resample_poly(right_poly, RESAMPLE_STEP)
        n = min(len(Ls), len(Rs))
        mids = [((Ls[i][0]+Rs[i][0])*0.5, (Ls[i][1]+Rs[i][1])*0.5) for i in range(n)]
        return self._dedup_sort_xy(mids)

    def _estimate_half_width(self, left_poly, right_poly):
        if not left_poly or not right_poly:
            return
        Ls = self._resample_poly(left_poly, RESAMPLE_STEP)
        widths = []
        for p in Ls:
            d, _ = self._nearest_point_on_poly(p, right_poly)
            widths.append(d)
        if len(widths) >= 3:
            w = float(np.median(widths))
            hw = 0.5 * w
            hw = max(HALF_WIDTH_MINMAX[0], min(HALF_WIDTH_MINMAX[1], hw))
            self._half_width_est = 0.8*self._half_width_est + 0.2*hw

    def _synthesize_from_single_side(self, poly: List[Tuple[float, float]], side: str):
        if not poly:
            return []
        res = self._resample_poly(poly, RESAMPLE_STEP)
        mids = []
        for i in range(len(res)-1):
            x1, y1 = res[i]
            x2, y2 = res[i+1]
            tx, ty = _unit(x2-x1, y2-y1)
            nx, ny = _rotate90(tx, ty)
            sign = -1.0 if side == 'left' else 1.0
            cx = x1 + sign * self._half_width_est * nx
            cy = y1 + sign * self._half_width_est * ny
            mids.append((cx, cy))
        return self._dedup_sort_xy(mids)

    def _dedup_sort_xy(self, pts: List[Tuple[float, float]]):
        if not pts:
            return []
        pts.sort(key=lambda p: (round(p[0]/0.05), round(p[1]/0.05)))
        out = []
        seen = set()
        for p in pts:
            k = (round(p[0]/0.05), round(p[1]/0.05))
            if k in seen:
                continue
            seen.add(k)
            out.append(p)
        out.sort(key=lambda p: p[0])
        return out

    # 중앙 경로 시작점
    def _center_start_node(self, mids: List[Tuple[float, float]]):
        try:
            cands = [(i, x) for i, (x, y) in enumerate(mids)
                     if START_X_RANGE[0] <= x < START_X_RANGE[1] and START_Y_CENTER[0] <= y < START_Y_CENTER[1]]
            return min(cands, key=lambda it: it[1])[0] if cands else (0 if mids else None)
        except Exception:
            return 0 if mids else None

    # 중앙 경로 Greedy (+ 헤딩 하드 게이트)
    def _greedy_center_path(self, mids: List[Tuple[float, float]],
                            left_poly: List[Tuple[float, float]],
                            right_poly: List[Tuple[float, float]]):
        if len(mids) < MIN_PATH_LEN:
            return []
        graph = self._build_graph_grid_radius(mids, CENTER_EDGE_LEN_TH)
        start = self._center_start_node(mids)
        if start is None:
            return []
        path = [start]
        seen = {start}
        steps = 0
        maxx = max(p[0] for p in mids)

        if self._prev_ref_dir is not None:
            ref_dir = self._prev_ref_dir
        else:
            ref_dir = self._compose_lane_dir(mids[start], left_poly, right_poly)
            if ref_dir == (0.0, 0.0):
                ref_dir = (1.0, 0.0)

        while steps < MAX_STEPS:
            steps += 1
            u = path[-1]
            px, py = mids[u]
            prev_dir = None
            if len(path) >= 2:
                ppx, ppy = mids[path[-2]]
                prev_dir = _unit(px-ppx, py-ppy)

            lane_fwd = self._compose_lane_dir((px, py), left_poly, right_poly)
            if lane_fwd == (0.0, 0.0):
                lane_fwd = ref_dir

            for cos_global_gate in (self.cos_global_delta, self.cos_global_relax):
                best = None
                best_score = 1e18

                for v in graph.get(u, []):
                    if v in seen:
                        continue
                    qx, qy = mids[v]
                    vx, vy = qx - px, qy - py
                    dist = math.hypot(vx, vy)
                    if dist <= MIN_DIST_TH:
                        continue
                    step_dir = (vx/dist, vy/dist)

                    if qx < px - CENTER_BACKTRACK_MARGIN:
                        continue
                    if _dot(step_dir, lane_fwd) < FWD_COS_TH:
                        continue
                    if _dot(step_dir, ref_dir) < cos_global_gate:
                        continue
                    if prev_dir is not None and _dot(step_dir, prev_dir) < self.cos_local_delta:
                        continue

                    lane_dir = self._compose_lane_dir((qx, qy), left_poly, right_poly)
                    dir_pen = (1.0 - _dot(step_dir, lane_dir))
                    dL, _ = self._point_seg_distance_and_tangent((qx, qy), left_poly)
                    dR, _ = self._point_seg_distance_and_tangent((qx, qy), right_poly)
                    corridor = min(dL, dR)
                    bd_pen = max(0.0, BD_MARGIN - corridor)
                    smooth_pen = 0.0 if prev_dir is None else (1.0 - _dot(prev_dir, step_dir))
                    score = dist + W_DIR*dir_pen + W_BD*bd_pen + W_SMOOTH*smooth_pen
                    score -= 1e-3 * (qx - px)

                    if score < best_score:
                        best_score = score
                        best = v

                if best is not None:
                    path.append(best)
                    seen.add(best)
                    break
            else:
                break

            if mids[path[-1]][0] >= maxx - 1e-6:
                break

        if len(path) < MIN_PATH_LEN:
            return []
        return path

    # (0,0) 앵커 고정 길이 경로 생성
    def _make_fixed_length_center(self, center_poly, left_poly, right_poly):
        ref_dir = self._prev_ref_dir if self._prev_ref_dir is not None else self._compose_lane_dir(ANCHOR_PT, left_poly, right_poly)
        if ref_dir == (0.0, 0.0):
            ref_dir = (1.0, 0.0)

        curve = center_poly[:] if center_poly else []
        path = [ANCHOR_PT]
        if curve:
            idx = min(range(len(curve)), key=lambda i: _euclid(curve[i], ANCHOR_PT))
            d0 = _euclid(curve[idx], ANCHOR_PT)
            if d0 > 1e-6:
                path.extend(self._linear_bridge(ANCHOR_PT, curve[idx], step=CENTER_STEP))
            path.extend(curve[idx+1:])

        resampled = [ANCHOR_PT]
        if len(path) >= 2:
            last = ANCHOR_PT
            for pt in path[1:]:
                seg = _euclid(last, pt)
                if seg < 1e-6:
                    last = pt
                    continue
                n = max(1, int(seg / CENTER_STEP))
                for k in range(1, n+1):
                    s = min(1.0, k / n)
                    q = (last[0] + s*(pt[0]-last[0]),
                         last[1] + s*(pt[1]-last[1]))
                    if _euclid(resampled[-1], q) >= 0.5*CENTER_STEP:
                        resampled.append(q)
                last = pt

        need = int(PATH_LENGTH_M / CENTER_STEP)
        out = resampled[:] if len(resampled) > 1 else [ANCHOR_PT]
        while len(out) <= need:
            base = out[-1]
            if len(out) >= 2:
                dvec = _unit(out[-1][0]-out[-2][0], out[-1][1]-out[-2][1])
                if dvec == (0.0, 0.0):
                    dvec = ref_dir
            else:
                dvec = ref_dir
            nxt = (base[0] + dvec[0]*CENTER_STEP, base[1] + dvec[1]*CENTER_STEP)
            out.append(nxt)

        return out[:need+1]

    # ────────────────────────────────────────────────────────────────────────
    # 메인 처리
    def _process(self, msg: TrackedConeArray):
        frame_id = (FALLBACK_FRAME if FORCE_FALLBACK_FRAME else
                    (msg.header.frame_id if (USE_HEADER_FRAME and msg.header.frame_id) else FALLBACK_FRAME))
        stamp = self.get_clock().now().to_msg()

        raw_pts, raw_zs, raw_keys = [], [], []
        for c in msg.cones:
            x, y, z = float(c.position.x), float(c.position.y), float(c.position.z)
            raw_pts.append((x, y)); raw_zs.append(z)
            raw_keys.append(_normalize_color_key(getattr(c, 'color', '')))

        arr = MarkerArray()
        mask = self._sanitize(raw_pts, raw_zs, raw_keys)
        if not mask:
            return arr, None
        pts  = [raw_pts[i] for i in mask]
        keys = [raw_keys[i] for i in mask]

        _ = self._build_graph_grid(pts) if len(pts) >= 2 else {}

        # 1) 좌/우 벽 라인 (Greedy) — 시작점 색상 기반 우선 키 선택
        left_start  = self._get_start_node(pts, START_Y_LEFT)
        right_start = self._get_start_node(pts, START_Y_RIGHT)
        left_prefer  = self._choose_prefer_keys(left_start,  keys, LEFT_KEYS)
        right_prefer = self._choose_prefer_keys(right_start, keys, RIGHT_KEYS)

        graph = self._build_graph_grid(pts) if len(pts) >= 2 else {}
        left_path_idx  = self._greedy_path(left_start,  graph, pts, keys, left_prefer)   if graph and left_start  is not None else []
        right_path_idx = self._greedy_path(right_start, graph, pts, keys, right_prefer)  if graph and right_start is not None else []
        if len(left_path_idx)  < MIN_PATH_LEN: left_path_idx  = []
        if len(right_path_idx) < MIN_PATH_LEN: right_path_idx = []
        left_poly  = [pts[i] for i in left_path_idx]
        right_poly = [pts[i] for i in right_path_idx]

        # 폭 추정
        if left_poly and right_poly:
            self._estimate_half_width(left_poly, right_poly)

        # 2) 중점 생성
        if left_poly and right_poly:
            mids_h = self._pair_midpoints_hungarian(left_poly, right_poly)
            if len(mids_h) >= PAIR_MIN_KEEP:
                mid_pts = mids_h
            else:
                mids_p = self._pair_midpoints_projection(left_poly, right_poly)
                mid_pts = mids_p if len(mids_p) >= PAIR_MIN_KEEP else self._pair_midpoints_resample_align(left_poly, right_poly)
        elif left_poly and not right_poly:
            mid_pts = self._synthesize_from_single_side(left_poly,  side='left')
        elif right_poly and not left_poly:
            mid_pts = self._synthesize_from_single_side(right_poly, side='right')
        else:
            mid_pts = self._prev_center_poly[:] if self._prev_center_poly else []

        mid_pts = self._densify_midpoints(mid_pts, left_poly, right_poly)

        # 3) 중앙 경로 (Greedy)
        center_idx: List[int] = []
        if len(mid_pts) >= MIN_PATH_LEN:
            center_idx = self._greedy_center_path(mid_pts, left_poly, right_poly)

        center_poly = [mid_pts[i] for i in center_idx] if center_idx else mid_pts[:]
        if len(center_poly) >= MIN_PATH_LEN and len(center_poly) >= 2:
            v = _unit(center_poly[1][0]-center_poly[0][0], center_poly[1][1]-center_poly[0][1])
            if self._prev_ref_dir is None:
                self._prev_ref_dir = v
            else:
                rx = REF_DIR_EMA_ALPHA*self._prev_ref_dir[0] + (1.0-REF_DIR_EMA_ALPHA)*v[0]
                ry = REF_DIR_EMA_ALPHA*self._prev_ref_dir[1] + (1.0-REF_DIR_EMA_ALPHA)*v[1]
                self._prev_ref_dir = _unit(rx, ry)

        # 4) (0,0) 앵커 고정 길이 경로로 변환 + Path 메시지 생성
        fixed_center = self._make_fixed_length_center(center_poly, left_poly, right_poly)
        self._prev_center_poly = fixed_center

        # Path 메시지 구성
        path_msg = Path()
        path_msg.header.frame_id = frame_id
        path_msg.header.stamp = stamp
        poses: List[PoseStamped] = []
        for x, y in fixed_center:
            ps = PoseStamped()
            ps.header.frame_id = frame_id
            ps.header.stamp = stamp
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            poses.append(ps)
        path_msg.poses = poses

        # ── RViz 출력 ───────────────────────────────────────────────────────
        def _mk_line(ns, idx, color_rgba, pts2d, zlift):
            mk = Marker()
            mk.header.frame_id = frame_id; mk.header.stamp = stamp
            mk.ns = ns; mk.id = idx
            mk.type = Marker.LINE_STRIP; mk.action = Marker.ADD
            mk.pose.orientation.w = 1.0
            mk.scale.x = 0.12 if ns == 'path' else 0.10
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = color_rgba
            mk.points = [Point(x=float(p[0]), y=float(p[1]), z=float(zlift)) for p in pts2d]
            mk.lifetime = self.life
            return mk

        if left_poly:
            arr.markers.append(_mk_line('path', 200000, LEFT_COLOR,  left_poly, 0.0))
        if right_poly:
            arr.markers.append(_mk_line('path', 200001, RIGHT_COLOR, right_poly, 0.0))

        if mid_pts:
            mk = Marker()
            mk.header.frame_id = frame_id; mk.header.stamp = stamp
            mk.ns = 'midpoints'; mk.id = 300000
            mk.type = Marker.SPHERE_LIST; mk.action = Marker.ADD
            mk.pose.orientation.w = 1.0
            mk.scale.x = mk.scale.y = mk.scale.z = MID_DOT_SCALE
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = MID_DOT_COLOR
            mk.points = [Point(x=float(p[0]), y=float(p[1]), z=float(0.1)) for p in mid_pts]
            mk.lifetime = self.life
            arr.markers.append(mk)

        if fixed_center:
            arr.markers.append(_mk_line('center_path', 300001, CENTER_COLOR, fixed_center, 0.2))

        return arr, path_msg


def main(args=None):
    rclpy.init(args=args)
    node = ConesColorSubscriber()
    try:
        execu = SingleThreadedExecutor()
        execu.add_node(node)
        execu.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

