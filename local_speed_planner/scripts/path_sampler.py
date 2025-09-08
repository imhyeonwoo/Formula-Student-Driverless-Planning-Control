#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import copy
import math


class PathSampler(Node):
    def __init__(self):
        super().__init__('path_sampler')

        # 구독 및 발행 설정
        self.sub_path = self.create_subscription(
            Path, '/local_planned_path', self.on_path, 10)
        self.pub_sampled = self.create_publisher(
            Path, '/sp/sampled_waypoints', 10)
        self.pub_markers = self.create_publisher(
            MarkerArray, '/sp/curvature', 10)
        # 속도 상한 시각화는 별도 노드에서 처리 예정
        # 대표 곡률(Reference Curvature) 퍼블리셔
        self.pub_rep_kappa = self.create_publisher(
            Float32, '/sp/rep_curvature', 10)

        # 파라미터: 샘플 간격 (m)
        self.declare_parameter('sample_dist', 0.5)
        self.sample_dist = self.get_parameter('sample_dist').value

        # 시작부(base_link 연결부) 곡률 무시 개수
        # /local_planned_path의 시작부는 잡음이 심하므로(∵강제로 잇기 때문) 곡률 계산에서 무시
        self.declare_parameter('ignore_front_n', 5)
        self.ignore_front_n = self.get_parameter('ignore_front_n').value

        # 곡률 평균(평활화) 윈도우 크기 (샘플 수, 홀수 권장)
        self.declare_parameter('curv_avg_window', 5)
        self.curv_avg_window = max(1, int(self.get_parameter('curv_avg_window').value))

        # 대표 곡률 산출 파라미터
        self.declare_parameter('rep_window_m', 15.0)   # m 단위, 전방 집계 구간 길이
        self.declare_parameter('rep_method', 'percentile')  # 'percentile'|'mean'|'max'
        self.declare_parameter('rep_percentile', 90.0)      # 0~100
        self.rep_window_m = float(self.get_parameter('rep_window_m').value)
        self.rep_method = str(self.get_parameter('rep_method').value)
        self.rep_percentile = float(self.get_parameter('rep_percentile').value)

        # 속도 상한 관련 파라미터는 별도 노드에서 관리

        # 디버깅용: 최소 막대 높이
        self.min_height = 0.05

    def on_path(self, msg: Path):
        if len(msg.poses) < 3:
            return

        # --- 등간격 샘플링 ---
        sampled_path = Path()
        sampled_path.header = msg.header

        cum_dist = [0.0]
        for i in range(1, len(msg.poses)):
            dx = msg.poses[i].pose.position.x - msg.poses[i-1].pose.position.x
            dy = msg.poses[i].pose.position.y - msg.poses[i-1].pose.position.y
            cum_dist.append(cum_dist[-1] + math.hypot(dx, dy))

        total_len = cum_dist[-1]
        s_targets = [s for s in self._frange(0.0, total_len, self.sample_dist)]

        j = 0
        for s in s_targets:
            while j < len(cum_dist)-1 and cum_dist[j+1] < s:
                j += 1
            if j >= len(cum_dist)-1:
                break
            ratio = (s - cum_dist[j]) / max(1e-6, (cum_dist[j+1] - cum_dist[j]))
            px = (1-ratio)*msg.poses[j].pose.position.x + ratio*msg.poses[j+1].pose.position.x
            py = (1-ratio)*msg.poses[j].pose.position.y + ratio*msg.poses[j+1].pose.position.y
            # 원본 메시지 객체를 변경하지 않도록 깊은 복사 사용
            pose_copy = copy.deepcopy(msg.poses[j])
            pose_copy.pose.position.x = px
            pose_copy.pose.position.y = py
            sampled_path.poses.append(pose_copy)

        self.pub_sampled.publish(sampled_path)

        # --- 곡률 계산 및 평활화 ---
        N = len(sampled_path.poses)
        kappa_raw = [0.0] * N
        for i in range(1, N - 1):
            # base_link 연결부 무시
            if i <= int(self.ignore_front_n):
                kappa_raw[i] = 0.0
                continue
            p_prev = sampled_path.poses[i - 1].pose.position
            p = sampled_path.poses[i].pose.position
            p_next = sampled_path.poses[i + 1].pose.position
            a = math.dist([p_prev.x, p_prev.y], [p.x, p.y])
            b = math.dist([p.x, p.y], [p_next.x, p_next.y])
            c = math.dist([p_prev.x, p_prev.y], [p_next.x, p_next.y])
            s_tri = (a + b + c) / 2.0
            area = max(s_tri * (s_tri - a) * (s_tri - b) * (s_tri - c), 0.0)
            curvature = 0.0 if area == 0 else (4.0 * math.sqrt(area)) / (a * b * c)
            kappa_raw[i] = curvature

        # 이동 평균(간단 평활화)
        win = max(1, int(self.curv_avg_window))
        half = max(0, (win - 1) // 2)
        kappa_smooth = [0.0] * N
        for i in range(N):
            s_idx = max(0, i - half)
            e_idx = min(N - 1, i + half)
            cnt = max(1, e_idx - s_idx + 1)
            acc = 0.0
            for j in range(s_idx, e_idx + 1):
                acc += kappa_raw[j]
            kappa_smooth[i] = acc / cnt

        # --- 곡률 프로파일 마커 생성 ---
        markers = MarkerArray()
        for i in range(1, N - 1):
            if i <= int(self.ignore_front_n):
                continue
            p = sampled_path.poses[i].pose.position
            curvature = kappa_smooth[i]
            height = max(curvature * 3.0, self.min_height)
            col_r = min(1.0, curvature * 5.0)
            col_g = 0.0
            col_b = max(0.0, 1.0 - curvature * 5.0)
            col_a = 1.0

            m = Marker()
            m.header = msg.header
            m.ns = "curvature_profile"
            m.id = i
            m.type = Marker.LINE_LIST
            m.action = Marker.ADD
            m.scale.x = 0.1
            m.color.r, m.color.g, m.color.b, m.color.a = col_r, col_g, col_b, col_a

            p0 = Point()
            p0.x, p0.y, p0.z = p.x, p.y, 0.0
            p1 = Point()
            p1.x, p1.y, p1.z = p.x, p.y, height

            m.points.append(p0)
            m.points.append(p1)
            markers.markers.append(m)

        self.pub_markers.publish(markers)

        # --- 대표 곡률 계산 및 퍼블리시 ---
        rep_val = self._compute_representative_kappa(kappa_smooth, self.sample_dist)
        rep_msg = Float32()
        rep_msg.data = float(rep_val)
        self.pub_rep_kappa.publish(rep_msg)

        # 속도 상한 계산/시각화는 제거 (별도 노드 예정)

    def _frange(self, start, stop, step):
        while start <= stop:
            yield start
            start += step

    def _compute_representative_kappa(self, kappa_list, ds):
        N = len(kappa_list)
        if N == 0:
            return 0.0
        # 시작부 무시 구간 이후부터 집계
        start_i = min(max(int(self.ignore_front_n), 1), N - 1)
        # 전방 윈도우 크기(샘플 수)
        window_n = max(1, int(self.rep_window_m / max(ds, 1e-6)))
        end_i = min(N, start_i + window_n)
        segment = kappa_list[start_i:end_i]
        if not segment:
            segment = kappa_list[start_i:]
        if not segment:
            return 0.0

        method = self.rep_method.lower()
        if method == 'mean':
            return sum(segment) / len(segment)
        elif method == 'max':
            return max(segment)
        else:
            # percentile (기본)
            p = max(0.0, min(100.0, self.rep_percentile))
            idx = int(round((p / 100.0) * (len(segment) - 1)))
            sorted_seg = sorted(segment)
            return sorted_seg[idx]
        """
        mean 방식 : segment 안의 곡률 값들을 모두 더한 뒤 그 개수로 나누어 평균을 구함
        max 방식 : segment 안의 곡률 값들 중에서 가장 큰 값을 선택함
        percentile 방식 : segment 안의 곡률 값들을 오름차순으로 정렬한 뒤, 지정된 백분위수에 해당하는 값을 선택함
        예를 들어, 90번째 백분위수는 전체 값들 중에서 상위 10%에 해당하는 값을 의미함
        """


def main(args=None):
    rclpy.init(args=args)
    node = PathSampler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
