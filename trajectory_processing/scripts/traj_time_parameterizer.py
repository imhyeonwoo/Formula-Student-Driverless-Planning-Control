#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
traj_time_parameterizer.py
──────────────────────────────────────────────────────────────
등간격 Path + 속도 프로파일 → 등시간격 Trajectory
 • 입력
     /local_planned_path      (nav_msgs/Path)
     /desired_speed_profile   (std_msgs/Float32MultiArray) ─ 선택
 • 출력
     /time_param_path         (nav_msgs/Path)

파라미터
 ├ dt           : 등시간 간격 [s]      (default 0.1)
 ├ v_nom        : 속도 미지 시 기본값  (default 5.0 m/s)
 └ preview_time : 최대 변환 길이 [s]  (default 3.0)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped


class TrajTimeParameterizer(Node):
    def __init__(self):
        super().__init__("traj_time_parameterizer")

        # ─── 파라미터 ───────────────────────────────────────
        self.dt           = self.declare_parameter("dt", 0.1).value
        self.v_nom        = self.declare_parameter("v_nom", 0.0).value
        self.preview_time = self.declare_parameter("preview_time", 3.0).value

        # ─── 내부 상태 ──────────────────────────────────────
        self.speed_profile: list[float] = []   # 최신 속도 배열

        # ─── Pub / Sub ─────────────────────────────────────
        self.pub_traj = self.create_publisher(Path, "/time_param_path", 10)

        self.create_subscription(Path,
                                 "/local_planned_path",
                                 self.path_callback, 10)

        self.create_subscription(Float32MultiArray,
                                 "/desired_speed_profile",
                                 self.speed_callback, 10)

        self.get_logger().info("traj_time_parameterizer node started")

    # ======================================================
    # speed_profile 수신
    # ======================================================
    def speed_callback(self, msg: Float32MultiArray):
        self.speed_profile = list(msg.data)

    # ======================================================
    # Path 수신 → 등시간 Trajectory 변환
    # ======================================================
    def path_callback(self, path: Path):
        N = len(path.poses)
        if N < 2:
            self.get_logger().warn("Path too short – skip")
            return

        # 1) 좌표 배열
        X = np.array([p.pose.position.x for p in path.poses])
        Y = np.array([p.pose.position.y for p in path.poses])

        # 2) 거리 d_i
        d = np.hypot(np.diff(X), np.diff(Y))          # N-1

        # 3) 구간 평균 속도 v_seg
        #    ─ speed_profile 길이가 짧으면 min 길이까지만 사용
        if self.speed_profile:
            sp   = np.array(self.speed_profile, float)
            sp   = sp[:N] if len(sp) >= N else np.pad(sp, (0, N-len(sp)), 'edge')
            v_i  = np.maximum(sp, 1e-3)               # 0 방지
            v_seg = 0.5 * (v_i[:-1] + v_i[1:])        # N-1
        else:
            v_seg = np.full(N-1, self.v_nom)          # 고정 속도

        # 4) 세그먼트 시간 Δt_i
        dt_seg = d / v_seg

        # 5) 누적 시간 테이블
        T = np.insert(np.cumsum(dt_seg), 0, 0.0)
        total_T = T[-1]

        # 6) 등시간 격자
        t_q = np.arange(0.0,
                        min(total_T, self.preview_time) + 1e-9,
                        self.dt)

        # 7) 좌표 보간
        x_q = np.interp(t_q, T, X)
        y_q = np.interp(t_q, T, Y)

        # 8) Path 메시지 작성
        traj = Path()
        traj.header = path.header
        traj.header.stamp = self.get_clock().now().to_msg()

        for tq, x, y in zip(t_q, x_q, y_q):
            ps = PoseStamped()
            ps.header.frame_id = path.header.frame_id
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            traj.poses.append(ps)

        self.pub_traj.publish(traj)
        self.get_logger().debug(
            f"Published {len(traj.poses)} pts  (T={t_q[-1]:.2f}s) "
            f"[spd {'on' if self.speed_profile else 'nom'}]"
        )


# ─────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = TrajTimeParameterizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
