#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
global_cones(visualization_msgs/MarkerArray) 퍼블리셔
- 입력 : placed_cones_temp.csv  (WGS84 lat/lon)
- 출력 : frame_id='reference' 평면좌표(x,y) CYLINDER 마커
"""

import math
import os
import pandas as pd

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory   # ✨

R_EARTH = 6_378_137.0  # [m]


def latlon_to_local(lat, lon, ref_lat, ref_lon):
    lat_r, lon_r = math.radians(lat), math.radians(lon)
    ref_lat_r, ref_lon_r = math.radians(ref_lat), math.radians(ref_lon)
    x = (lon_r - ref_lon_r) * math.cos(ref_lat_r) * R_EARTH
    y = (lat_r - ref_lat_r) * R_EARTH
    return x, y


class GlobalConePublisher(Node):
    def __init__(self):
        super().__init__("publish_global_cones")

        # ────── 매개변수 선언 ──────
        pkg_share = get_package_share_directory("gps_global_planner")
        default_csv = os.path.join(pkg_share, "data", "placed_cones_temp.csv")
        self.declare_parameter("csv_file", default_csv)
        # 127.0505869,37.5573749  ← 동일 기준점
        self.declare_parameter("ref_lat", 37.5573749)
        self.declare_parameter("ref_lon", 127.0505869)
        self.declare_parameter("frame_id", "reference")        # ✨

        # ────── 매개변수 읽기 ──────
        csv_file = self.get_parameter("csv_file").value
        self.ref_lat = self.get_parameter("ref_lat").value
        self.ref_lon = self.get_parameter("ref_lon").value
        self.frame_id = self.get_parameter("frame_id").value

        # ────── CSV 로드 ──────
        try:
            df = pd.read_csv(csv_file)
        except Exception as e:
            self.get_logger().fatal(f"CSV 읽기 실패: {e}")
            raise e

        lat_col = [c for c in df.columns if "lat" in c.lower()][0]
        lon_col = [c for c in df.columns if "lon" in c.lower()][0]
        self.cones_local = [
            latlon_to_local(df.loc[i, lat_col],
                            df.loc[i, lon_col],
                            self.ref_lat, self.ref_lon)
            for i in range(len(df))
        ]
        self.get_logger().info(f"{len(self.cones_local)}개의 콘 좌표 로드 완료")

        # ────── Publisher & Timer ──────
        self._pub = self.create_publisher(MarkerArray, "global_cones", 10)
        self.create_timer(1.0, self._publish_markers)  # 1 Hz

    # ---------------- 내부 ----------------
    def _publish_markers(self):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        for idx, (x, y) in enumerate(self.cones_local):
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = self.frame_id      # "reference"
            m.ns = "cones"
            m.id = idx
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.5
            m.scale.z = 0.75
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.55, 0.0, 1.0
            marker_array.markers.append(m)

        self._pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalConePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
