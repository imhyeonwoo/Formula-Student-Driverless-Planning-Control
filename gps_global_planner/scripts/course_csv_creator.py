#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
course_csv_creator.py   (updated 2025-07-07)

ROS 2 bag → CSV 변환 스크립트
 - input  : /ublox_gps_node/fix  (sensor_msgs/msg/NavSatFix)
 - output : index, Long, Lat, UTM_X, UTM_Y, Cov00
            └─ Cov00 = position_covariance[0]  ← 기존 Status 대신
"""

import os
import sys
import csv
import importlib

import rclpy
from rclpy.logging import get_logger

import utm
import rosbag2_py
from rclpy.serialization import deserialize_message

# ───────── 기본 경로 ─────────
PKG_SRC_DIR = "/home/ihw/workspace/kai_2025/src/Planning/gps_global_planner"
DATA_DIR    = os.path.join(PKG_SRC_DIR, "data")
os.makedirs(DATA_DIR, exist_ok=True)

# BAG_URI_DEFAULT = "/home/ihw/workspace/bag/gps_imu_car/ilgam_250721"
BAG_URI_DEFAULT = "/home/ihw/workspace/bag/4planning_250722_nocheon_rtk_curved"
# CSV_OUT_DEFAULT = os.path.join(DATA_DIR, "ilgam_250721.csv")
CSV_OUT_DEFAULT = os.path.join(DATA_DIR, "4planning_250722_nocheon_rtk_curved.csv")
TOPIC_NAME      = "/ublox_gps_node/fix"

# ======================================================================
def create_course_csv_ros2(bag_uri: str, csv_path: str) -> None:
    log = get_logger("course_csv_creator")

    if not os.path.exists(bag_uri):
        log.error(f"Bag 디렉터리가 존재하지 않습니다: {bag_uri}")
        sys.exit(1)

    # 1) Bag 열기 -------------------------------------------------------
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_uri, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", "")
    )

    # 2) 토픽/타입 확인 --------------------------------------------------
    topics_meta = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if TOPIC_NAME not in topics_meta:
        log.error(f"Bag 안에 '{TOPIC_NAME}' 토픽이 없습니다.")
        sys.exit(1)

    msg_type = topics_meta[TOPIC_NAME]        # "sensor_msgs/msg/NavSatFix"
    pkg, _, msg = msg_type.split("/")
    MsgClass = getattr(importlib.import_module(f"{pkg}.msg"), msg)

    # 3) CSV 작성 -------------------------------------------------------
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        # ★ Header: Status → Cov00 로 변경
        writer.writerow(["index", "Long", "Lat", "UTM_X", "UTM_Y", "Cov00"])

        idx = 0
        while reader.has_next():
            topic, raw, _ = reader.read_next()
            if topic != TOPIC_NAME:
                continue

            msg_obj: MsgClass = deserialize_message(raw, MsgClass)
            lat, lon = msg_obj.latitude, msg_obj.longitude
            # ★ Status 대신 공분산[0] 사용
            cov00    = float(msg_obj.position_covariance[0])

            try:
                easting, northing, *_ = utm.from_latlon(lat, lon)
            except Exception as e:
                log.warning(f"UTM 변환 실패(idx={idx}): {e}")
                continue

            # ★ cov00 저장
            writer.writerow([idx, lon, lat, easting, northing, cov00])
            idx += 1

    log.info(f"CSV 생성 완료: {csv_path}  (총 {idx} 샘플)")

# ======================================================================
def main() -> None:
    rclpy.init()
    log = get_logger("course_csv_creator")

    bag_uri = BAG_URI_DEFAULT
    csv_out = CSV_OUT_DEFAULT
    if len(sys.argv) > 1:
        bag_uri = sys.argv[1]
    if len(sys.argv) > 2:
        csv_out = sys.argv[2]

    log.info(f"Bag  디렉터리 : {bag_uri}")
    log.info(f"CSV 출력 파일 : {csv_out}")

    try:
        create_course_csv_ros2(bag_uri, csv_out)
    finally:
        rclpy.shutdown()

# ======================================================================
if __name__ == "__main__":
    main()
