#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from visualization_msgs.msg import Marker
from custom_interface.msg import TrackedConeArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, UInt8

qos = QoSProfile(depth=10)
qos.reliability = ReliabilityPolicy.BEST_EFFORT

# =================== 전역 파라미터 =================== #
#----- AEB (defaults; can be overridden via params) -----#
AEB_THRESH = 4   # 빨간콘 몇 개 이상일 때 AEB 작동시킬지
AEB_HOR = 6.0     # AEB 작동 영역의 좌우 반폭(Horizontal)
AEB_VER = 9.0     # AEB 작동 영역의 앞쪽 길이(Vertical)

#=====================================================#

class AEB(Node):
    def __init__(self):
        super().__init__('AEB_Determine_Node')

        ###### Parameters ######
        self.declare_parameter('topic', '/cone/fused/ukf')
        self.declare_parameter('aeb_thresh', AEB_THRESH)
        self.declare_parameter('aeb_hor', AEB_HOR)
        self.declare_parameter('aeb_ver', AEB_VER)

        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.aeb_thresh = int(self.get_parameter('aeb_thresh').get_parameter_value().integer_value)
        self.aeb_hor = float(self.get_parameter('aeb_hor').get_parameter_value().double_value)
        self.aeb_ver = float(self.get_parameter('aeb_ver').get_parameter_value().double_value)

        ###### Subscriber ######
        self.red_sub = self.create_subscription(
            TrackedConeArray, self.topic, self.red_callback, qos)
        

        ###### Publisher ######
        self.aeb_pub = self.create_publisher(UInt8, '/aeb', 10)

        # ----- Visualize -----#
        self.redcone_pub = self.create_publisher(Marker, '/red_cone_marker', 10)
        self.aeb_roi_pub = self.create_publisher(Marker, '/aeb_roi', 10)

        #----- 콘 좌표 정의 -----#
        self.red_cone: List[Tuple[float, float, float]] = []

        #----- 메인루프 주기적 실행 -----#
        self.create_timer(0.1, self.main_loop)

    #=======================
    #  Callbacks
    #=======================
    def red_callback(self, msg: TrackedConeArray):
        if len(msg.cones) < 3:
            return
        self.red_cone = []
        for cone in msg.cones:
            if getattr(cone, "color", "") == "Red Cone":
                x, y, z = cone.position.x, cone.position.y, cone.position.z
                self.red_cone.append((x, y, z))

    #=======================
    #  AEB Determination
    #=======================
    def determine_aeb(self, cone: List[Tuple[float, float, float]]):
        n_red = 0
        for x, y, z in cone:
            if 0.0 <= x <= self.aeb_ver and -self.aeb_hor <= y <= self.aeb_hor:
                n_red += 1
        aebmsg = UInt8()
        # Trigger when count meets or exceeds threshold
        aebmsg.data = 1 if n_red >= self.aeb_thresh else 0
        self.aeb_pub.publish(aebmsg)
        # Debug (throttled) — shows ROI count and output
        if (self.get_clock().now().nanoseconds // 1_000_000_000) % 1 == 0:
            self.get_logger().debug(
                f"AEB ROI count={n_red} (th={self.aeb_thresh}) -> {aebmsg.data}")
        return aebmsg.data

    #=======================
    #  Visualization
    #=======================
    def vis_RedCone(self):
        marker = Marker()
        marker.header.frame_id = 'os_sensor'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'Red Cones'
        marker.id = 7
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.pose.orientation.w = 1.0
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
        for x, y, z in self.red_cone:
            marker.points.append(Point(x=float(x), y=float(y), z=float(z)))
        self.redcone_pub.publish(marker)

    def vis_AEBzone(self, aeb_flag: int):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'AEB Zone'
        marker.id = 8
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = self.aeb_ver
        marker.scale.y = 2 * self.aeb_hor
        marker.scale.z = 0.1
        marker.pose.position.x = self.aeb_ver / 2.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=(0.8 if aeb_flag == 1 else 0.2))
        self.aeb_roi_pub.publish(marker)

    #=======================
    #  Main Loop
    #=======================
    def main_loop(self):
        # AEB 판정 및 ROI는 항상 퍼블리시
        aeb_flag = self.determine_aeb(self.red_cone)
        self.vis_AEBzone(aeb_flag)

        # 콘 시각화는 데이터가 있을 때만
        if self.red_cone:
            self.vis_RedCone()

def main(args=None):
    rclpy.init(args=args)
    node = AEB()
    node.get_logger().info('AEB Determination Node Started')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
