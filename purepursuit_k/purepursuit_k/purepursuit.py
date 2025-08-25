#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math
import numpy as np
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

'''
전역변수 선언
'''
#---- Vehicle Params (단위 : m) ----
WHEELBASE = 3.0
MAX_STEERING = 45
MAX_SPEED = 60
CAR_POSITION = (0,0)
SPEED = 10.0

#---- Pure Pursuit Logic Params ----
#LAD = 2.0              #고정 Look Ahead #ROS2 파라미터 설정으로 인한 주석처리
MAX_LAD = 3.0
MIN_LAD = 1.0

#---- Data ----
CURRNET_SPEED = 0.0
PATH_INTERVAL = 0.2     #경로 샘플링 간격

class PurePursuitControl(Node):
    def __init__(self):
        super().__init__('PurePursuitControl')

        ##### Params #####
        self.declare_parameter('LAD', 2.0)
        self.declare_parameter('WHEELBASE', 3.0)
        self.declare_parameter('SPEED', 10.0)

        ##### Load Params #####
        self.LAD = self.get_parameter('LAD').value
        self.WHEELBASE = self.get_parameter('WHEELBASE').value
        self.SPEED = self.get_parameter('SPEED').value

        ###### Subscriber ######

        #경로
        self.create_subscription(
            Path, 
            '/local_planned_path', 
            self.callback_path, 
            10)
        
        # 현재속도
        self.create_subscription(
            Float32MultiArray, 
            '/current_speed', 
            self.callback_currentspeed, 
            10)

        ###### Publisher ######
        self.pub_speed = self.create_publisher(Float32MultiArray, '/desired_speed', 10)
        self.speed_msg = Float32MultiArray()
        self.pub_str = self.create_publisher(Float32MultiArray, '/cmd/steering_angle', 10)
        self.str_msg = Float32MultiArray()

        #----- Visualize -----#
        self.pub_lad = self.create_publisher(Marker, '/lookahead_point_marker', 1)

        #----- Initialize -----#
        self.path = []              # 경로
        self.current_speed = 0.0    # 현재 속도

        #----- Timer -----#
        self.timer = self.create_timer(0.1, self.main_loop)

        self.get_logger().info('----- Pure Pursuit Control Started -----')

    #----- Callback : Path -----#
    def callback_path(self, msg: Path):
        path_xy = []
        poses = msg.poses
        if not poses:
            self.path = []
            return
        last = None
        for ps in poses:
            x = float(ps.pose.position.x)
            y = float(ps.pose.position.y)
            # 유효성 체크
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            if last is None or math.hypot(x - last[0], y - last[1]) >= PATH_INTERVAL:
                path_xy.append((x, y))
                last = (x, y)

        self.path = path_xy

    #----- Callback : Current Speed -----#
    def callback_currentspeed(self, msg = Float32MultiArray):
        data = msg.data
        self.current_speed = data[0]
        return

    #----- 동적 lad (추후 수정) -----#
    def dynamic_lad(self):
        lad = 0
        return lad
    
    #----- 타겟 좌표 선정 -----#
    def find_target(self, lad):
        if not self.path:
            if not self.path:
                self.get_logger().warn("Path is empty. Returning (0.0, 0.0) as target.")
                return (0.0, 0.0)
        target = None
        min_err = float('inf')

        for (x,y) in self.path:
            distance = math.hypot(x,y)
            err = abs(lad - distance)
            if err < min_err:
                min_err = err
                target = (x,y)
        
        if target is None:
            return (0.0, 0.0)
        return target
    
    #----- 조향각 계산 -----#
    def compute_steering(self, target):
        dx = target[0] - CAR_POSITION[0]
        dy = target[1] - CAR_POSITION[1]
        alpha = math.atan2(dy, dx)
        
        steering = math.atan2(2.0 * self.WHEELBASE * math.sin(alpha), math.hypot(dx, dy))
        return steering
    
    #----- 조향각,속도 퍼블리시 -----#
    def drive(self, steering, speed):
        self.str_msg.data = [float(steering)]
        self.pub_str.publish(self.str_msg)

        # self.speed_msg.data = [float(speed)]
        # self.pub_speed.publish(self.speed_msg)

    ###### Visualize ######
    #----- Look Ahead Distance -----#
    def vis_lad(self, lad: float):
        if lad is None or lad <= 0:
            return

        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'lad_semicircle'
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)
        marker.pose.orientation.w = 1.0
        cx, cy = float(CAR_POSITION[0]), float(CAR_POSITION[1])
        num_pts = 64
        for theta in np.linspace(-math.pi/2, math.pi/2, num_pts):
            x = cx + lad * math.cos(theta)   # +x 전방
            y = cy + lad * math.sin(theta)
            marker.points.append(Point(x=x, y=y, z=0.0))
        self.pub_lad.publish(marker)

    def main_loop(self):
        # 경로가 없을 경우
        if self.path is None:
            return
        
        #----- Look Ahead 정의 -----#
        lad = self.LAD

        #----- 타겟좌표 선정 -----#
        target = self.find_target(lad)
        if target is None:
            self.get_logger().warn(" 유효 타겟 없음 ")

        #----- 조향각 산출 -----#
        steering_angle = self.compute_steering(target)
        steering_angle = np.clip(steering_angle, math.radians(-MAX_STEERING), math.radians(MAX_STEERING))

        #----- 속도 (추후 값 지정) ----#
        speed = self.SPEED

        #----- 제어값 퍼블리시 -----#
        self.drive(steering_angle, speed)

        #----- 시각화 -----#
        self.vis_lad(lad)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
