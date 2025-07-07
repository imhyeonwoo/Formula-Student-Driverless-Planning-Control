#!/usr/bin/env python3
"""local_xy 궤적 → yaw 추정 (IMU 도입 전 임시)"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32


def wrap_angle(a):
    while a > math.pi:
        a -= 2*math.pi
    while a < -math.pi:
        a += 2*math.pi
    return a

class GlobalYawEstimator(Node):
    def __init__(self):
        super().__init__('global_yaw_estimator')
        self.prev_x = self.prev_y = None
        self.filtered_yaw = None
        self.alpha = self.declare_parameter('alpha', 0.2).value

        self.pub_raw = self.create_publisher(Float32, 'raw_global_yaw', 10)
        self.pub_filt = self.create_publisher(Float32, 'global_yaw', 10)
        self.create_subscription(PointStamped, 'local_xy', self.cb_xy, 10)
        self.get_logger().info('GlobalYawEstimator started (subscribe local_xy)')

    def cb_xy(self, msg: PointStamped):
        x, y = msg.point.x, msg.point.y
        if self.prev_x is not None:
            dx, dy = x - self.prev_x, y - self.prev_y
            if math.hypot(dx, dy) > 0.1:
                raw = wrap_angle(math.atan2(dy, dx))
                self.pub_raw.publish(Float32(data=raw))
                if self.filtered_yaw is None:
                    self.filtered_yaw = raw
                else:
                    diff = wrap_angle(raw - self.filtered_yaw)
                    self.filtered_yaw = wrap_angle(self.filtered_yaw + self.alpha * diff)
                self.pub_filt.publish(Float32(data=self.filtered_yaw))
                self.get_logger().debug(
                    f"Yaw raw={math.degrees(raw):.1f}°, filt={math.degrees(self.filtered_yaw):.1f}°")
        self.prev_x, self.prev_y = x, y


def main():
    rclpy.init()
    node = GlobalYawEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()