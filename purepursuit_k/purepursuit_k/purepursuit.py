#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math
import numpy as np
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

#---- Vehicle Params ----
MAX_STEERING = 30      # [deg]
CAR_POSITION = (0.0, 0.0)
PATH_INTERVAL = 0.2
SPEED_2_RPM = 0.00535  # m/s -> rpm
LAD_MIN = 4.2
LAD_MAX = 10.0
LAD_MINMAX_THRESH = 7.0


class PurePursuitControl(Node):
    def __init__(self):
        super().__init__('PurePursuitControl')

        # Params
        self.declare_parameter('WHEELBASE', 1.295)
        self.declare_parameter('SPEED', 20.0)
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('LAD_MIN', LAD_MIN) #초기값
        self.declare_parameter('LAD_MAX', LAD_MAX) #초기값

        self.WHEELBASE = float(self.get_parameter('WHEELBASE').value)
        self.SPEED = float(self.get_parameter('SPEED').value)
        self.target_frame = str(self.get_parameter('target_frame').value)
        self.LAD_MIN = float(self.get_parameter('LAD_MIN').value)
        self.LAD_MAX = float(self.get_parameter('LAD_MAX').value)

        # Subscribers
        self.create_subscription(Path, '/local_planned_path', self.callback_path, 10)
        self.create_subscription(Float32, '/current_speed', self.callback_currentspeed, 10)

        # Publishers
        self.pub_rpm = self.create_publisher(Float32, '/cmd/rpm', 10)
        self.pub_steer = self.create_publisher(Float32, '/cmd/steer', 10)

        # Visualize
        self.pub_lad = self.create_publisher(Marker, '/lookahead_point_marker', 1)
        self.pub_target = self.create_publisher(Marker, '/pp_target_marker', 1)
        self.pub_line = self.create_publisher(Marker, '/pp_target_line', 1)

        # State
        self.path = []
        self.current_speed = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer
        self.timer = self.create_timer(0.1, self.main_loop)

        self.get_logger().info('----- Pure Pursuit Control Started (Variable LAD) -----')

    def callback_path(self, msg: Path):
        path_xy = []
        poses = msg.poses
        if not poses:
            self.path = []
            return

        src_frame = msg.header.frame_id if msg.header.frame_id else self.target_frame

        last = None
        for ps in poses:
            x = float(ps.pose.position.x)
            y = float(ps.pose.position.y)
            if not (math.isfinite(x) and math.isfinite(y)):
                continue

            if src_frame != self.target_frame:
                out = self._transform_xy(x, y, src_frame, self.target_frame)
                if out is None:
                    continue
                x, y = out

            if last is None or math.hypot(x - last[0], y - last[1]) >= PATH_INTERVAL:
                path_xy.append((x, y))
                last = (x, y)

        if src_frame != self.target_frame and path_xy:
            self.get_logger().info(
                f"Transformed path from {src_frame} to {self.target_frame} (N={len(path_xy)})"
            )

        self.path = path_xy

    def callback_currentspeed(self, msg: Float32):
        self.current_speed = float(msg.data)

    def _quat_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _transform_xy(self, x, y, src_frame, target_frame):
        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                target_frame, src_frame, Time())
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed {src_frame}->{target_frame}: {e}")
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = self._quat_to_yaw(q)
        c, s = math.cos(yaw), math.sin(yaw)

        xb = c * x - s * y + t.x
        yb = s * x + c * y + t.y
        return (float(xb), float(yb))

    # ★★★ 변경된 함수: 가변 LAD 계산 로직 ★★★
    def calculate_variable_lad(self):
        if len(self.path) < 2:
            return self.LAD_MIN

        angles = []
        for i in range(1, len(self.path)):
            p_prev = self.path[i-1]
            p_curr = self.path[i]
            dx = p_curr[0] - p_prev[0]
            dy = p_curr[1] - p_prev[1]
            angle = abs(math.atan2(dy, dx))
            angles.append(abs(angle))

        if not angles:
            return self.LAD_MIN

        avg_angle = sum(angles) / len(angles)

        normalized_angle = avg_angle / (math.pi / 4.0)
        lad = self.LAD_MAX - (normalized_angle * (self.LAD_MAX - self.LAD_MIN))
        lad = float(np.clip(lad, self.LAD_MIN, self.LAD_MAX))
        lad = LAD_MAX if lad > LAD_MINMAX_THRESH else LAD_MIN
        return lad

    def find_target(self, lad: float):
        if not self.path:
            self.get_logger().warn("Path is empty. Returning (0.0, 0.0) as target.")
            return (0.0, 0.0)
        target = None
        min_err = float('inf')
        for (x, y) in self.path:
            distance = math.hypot(x, y)
            err = abs(lad - distance)
            if err < min_err:
                min_err = err
                target = (x, y)
        return target if target is not None else (0.0, 0.0)

    def compute_steering(self, target):
        dx = target[0] - CAR_POSITION[0]
        dy = target[1] - CAR_POSITION[1]
        alpha = math.atan2(dy, dx)
        steering = -1 * math.atan2(2.0 * self.WHEELBASE * math.sin(alpha), math.hypot(dx, dy))
        return steering

    def compute_speed2rpm(self, speed):
        rpm = (speed / 3.6) / SPEED_2_RPM  # km/h -> m/s -> RPM
        return rpm

    def drive(self, steering, speed):
        steer_msg = Float32()
        steer_msg.data = float(steering)
        self.pub_steer.publish(steer_msg)

        rpm_msg = Float32()
        rpm_msg.data = float(speed)
        self.pub_rpm.publish(rpm_msg)

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
        cx, cy = CAR_POSITION
        num_pts = 64
        for theta in np.linspace(-math.pi/2, math.pi/2, num_pts):
            x = cx + lad * math.cos(theta)
            y = cy + lad * math.sin(theta)
            marker.points.append(Point(x=x, y=y, z=0.0))
        self.pub_lad.publish(marker)

    def vis_target(self, target):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'pp_target'
        marker.id = 2
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color = ColorRGBA(r=1.0, g=0.2, b=1.0, a=1.0)
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = float(target[0])
        marker.pose.position.y = float(target[1])
        marker.pose.position.z = 0.05
        self.pub_target.publish(marker)

    def vis_str(self, target):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'pp_target_line'
        marker.id = 3
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.04
        marker.color = ColorRGBA(r=1.0, g=0.6, b=0.0, a=1.0)
        marker.pose.orientation.w = 1.0

        cx, cy = CAR_POSITION
        marker.points.append(Point(x=float(cx), y=float(cy), z=0.02))
        marker.points.append(Point(x=float(target[0]), y=float(target[1]), z=0.02))
        self.pub_line.publish(marker)

    def main_loop(self):
        if not self.path:
            return

        # ★★★ 변경점: 가변 LAD 계산 함수 호출 ★★★
        lad = self.calculate_variable_lad()
        target = self.find_target(lad)
        steering_angle = self.compute_steering(target)

        # publish steering angle in degrees
        steering_angle_deg = math.degrees(steering_angle)
        steering_angle_deg = float(np.clip(steering_angle_deg, -MAX_STEERING, MAX_STEERING))
        speed = float(self.SPEED)
        rpm = self.compute_speed2rpm(speed)
        self.drive(steering_angle_deg, rpm)

        self.vis_lad(lad)
        self.vis_target(target)
        self.vis_str(target)

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