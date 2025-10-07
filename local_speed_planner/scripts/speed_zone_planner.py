#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, UInt8


class SpeedZonePlanner(Node):
    def __init__(self):
        super().__init__('speed_zone_planner')

        # Parameters
        self.declare_parameter('topic_in', '/sp/rep_curvature')
        self.declare_parameter('topic_out', '/cmd/speed')
        # New: RPM publishing params
        self.declare_parameter('rpm_topic', '/cmd/rpm')
        self.declare_parameter('publish_rpm', True)
        # AEB override params
        self.declare_parameter('aeb_topic', '/aeb')
        self.declare_parameter('enforce_aeb_stop', True)
        # Conversion: v[m/s] = RPM_motor * rpm_divisor  =>  RPM = v / rpm_divisor
        # Derived from: RPM_wheel = RPM_motor / 4.6, circumference = pi * 0.47 m
        # v = RPM_motor * (1/4.6) * (1/60) * (pi * 0.47) ≈ RPM_motor * 0.00535 [m/s]
        self.declare_parameter('rpm_divisor', 0.00535)
        self.declare_parameter('publish_rate_hz', 20.0)

        # Zone thresholds (curvature [1/m])
        # Increasing kappa means tighter curve -> lower speed
        self.declare_parameter('thr_hm_enter', 0.12)  # HIGH->MID enter
        self.declare_parameter('thr_ml_enter', 0.22)  # MID->LOW enter
        self.declare_parameter('hysteresis_factor', 0.8)  # exit = enter * factor
        self.declare_parameter('min_hold_sec', 0.7)

        # Zone speeds (m/s)
        self.declare_parameter('v_high', 10.0)
        self.declare_parameter('v_mid', 7.0)
        self.declare_parameter('v_low', 4.0)

        # Ramp limits
        self.declare_parameter('a_acc_max', 2.0)  # m/s^2
        self.declare_parameter('a_dec_max', 3.0)  # m/s^2

        # Load parameters
        self.topic_in = self.get_parameter('topic_in').value
        self.topic_out = self.get_parameter('topic_out').value
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        # RPM params
        self.rpm_topic = self.get_parameter('rpm_topic').value
        self.publish_rpm = bool(self.get_parameter('publish_rpm').value)
        self.rpm_divisor = float(self.get_parameter('rpm_divisor').value)
        # AEB params
        self.aeb_topic = self.get_parameter('aeb_topic').value
        self.enforce_aeb_stop = bool(self.get_parameter('enforce_aeb_stop').value)

        self.thr_hm_enter = float(self.get_parameter('thr_hm_enter').value)
        self.thr_ml_enter = float(self.get_parameter('thr_ml_enter').value)
        self.hyst = float(self.get_parameter('hysteresis_factor').value)
        self.min_hold_sec = float(self.get_parameter('min_hold_sec').value)

        self.thr_hm_exit = self.thr_hm_enter * self.hyst
        self.thr_ml_exit = self.thr_ml_enter * self.hyst

        self.v_high = float(self.get_parameter('v_high').value)
        self.v_mid = float(self.get_parameter('v_mid').value)
        self.v_low = float(self.get_parameter('v_low').value)

        self.a_acc_max = float(self.get_parameter('a_acc_max').value)
        self.a_dec_max = float(self.get_parameter('a_dec_max').value)

        # State
        self.last_kappa = None
        self.zone = None  # 'HIGH'|'MID'|'LOW'
        self.last_zone_change = self.get_clock().now()
        self.speed_cmd = 0.0
        self.aeb_active = False

        # ROS I/O
        self.sub = self.create_subscription(Float32, self.topic_in, self.on_kappa, 10)
        self.pub = self.create_publisher(Float32, self.topic_out, 10)
        self.pub_rpm = None
        if self.publish_rpm:
            self.pub_rpm = self.create_publisher(Float32, self.rpm_topic, 10)
        # AEB subscriber
        self.sub_aeb = self.create_subscription(UInt8, self.aeb_topic, self.on_aeb, 10)

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f"SpeedZonePlanner started: in={self.topic_in}, out_speed={self.topic_out}, out_rpm={self.rpm_topic if self.publish_rpm else 'disabled'}, rate={self.publish_rate_hz} Hz")

    def on_kappa(self, msg: Float32):
        self.last_kappa = float(msg.data)

        # Enforce minimum hold time
        now = self.get_clock().now()
        dt_since_change = (now - self.last_zone_change).nanoseconds * 1e-9

        new_zone = self.zone
        k = self.last_kappa

        if self.zone is None:
            # Initial selection
            if k >= self.thr_ml_enter:
                new_zone = 'LOW'
            elif k >= self.thr_hm_enter:
                new_zone = 'MID'
            else:
                new_zone = 'HIGH'
        else:
            if dt_since_change >= self.min_hold_sec:
                if self.zone == 'HIGH':
                    if k >= self.thr_hm_enter:
                        new_zone = 'MID'
                elif self.zone == 'MID':
                    if k >= self.thr_ml_enter:
                        new_zone = 'LOW'
                    elif k <= self.thr_hm_exit:
                        new_zone = 'HIGH'
                elif self.zone == 'LOW':
                    if k <= self.thr_ml_exit:
                        new_zone = 'MID'

        if new_zone != self.zone:
            self.zone = new_zone
            self.last_zone_change = now
            # self.get_logger().info(f"Zone -> {self.zone} (kappa={k:.3f})")

    def on_aeb(self, msg: UInt8):
        # AEB active when data >= 1
        self.aeb_active = int(msg.data) >= 1
        # Optionally, reset ramp state when AEB engages
        if self.aeb_active:
            self.speed_cmd = 0.0

    def on_timer(self):
        # AEB override: force outputs to zero regardless of zone/data
        if self.enforce_aeb_stop and self.aeb_active:
            zero = Float32(); zero.data = 0.0
            self.pub.publish(zero)
            if self.publish_rpm and self.pub_rpm is not None:
                zero_rpm = Float32(); zero_rpm.data = 0.0
                self.pub_rpm.publish(zero_rpm)
            return

        # Determine target speed by zone
        if self.zone == 'HIGH':
            target = self.v_high
        elif self.zone == 'MID':
            target = self.v_mid
        elif self.zone == 'LOW':
            target = self.v_low
        else:
            # No data yet
            return

        # Ramp towards target with accel/deccl limits
        dt = 1.0 / max(self.publish_rate_hz, 1.0)
        if target > self.speed_cmd:
            self.speed_cmd = min(self.speed_cmd + self.a_acc_max * dt, target)
        else:
            self.speed_cmd = max(self.speed_cmd - self.a_dec_max * dt, target)

        # Publish speed (m/s)
        msg = Float32()
        msg.data = float(self.speed_cmd)
        self.pub.publish(msg)

        # Optionally publish RPM derived from speed
        if self.publish_rpm and self.pub_rpm is not None:
            rpm = 0.0
            if self.rpm_divisor > 0.0:
                rpm = float(self.speed_cmd) / self.rpm_divisor
            rpm_msg = Float32()
            rpm_msg.data = float(rpm)
            self.pub_rpm.publish(rpm_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SpeedZonePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
