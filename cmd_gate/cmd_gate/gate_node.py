#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Float32
from std_srvs.srv import Trigger


class CmdGateNode(Node):
    """
    Gate /cmd/* topics until /green is received.

    - Subscribes:  /pre_cmd/speed, /pre_cmd/rpm, /pre_cmd/steer (Float32)
    - Publishes:   /cmd/speed, /cmd/rpm, /cmd/steer (Float32)
    - Service(s):  /green (Trigger) to enable gate (idempotent)

    Parameters:
      - use_gate (bool, default: True): If False, bypass gate (always pass-through)
      - pre_green_policy (str, default: 'zero'): 'zero' or 'drop'
      - publish_rate_hz (float, default: 50.0): zero publishing rate when policy=='zero'
      - auto_open (bool, default: False): start with gate open (for standalone tests)
      - in_speed_topic (str, default: '/pre_cmd/speed')
      - in_rpm_topic   (str, default: '/pre_cmd/rpm')
      - in_steer_topic (str, default: '/pre_cmd/steer')
      - out_speed_topic (str, default: '/cmd/speed')
      - out_rpm_topic   (str, default: '/cmd/rpm')
      - out_steer_topic (str, default: '/cmd/steer')
    """

    def __init__(self) -> None:
        super().__init__('cmd_gate_node')

        # Parameters
        self.use_gate = self.declare_parameter('use_gate', True).get_parameter_value().bool_value
        self.pre_green_policy = self.declare_parameter('pre_green_policy', 'zero').get_parameter_value().string_value
        self.publish_rate_hz = self.declare_parameter('publish_rate_hz', 50.0).get_parameter_value().double_value
        self.auto_open = self.declare_parameter('auto_open', False).get_parameter_value().bool_value

        self.in_speed_topic = self.declare_parameter('in_speed_topic', '/pre_cmd/speed').get_parameter_value().string_value
        self.in_rpm_topic = self.declare_parameter('in_rpm_topic', '/pre_cmd/rpm').get_parameter_value().string_value
        self.in_steer_topic = self.declare_parameter('in_steer_topic', '/pre_cmd/steer').get_parameter_value().string_value

        self.out_speed_topic = self.declare_parameter('out_speed_topic', '/cmd/speed').get_parameter_value().string_value
        self.out_rpm_topic = self.declare_parameter('out_rpm_topic', '/cmd/rpm').get_parameter_value().string_value
        self.out_steer_topic = self.declare_parameter('out_steer_topic', '/cmd/steer').get_parameter_value().string_value

        # Gate state
        self._enabled_lock = threading.Lock()
        self._enabled: bool = bool(self.auto_open) or (not self.use_gate)

        # QoS: best effort/history keep last is enough for commands
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers
        self.pub_speed = self.create_publisher(Float32, self.out_speed_topic, qos)
        self.pub_rpm = self.create_publisher(Float32, self.out_rpm_topic, qos)
        self.pub_steer = self.create_publisher(Float32, self.out_steer_topic, qos)

        # Subscribers
        self.sub_speed = self.create_subscription(Float32, self.in_speed_topic, self.cb_speed, qos)
        self.sub_rpm = self.create_subscription(Float32, self.in_rpm_topic, self.cb_rpm, qos)
        self.sub_steer = self.create_subscription(Float32, self.in_steer_topic, self.cb_steer, qos)

        # Services
        self.srv_green = self.create_service(Trigger, '/green', self.srv_green_cb)

        # Timers (for zero policy)
        self.zero_timer = None  # type: Optional[rclpy.timer.Timer]
        if self.pre_green_policy == 'zero' and self.use_gate:
            period = 1.0 / max(1e-3, self.publish_rate_hz)
            self.zero_timer = self.create_timer(period, self.cb_zero_timer)

        self.get_logger().info(
            f"CMD Gate started | use_gate={self.use_gate}, auto_open={self.auto_open}, policy={self.pre_green_policy}"
        )

    # ------------- Utility -------------
    def is_enabled(self) -> bool:
        with self._enabled_lock:
            return self._enabled

    def set_enabled(self, value: bool) -> None:
        with self._enabled_lock:
            self._enabled = bool(value)

    # ------------- Callbacks -------------
    def srv_green_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if not self.use_gate:
            response.success = True
            response.message = 'Gate bypass mode: already enabled.'
            return response

        if not self.is_enabled():
            self.set_enabled(True)
            self.get_logger().info('✅ /green received: gate ENABLED (commands pass-through).')
            response.success = True
            response.message = 'Gate enabled.'
        else:
            response.success = True
            response.message = 'Gate already enabled.'
        return response

    def cb_speed(self, msg: Float32) -> None:
        if not self.use_gate or self.is_enabled():
            self.pub_speed.publish(msg)
        else:
            if self.pre_green_policy == 'drop':
                # drop silently
                return
            # zero policy handled by zero timer; no action here

    def cb_rpm(self, msg: Float32) -> None:
        if not self.use_gate or self.is_enabled():
            self.pub_rpm.publish(msg)
        else:
            if self.pre_green_policy == 'drop':
                return

    def cb_steer(self, msg: Float32) -> None:
        if not self.use_gate or self.is_enabled():
            self.pub_steer.publish(msg)
        else:
            if self.pre_green_policy == 'drop':
                return

    def cb_zero_timer(self) -> None:
        if not self.use_gate:
            return
        if self.is_enabled():
            return
        # publish zeros to keep actuators safe
        z = Float32(data=0.0)
        self.pub_speed.publish(z)
        self.pub_rpm.publish(z)
        self.pub_steer.publish(z)


def main(args=None):
    rclpy.init(args=args)
    node = CmdGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down cmd_gate_node (KeyboardInterrupt)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

