#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    gate_node = Node(
        package='cmd_gate',
        executable='cmd_gate_node',
        name='cmd_gate',
        output='screen',
        parameters=[
            {
                'use_gate': True,
                'pre_green_policy': 'zero',   # 'zero' or 'drop'
                'publish_rate_hz': 50.0,
                'auto_open': False,           # True for standalone quick tests
                'in_speed_topic': '/pre_cmd/speed',
                'in_rpm_topic': '/pre_cmd/rpm',
                'in_steer_topic': '/pre_cmd/steer',
                'out_speed_topic': '/cmd/speed',
                'out_rpm_topic': '/cmd/rpm',
                'out_steer_topic': '/cmd/steer',
            }
        ],
    )

    return LaunchDescription([
        gate_node
    ])

