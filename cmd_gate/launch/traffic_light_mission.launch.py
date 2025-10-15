#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # --- Local Speed Planner (path sampler + speed zone) ---
    lsp_share = get_package_share_directory('local_speed_planner')
    lsp_cfg = os.path.join(lsp_share, 'config', 'full_course.yaml')

    path_sampler = Node(
        package='local_speed_planner',
        executable='path_sampler.py',
        name='path_sampler',
        output='screen',
        parameters=[lsp_cfg],
    )

    speed_zone = Node(
        package='local_speed_planner',
        executable='speed_zone_planner.py',
        name='speed_zone_planner',
        output='screen',
        parameters=[
            lsp_cfg,
            {
                # Override output topics to pre_cmd
                'topic_out': '/pre_cmd/speed',
                'rpm_topic': '/pre_cmd/rpm',
            },
        ],
    )

    # --- Pure Pursuit (choose one variant) ---
    pp_share = get_package_share_directory('pure_pursuit')
    pp_cfg = os.path.join(pp_share, 'config', 'adaptive_full_course.yaml')

    pure_pursuit = Node(
        package='pure_pursuit',
        executable='pure_pursuit_adaptive_node',
        name='pure_pursuit_adaptive',
        output='screen',
        parameters=[
            pp_cfg,
            {
                # Override steer output topic to pre_cmd
                'steer_topic': '/pre_cmd/steer',
            }
        ],
    )

    # --- CMD Gate ---
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
                'auto_open': False,           # Start closed; wait for /green
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
        path_sampler,
        speed_zone,
        pure_pursuit,
        gate_node,
    ])

