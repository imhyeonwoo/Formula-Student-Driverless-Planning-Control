#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_name = 'local_speed_planner'
    share_dir = get_package_share_directory(pkg_name)
    # config_path = os.path.join(share_dir, 'config', 'steering_course.yaml') # 조향 코스
    # config_path = os.path.join(share_dir, 'config', 'accel_course.yaml')  # 가속 코스
    config_path = os.path.join(share_dir, 'config', 'full_course.yaml')   # 종합 주행

    path_sampler_node = Node(
        package=pkg_name,
        executable='path_sampler.py',
        name='path_sampler',
        output='screen',
        parameters=[config_path],
    )

    speed_zone_node = Node(
        package=pkg_name,
        executable='speed_zone_planner.py',
        name='speed_zone_planner',
        output='screen',
        parameters=[config_path],
    )

    return LaunchDescription([
        path_sampler_node,
        speed_zone_node,
    ])

