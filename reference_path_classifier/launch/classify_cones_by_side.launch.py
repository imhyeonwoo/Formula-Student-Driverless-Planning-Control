# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    densify_step = LaunchConfiguration('densify_step')
    max_gap      = LaunchConfiguration('max_gap')
    n_max        = LaunchConfiguration('n_max')
    path_step    = LaunchConfiguration('path_step')
    merge_radius = LaunchConfiguration('merge_radius')

    return LaunchDescription([
        DeclareLaunchArgument('densify_step', default_value='0.5'),
        DeclareLaunchArgument('max_gap',      default_value='1.2'),
        DeclareLaunchArgument('n_max',        default_value='6.0'),
        DeclareLaunchArgument('path_step',    default_value='0.20'),
        DeclareLaunchArgument('merge_radius', default_value='0.20'),

        # 1) 좌/우 분류 노드: 출력은 /real 로 리매핑
        Node(
            package='reference_path_classifier',
            executable='classify_cones_by_side',
            name='classify_cones_by_side',
            output='screen',
            remappings=[
                ('/left_cone_marker',  '/left_cone_marker/real'),
                ('/right_cone_marker', '/right_cone_marker/real'),
            ],
        ),

        # 2) 가상 콘 보간 노드: /real 입력 → /virtual 출력
        Node(
            package='reference_path_classifier',
            executable='virtual_cone_densifier',
            name='virtual_cone_densifier',
            output='screen',
            parameters=[{
                'densify_step': densify_step,
                'max_gap': max_gap,
                'n_max': n_max,
                'path_step': path_step,
            }],
        ),

        # 3) 머지 노드: /{real,virtual} → 기본 토픽(/left|right_cone_marker)
        Node(
            package='reference_path_classifier',
            executable='cone_marker_merger',
            name='cone_marker_merger',
            output='screen',
            parameters=[{
                'r_merge': merge_radius,
            }],
        ),
    ])
