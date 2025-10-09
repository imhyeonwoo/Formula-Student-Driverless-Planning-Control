from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('pure_pursuit')
    default_params = os.path.join(pkg_share, 'config', 'pure_pursuit_static_damped.yaml')

    from launch.substitutions import LaunchConfiguration
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to the parameter YAML file'
        ),

        Node(
            package='pure_pursuit',
            executable='pure_pursuit_static_damped',
            name='pure_pursuit_static_damped',
            output='screen',
            parameters=[params_file],
            # remappings=[('in_topic','/local_planned_path'), ...]  # 필요 시 설정
        ),
    ])

