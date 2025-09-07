from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('pure_pursuit')
    default_params = os.path.join(pkg_share, 'config', 'pure_pursuit_static.yaml')

    params_file = LaunchConfiguration('params_file')
    use_constant_cmd = LaunchConfiguration('use_constant_cmd')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to the parameter YAML file'
        ),
        DeclareLaunchArgument(
            'use_constant_cmd',
            default_value='true',
            description='Launch constant_cmd_publisher to publish /cmd/speed and /cmd/rpm'
        ),

        Node(
            package='pure_pursuit',
            executable='pure_pursuit_static',
            name='pure_pursuit_static',
            output='screen',
            parameters=[params_file],
            # 필요 시 여기서 remappings=[('in_topic','/local_planned_path'), ...] 추가 가능
        ),

        # Optional constant publishers for /cmd/speed and /cmd/rpm
        # 상수 속도 노드 실행 유무는 use_constant_cmd로 제어 (adaptive와 동일)
        Node(
            package='pure_pursuit',
            executable='constant_cmd_publisher',
            name='constant_cmd_publisher',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(use_constant_cmd),
        ),
    ])
