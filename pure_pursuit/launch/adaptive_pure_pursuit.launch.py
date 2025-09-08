from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('pure_pursuit')
    # Param file (current default)
    param_file = os.path.join(pkg_share, 'config', 'adaptive_steering_course.yaml')
    # param_file = os.path.join(pkg_share, 'config', 'adaptive_pp.yaml')
    use_constant_cmd = LaunchConfiguration('use_constant_cmd')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_constant_cmd', default_value='true',
            description='Launch constant_cmd_publisher to publish /cmd/speed and /cmd/rpm'
        ),
        Node(
            package='pure_pursuit',
            executable='pure_pursuit_adaptive_node',
            name='pure_pursuit_adaptive',
            output='screen',
            parameters=[param_file],
        ),
        # Optional constant publishers for /cmd/speed and /cmd/rpm
        # 상수 속도 안 박을거면 이 노드 부분 주석 처리하기!!!!!!
        # Node(
        #     package='pure_pursuit',
        #     executable='constant_cmd_publisher',
        #     name='constant_cmd_publisher',
        #     output='screen',
        #     parameters=[param_file],
        #     condition=IfCondition(use_constant_cmd),
        # ),
    ])
