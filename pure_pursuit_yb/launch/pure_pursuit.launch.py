import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 파라미터 파일 경로 설정
    param_file = os.path.join(
        get_package_share_directory('pure_pursuit'),
        'config',
        'pure_pursuit_params.yaml'
    )

    # 2. Pure Pursuit 노드 실행 설정
    pure_pursuit_node = Node(
        package='pure_pursuit',
        executable='pure_pursuit',
        name='pure_pursuit',
        parameters=[param_file],
        output='screen'
    )

    return LaunchDescription([
        pure_pursuit_node
    ])
