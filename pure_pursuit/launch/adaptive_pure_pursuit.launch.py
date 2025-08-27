from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('pure_pursuit')
    param_file = os.path.join(pkg_share, 'config', 'adaptive_pp.yaml')

    return LaunchDescription([
        Node(
            package='pure_pursuit',
            executable='pure_pursuit_adaptive_node',
            name='pure_pursuit_adaptive',
            output='screen',
            parameters=[param_file],
        )
    ])
