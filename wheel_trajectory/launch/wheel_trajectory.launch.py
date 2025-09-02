from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('wheel_trajectory')
    cfg = os.path.join(pkg, 'config', 'wheel_trajectory.yaml')

    return LaunchDescription([
        Node(
            package='wheel_trajectory',
            executable='wheel_trajectory_node',
            name='wheel_trajectory',
            output='screen',
            parameters=[cfg],
        )
    ])
