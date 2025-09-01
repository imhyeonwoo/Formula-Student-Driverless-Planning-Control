from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('pure_pursuit')
    cfg = os.path.join(pkg_share, 'config', 'pure_pursuit_vis.yaml')

    return LaunchDescription([
        Node(
            package='pure_pursuit',
            executable='car_marker_publisher.py',
            name='car_marker_publisher',
            output='screen',
            parameters=[cfg]
        ),
        Node(
            package='pure_pursuit',
            executable='traj_predictor.py',
            name='traj_predictor',
            output='screen',
            parameters=[cfg]
        ),
    ])
