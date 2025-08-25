from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('speed_planner')
    default_params = os.path.join(pkg_share, 'config', 'simple_speed_planner.yaml')


    params_file_arg = DeclareLaunchArgument(
    'params_file',
    default_value=default_params,
    description='Path to the YAML file with ROS2 parameters.'
    )


    planner_node = Node(
    package='speed_planner',
    executable='simple_speed_planner',
    name='simple_speed_planner',
    output='screen',
    parameters=[LaunchConfiguration('params_file')]
    )


    return LaunchDescription([
    params_file_arg,
    planner_node
    ])