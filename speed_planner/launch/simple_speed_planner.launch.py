from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='speed_planner',
            executable='simple_speed_planner',
            name='simple_speed_planner',
            output='screen',
            parameters=['config/simple_speed_planner.yaml']
        )
    ])
