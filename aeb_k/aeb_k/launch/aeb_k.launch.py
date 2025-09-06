from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aeb_k',
            executable='aeb_k',
            name='AEB_Determination_Node',
            output='screen',
        )
    ])

