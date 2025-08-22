from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cone_labeling_k',
            executable='potential_field',
            name='potential_field_node',
            output='screen',
        )
    ])

