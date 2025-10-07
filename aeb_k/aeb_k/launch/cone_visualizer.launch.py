from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aeb_k',
            executable='cone_visualizer',
            name='cone_visualizer',
            output='screen',
            parameters=[{
                'topic': '/cone/fused',
                'frame_id_override': '',
                'scale': 0.3,
                'alpha': 0.9,
                'publish_text_ids': False,
            }]
        )
    ])

