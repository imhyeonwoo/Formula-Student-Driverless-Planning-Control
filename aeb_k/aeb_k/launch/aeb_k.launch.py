from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aeb_k',
            executable='aeb_k',
            name='AEB_Determination_Node',
            output='screen',
            parameters=[{
                # 'topic': '/cone/fused/ukf',   # align with cone_visualizer default
                'topic': '/cone/fused',
                # Tune thresholds/ROI via launch if needed
                # 'aeb_thresh': 10,
                # 'aeb_hor': 6.0,
                # 'aeb_ver': 9.0,
            }]
        )
    ])
