# launch/cone_labeling_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1) cone_classifier 노드
        Node(
            package='cone_labeling',
            executable='cone_classifier',
            name='cone_classifier',
            output='screen',
        ),

        # 2) cone_delaunay_connector 노드
        Node(
            package='cone_labeling',
            executable='cone_delaunay_connector',
            name='cone_delaunay_connector',
            output='screen',
        ),

        # 3) midpoint_bspline_interpolator 노드
        Node(
            package='cone_labeling',
            executable='midpoint_bspline_interpolator',
            name='midpoint_bspline_interpolator',
            output='screen',
        ),

        # 4) speed_planning 노드 (곡률 기반 속도 플래너)
        # Node(
        #     package='cone_labeling',
        #     executable='speed_planning',
        #     name='speed_planning',
        #     output='screen',
        # ),
    ])

