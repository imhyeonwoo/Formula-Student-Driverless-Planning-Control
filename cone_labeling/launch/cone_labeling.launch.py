from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # cone_classifier 노드 실행
        Node(
            package='cone_labeling',
            executable='cone_classifier',
            name='cone_classifier',
            output='screen',
            parameters=[
                # 필요한 파라미터가 있으면 여기에 추가
            ],
        ),
        # cone_delaunay_connector 노드 실행
        Node(
            package='cone_labeling',
            executable='cone_delaunay_connector',
            name='cone_delaunay_connector',
            output='screen',
        ),
        # midpoint_bspline_interpolator 노드 실행
        Node(
            package='cone_labeling',
            executable='midpoint_bspline_interpolator',
            name='midpoint_bspline_interpolator',
            output='screen',
        ),
    ])

