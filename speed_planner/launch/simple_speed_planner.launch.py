from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('speed_planner')
    default_params = os.path.join(pkg_share, 'config', 'simple_speed_planner.yaml')

    # 기존: 파라미터 파일 경로
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to the YAML file with ROS2 parameters.'
    )

    # 추가: 런치에서 편하게 덮어쓸 수 있는 주요 파라미터들(선택사항)
    wheel_d_arg = DeclareLaunchArgument(
        'wheel_diameter_m', default_value='0.47', description='Wheel diameter (m)'
    )
    gear_arg = DeclareLaunchArgument(
        'gear_ratio', default_value='4.6', description='Motor:wheel gear ratio'
    )
    desired_rpm_topic_arg = DeclareLaunchArgument(
        'desired_rpm_topic', default_value='/cmd/rpm', description='Desired RPM topic'
    )
    desired_speed_topic_arg = DeclareLaunchArgument(
        'desired_speed_topic', default_value='/cmd/speed', description='Desired speed topic (m/s)'
    )
    desired_pub_rate_arg = DeclareLaunchArgument(
        'desired_pub_rate_hz', default_value='50.0', description='Desired command publish rate (Hz)'
    )

    planner_node = Node(
        package='speed_planner',
        executable='simple_speed_planner',
        name='simple_speed_planner',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            # 아래 dict는 YAML 값을 런치 인자로 덮어씌움(선택적으로 사용 가능)
            {
                'wheel_diameter_m': LaunchConfiguration('wheel_diameter_m'),
                'gear_ratio': LaunchConfiguration('gear_ratio'),
                'desired_rpm_topic': LaunchConfiguration('desired_rpm_topic'),
                'desired_speed_topic': LaunchConfiguration('desired_speed_topic'),
                'desired_pub_rate_hz': LaunchConfiguration('desired_pub_rate_hz'),
            }
        ]
    )

    return LaunchDescription([
        params_file_arg,
        wheel_d_arg,
        gear_arg,
        desired_rpm_topic_arg,
        desired_speed_topic_arg,
        desired_pub_rate_arg,
        planner_node
    ])
