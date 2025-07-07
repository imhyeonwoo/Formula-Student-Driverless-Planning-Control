from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # ───────────────── 기준 좌표 파라미터 선언
    # 127.0505869,37.5573749
    ref_lat_arg = DeclareLaunchArgument('ref_lat', default_value='37.5573749')
    ref_lon_arg = DeclareLaunchArgument('ref_lon', default_value='127.0505869')
    # ref_lat_arg = DeclareLaunchArgument('ref_lat', default_value='37.54995')
    # ref_lon_arg = DeclareLaunchArgument('ref_lon', default_value='127.05485')

    ref_lat = LaunchConfiguration('ref_lat')
    ref_lon = LaunchConfiguration('ref_lon')

    # ───────────────── CSV 기본 경로 구성
    csv_default = os.path.join(
        get_package_share_directory('gps_global_planner'),
        'data', 'nocheon_250613_curved_rtk_1.csv'
    )

    csv_arg = DeclareLaunchArgument(
        'csv_file',
        default_value=csv_default
    )
    csv_file = LaunchConfiguration('csv_file')

    # ───────────────── 노드들 선언
    gps_to_local = Node(
        package='gps_global_planner',
        executable='gps_to_local_cartesian_node',
        name='gps_to_local_cartesian',
        parameters=[{
            'ref_lat': ref_lat,
            'ref_lon': ref_lon
        }]
    )

    path_marker = Node(
        package='gps_global_planner',
        executable='status_colored_path_publisher_node',
        name='status_colored_path_publisher',
        parameters=[{
            'csv_filename': csv_file,
            'ref_lat': ref_lat,
            'ref_lon': ref_lon
        }]
    )

    tf_broadcaster = Node(
        package='gps_global_planner',
        executable='vehicle_tf_broadcaster_node',
        name='vehicle_tf_broadcaster'
    )

    return LaunchDescription([
        ref_lat_arg, ref_lon_arg, csv_arg,
        gps_to_local,
        path_marker,
        tf_broadcaster
    ])