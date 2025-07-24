from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # ───────────────── 기준 좌표 파라미터 선언 ─────────────────
    # konkuk_250721 : 37.542109, 127.078148
    # administartor_250721 : 37.543116, 127.076076
    # engineering_250721 : 37.541591, 127.079700
    # ilgam_250721 : 37.540190, 127.076488
    ref_lat_arg = DeclareLaunchArgument('ref_lat', default_value='37.540190')
    ref_lon_arg = DeclareLaunchArgument('ref_lon', default_value='127.076488')
    # 여기서 ref lat/lon 수정 시 -> publish_global_cones.py에서도 동일하게 수정 필요

    ref_lat = LaunchConfiguration('ref_lat')
    ref_lon = LaunchConfiguration('ref_lon')

    # ───────────────── CSV 기본 경로 구성 ───────────────────────
    csv_default = os.path.join(
        get_package_share_directory('gps_global_planner'),
        'data', 'ilgam_250721.csv'
    )

    csv_arg      = DeclareLaunchArgument('csv_file', default_value=csv_default)
    csv_file     = LaunchConfiguration('csv_file')

    # ★ covariance threshold 파라미터 추가 (기본 0.0025 -> 5cm)
    cov_thr_arg  = DeclareLaunchArgument('cov_threshold', default_value='0.0025')
    cov_threshold = LaunchConfiguration('cov_threshold')

    # ───────────────── 노드들 선언 ───────────────────────────────
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
            'csv_filename'  : csv_file,
            'ref_lat'       : ref_lat,
            'ref_lon'       : ref_lon,
            # ★ threshold 전달
            'cov_threshold' : cov_threshold
        }]
    )

    tf_broadcaster = Node(
        package='gps_global_planner',
        executable='vehicle_tf_broadcaster_node',
        name='vehicle_tf_broadcaster'
    )

    return LaunchDescription([
        ref_lat_arg, ref_lon_arg,
        csv_arg, cov_thr_arg,          # ★ LaunchDescription에 추가
        gps_to_local,
        path_marker,
        tf_broadcaster
    ])
