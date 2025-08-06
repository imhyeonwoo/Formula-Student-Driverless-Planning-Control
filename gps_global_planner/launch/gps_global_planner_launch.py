# file: src/Planning/gps_global_planner/launch/gps_global_planner_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # ───────────────── 기준 좌표 파라미터 선언 ─────────────────
    ref_lat_arg = DeclareLaunchArgument('ref_lat', default_value='37.540190')
    ref_lon_arg = DeclareLaunchArgument('ref_lon', default_value='127.076488')
    ref_lat = LaunchConfiguration('ref_lat')
    ref_lon = LaunchConfiguration('ref_lon')

    # ───────────────── CSV 기본 경로 구성 ───────────────────────
    csv_default = os.path.join(
        get_package_share_directory('gps_global_planner'),
        'data', 'administrator_250721.csv'
    )
    csv_arg       = DeclareLaunchArgument('csv_file', default_value=csv_default)
    csv_file      = LaunchConfiguration('csv_file')

    # covariance threshold
    cov_thr_arg   = DeclareLaunchArgument('cov_threshold', default_value='0.0025')
    cov_threshold = LaunchConfiguration('cov_threshold')

    # ───────────────── 오프셋/높이 고정 파라미터 선언 ─────────────────
    # base_link : 차량 후륜축 중심
    # GPS Antenna 위치(base_link -> gps_antenna)
    ant_x_arg = DeclareLaunchArgument('antenna_offset_x', default_value='1.7')
    ant_y_arg = DeclareLaunchArgument('antenna_offset_y', default_value='0.0')
    ant_z_arg = DeclareLaunchArgument('antenna_offset_z', default_value='3.0')
    antenna_offset_x = LaunchConfiguration('antenna_offset_x')
    antenna_offset_y = LaunchConfiguration('antenna_offset_y')
    antenna_offset_z = LaunchConfiguration('antenna_offset_z')

    # LiDAR 위치(base_link -> os_sensor)
    sen_x_arg = DeclareLaunchArgument('sensor_offset_x', default_value='0.0')
    sen_y_arg = DeclareLaunchArgument('sensor_offset_y', default_value='0.0')
    sen_z_arg = DeclareLaunchArgument('sensor_offset_z', default_value='2.0')
    sensor_offset_x = LaunchConfiguration('sensor_offset_x')
    sensor_offset_y = LaunchConfiguration('sensor_offset_y')
    sensor_offset_z = LaunchConfiguration('sensor_offset_z')

    # base_link z를 지면으로 고정할지 여부(옵션)
    fix_z_arg   = DeclareLaunchArgument('fix_base_z_to_zero', default_value='true')
    base_z_arg  = DeclareLaunchArgument('base_z_level',       default_value='0.0')
    fix_base_z_to_zero = LaunchConfiguration('fix_base_z_to_zero')
    base_z_level       = LaunchConfiguration('base_z_level')

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
            'cov_threshold' : cov_threshold
        }]
    )

    tf_broadcaster = Node(
        package='gps_global_planner',
        executable='vehicle_tf_broadcaster_node',  # 빌드된 실행파일명에 맞춰주세요
        name='vehicle_tf_broadcaster',
        parameters=[{
            # offsets
            'antenna_offset_x': antenna_offset_x,
            'antenna_offset_y': antenna_offset_y,
            'antenna_offset_z': antenna_offset_z,
            'sensor_offset_x' : sensor_offset_x,
            'sensor_offset_y' : sensor_offset_y,
            'sensor_offset_z' : sensor_offset_z,
            # z policy
            'fix_base_z_to_zero': fix_base_z_to_zero,
            'base_z_level'      : base_z_level,
        }]
    )

    return LaunchDescription([
        # 기본 파라미터 인자
        ref_lat_arg, ref_lon_arg,
        csv_arg, cov_thr_arg,

        # 오프셋 인자
        ant_x_arg, ant_y_arg, ant_z_arg,
        sen_x_arg, sen_y_arg, sen_z_arg,

        # z 고정 정책 인자
        fix_z_arg, base_z_arg,

        # 노드들
        gps_to_local,
        path_marker,
        tf_broadcaster
    ])
