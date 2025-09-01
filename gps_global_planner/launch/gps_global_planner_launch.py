# file: src/Planning/gps_global_planner/launch/gps_global_planner_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # ───────────── 공통 인자 ─────────────
    enable_hud_arg = DeclareLaunchArgument('enable_hud', default_value='true')
    enable_hud = LaunchConfiguration('enable_hud')

    # ───────────── 기준 좌표 ─────────────
    ref_lat_arg = DeclareLaunchArgument('ref_lat', default_value='37.237394')
    ref_lon_arg = DeclareLaunchArgument('ref_lon', default_value='126.770827')
    ref_lat = LaunchConfiguration('ref_lat')
    ref_lon = LaunchConfiguration('ref_lon')
    # 기본 좌표는 건국대학교 일감호 와우도 기준 : (37.540091, 127.076555)
    # 대회장 레퍼런스 좌표 : (37.237394, 126.770827)

    # ───────────── CSV 기본 경로 ─────────────
    csv_default = os.path.join(
        get_package_share_directory('gps_global_planner'),
        'data', '4planning_250830_curved.csv'   # 조향 코스
        # 'data', '4planning_250830_straight.csv'   # 가속 코스
    )
    csv_arg  = DeclareLaunchArgument('csv_file', default_value=csv_default)
    csv_file = LaunchConfiguration('csv_file')

    # covariance threshold
    cov_thr_arg   = DeclareLaunchArgument('cov_threshold', default_value='0.0025')
    cov_threshold = LaunchConfiguration('cov_threshold')

    # ───────────── 오프셋/높이 ─────────────
    ant_x_arg = DeclareLaunchArgument('antenna_offset_x', default_value='0.9')
    ant_y_arg = DeclareLaunchArgument('antenna_offset_y', default_value='0.0')
    ant_z_arg = DeclareLaunchArgument('antenna_offset_z', default_value='1.0')
    antenna_offset_x = LaunchConfiguration('antenna_offset_x')
    antenna_offset_y = LaunchConfiguration('antenna_offset_y')
    antenna_offset_z = LaunchConfiguration('antenna_offset_z')

    sen_x_arg = DeclareLaunchArgument('sensor_offset_x', default_value='0.5')
    sen_y_arg = DeclareLaunchArgument('sensor_offset_y', default_value='0.0')
    sen_z_arg = DeclareLaunchArgument('sensor_offset_z', default_value='1.295')
    sensor_offset_x = LaunchConfiguration('sensor_offset_x')
    sensor_offset_y = LaunchConfiguration('sensor_offset_y')
    sensor_offset_z = LaunchConfiguration('sensor_offset_z')

    # base_link z 고정 옵션
    fix_z_arg   = DeclareLaunchArgument('fix_base_z_to_zero', default_value='true')
    base_z_arg  = DeclareLaunchArgument('base_z_level',       default_value='0.0')
    fix_base_z_to_zero = LaunchConfiguration('fix_base_z_to_zero')
    base_z_level       = LaunchConfiguration('base_z_level')

    # ───────────── 노드들 ─────────────
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

    # Static transform: map -> odom (identity transform)
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{}]
    )

    tf_broadcaster = Node(
        package='gps_global_planner',
        executable='vehicle_tf_broadcaster_node',
        name='vehicle_tf_broadcaster',
        parameters=[{
            'antenna_offset_x': antenna_offset_x,
            'antenna_offset_y': antenna_offset_y,
            'antenna_offset_z': antenna_offset_z,
            'sensor_offset_x' : sensor_offset_x,
            'sensor_offset_y' : sensor_offset_y,
            'sensor_offset_z' : sensor_offset_z,
            'fix_base_z_to_zero': fix_base_z_to_zero,
            'base_z_level'      : base_z_level,
        }]
    )

    car_marker = Node(
        package='gps_global_planner',
        executable='car_marker_publisher.py',
        name='car_marker_publisher',
        output='screen',
        parameters=[{}]
    )

    # ───────────── HUD 오버레이 (Python 스크립트) ─────────────
    hud_overlay = Node(
        package='gps_global_planner',
        executable='hud_overlay_node.py',      # scripts/hud_overlay_node.py
        name='hud_overlay_node',
        output='screen',
        parameters=[{}],
        condition=IfCondition(enable_hud)
    )

    return LaunchDescription([
        # 스위치
        enable_hud_arg,

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
        tf_broadcaster,
        car_marker,
        map_to_odom,
        hud_overlay,
    ])
