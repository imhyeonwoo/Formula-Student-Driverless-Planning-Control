from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node           # ← 여기!

import multiprocessing


def generate_launch_description():
    # 필요 시 숫자를 직접 넣어도 됩니다 (예: n_threads = 8)
    n_threads = multiprocessing.cpu_count()

    return LaunchDescription([
        # OpenMP 환경변수
        SetEnvironmentVariable('OMP_NUM_THREADS', str(n_threads)),
        SetEnvironmentVariable('OMP_PROC_BIND',  'spread'),

        # C++ 노드
        Node(
            package='reference_path_classifier',
            executable='classify_cones_by_side',
            output='screen'
        )
    ])
